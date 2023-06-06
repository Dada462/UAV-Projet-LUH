#!/usr/bin/env python

import re

import numpy as np
import pyproj
import rospy as rp
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, PointStamped
# from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu, MagneticField

ROS_LOG_COMMON_PREFIX_LENGTH = len("[] [xxxxxxxxxx.xxxxxx]: ")
ROS_LOG_PREFIX_LENGTH = lambda log_type: len(str(log_type)) + ROS_LOG_COMMON_PREFIX_LENGTH
ROS_INFO_PREFIX_LENGTH = ROS_LOG_PREFIX_LENGTH("info")

class GeoHelper:
    
    @staticmethod
    def wrap_around_angle(angle: float):
        return (angle + 180.) % 360. - 180.
    
    @classmethod
    def latlong2utm_zone(cls, latitude: float, longitude: float):
        
        latitude = cls.wrap_around_angle(latitude)
        longitude = cls.wrap_around_angle(longitude)
        
        # general case
        if latitude < 72.0 or latitude > 84.0 or longitude < 0.0 or longitude > 42.0:
            return (int((longitude + 180.) / 6) % 60) + 1
        # edge cases
        elif latitude >= 56.0 and latitude < 64.0 and longitude >= 3.0 and longitude < 12.0:
            return 32
        else:
            if longitude < 9.0:
                return 31
            elif longitude < 21.0:
                return 33
            elif longitude < 33.0:
                return 35
            else:
                return 37
            
    @classmethod
    def lat2hemisphere(cls, latitude: float):
        
        # latitude is positive for north, negative for south (corrected for 360 degree wrap-around)
        return "N" if cls.wrap_around_angle(latitude) > 0 else "S"
    
    GRS80_REGEX = re.compile(r"ETRS[ -_]?(19)?89|NAD[ -_]?(19)?83|GRS[ -_]?(19)?80|POSGAR[ -_]?(19)?9[48]|ITRF|IERS")
    
    @classmethod
    def get_ellipsoid(cls, crs: str):
        
        crs = crs.upper()
        
        if cls.GRS80_REGEX.match(crs):
            return "GRS80"
        else:
            return "WGS84"


def unpack_msg(msg):    
    return [getattr(msg, slot) for slot in msg.__slots__]


class World2MapTransformer:
    
    def __init__(self, aggregation_window: float = 10.):
        
        rp.init_node("world2map_transformer", log_level=rp.INFO)
        self._gps_sub = rp.Subscriber("/mavros/global_position/global", NavSatFix, self._gps_aggregation_callback)
        # self._compass_sub = rp.Subscriber("/mavros/global_position/compass_hdg", Float64, self._compass_callback)
        self._magn_field_sub = rp.Subscriber("/mavros/imu/mag", MagneticField, self._magn_field_aggregation_callback)
        self._imu_sub = rp.Subscriber("/mavros/imu/data", Imu, self._imu_aggregation_callback)
        
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self._gps_positions = []
        self._mean_gps_position = None
        self._gps_variances = []
        # self._compass_headings = []
        self._magn_fields = []
        # self._mean_compass_heading = None
        self._mean_magn_field = None
        self._imu_data = []
        self._mean_imu_data = None
        self._imu_variances = []
        
        self._aggregation_window = aggregation_window
        self._transformer = None
        self._map_orientation = None
        
        self._aggregate_data()
    
    def validate_crs(self, crs: str) -> pyproj.CRS:
        """
        Converts a CRS string to a pyproj CRS object. Works for EPSG codes and UTM.
        For UTM, zone and hemisphere are determined from the GPS position. 
        Corresponding ellipsoid is determined from an CRS identifier in string (defaults to WGS84).

        :param crs: CRS string, e.g. "EPSG:4326" or "UTM ETRS89"
        :type crs: str
        :raises NotImplementedError: raises error if CRS is not supported (non-EPSG or non-UTM)
        :return: pyproj CRS object obtained from given string (and optionally GPS position)
        :rtype: pyproj.CRS
        """
        
        crs = crs.upper()
        if crs.startswith("EPSG:"):
            crs = pyproj.CRS.from_epsg(int(crs[5:]))
        elif "UTM" in crs:
            assert self._mean_gps_position is not None, \
                f"Mean GPS position must be available to determine UTM zone for CRS {crs}. " \
                "Call _aggregate_data() first or provide EPSG coordinate reference system."
            
            mean_longitude, mean_latitude, _ = self._mean_gps_position
            crs = pyproj.CRS.from_dict({"proj": "utm", 
                                        "zone": GeoHelper.latlong2utm_zone(mean_latitude, mean_longitude),
                                        "south": GeoHelper.lat2hemisphere(mean_latitude) == "S",
                                        "ellps": GeoHelper.get_ellipsoid(crs)})
        else:
            raise NotImplementedError(f"CRS {crs} is not supported.")
        
        return crs
        
    def set_geodetic_transform(self, crs_src: str, crs_tgt: str):
        """
        Set coordinate reference systems for geodetic transformation. Works for EPSG codes and UTM. 
        For UTM, zone and hemisphere are determined from the GPS position. 
        Corresponding ellipsoid is determined from an CRS identifier in string (defaults to WGS84).

        :param crs0: source CRS string, e.g. "EPSG:4326" or "UTM ETRS89"
        :type crs0: str
        :param crs1: target CRS string, e.g. "EPSG:25833" or "UTM NAD-83"
        :type crs1: str
        """
        
        crs_src = self.validate_crs(crs_src)
        crs_tgt = self.validate_crs(crs_tgt)
        # check if crs1 is projected (otherwise cannot be used for cartesian ROS coordinates)
        assert crs_tgt.is_projected, f"Target CRS must be projected. Current target CRS: {crs_tgt}"
        self._transformer = pyproj.Transformer.from_crs(crs_src, crs_tgt, always_xy=True)
        rp.loginfo("Set geodetic transform from \n"
                   f"{ROS_INFO_PREFIX_LENGTH * ' '} \t {crs_src} \n"
                   f"{ROS_INFO_PREFIX_LENGTH * ' '}to \n"
                   f"{ROS_INFO_PREFIX_LENGTH * ' '} \t {crs_tgt}")
    
    def transform_world2map(self):
        """
        Generate static transform from world to map frame. 
        Uses coordinate reference systems (CRS) and GPS position to determine translation 
        between CRS origin and current location. Map frame originates at starting position.
        Uses compass heading and IMU data to determine rotation between world and map frame.
        """
        
        assert self._mean_gps_position is not None, "Mean GPS position must be available. Call _aggregate_data() first."
        assert self._transformer is not None, "Coordinate reference systems for transformation need to be set." \
                                              "Call set_geodetic_transform() first."
        assert self._map_orientation is not None, "Map orientation needs to be set. " \
                                                  "Call compute_map_orientation() first."
        
        rp.loginfo("Compute GPS position in map frame")
        gps_in_crs1 = self._transformer.transform(*self._mean_gps_position)
        rp.logdebug(f"GPS position in map frame: {gps_in_crs1}")
        
        rp.loginfo("Prepare static transform from world to map frame")
        roll, pitch, yaw = self._map_orientation
        
        world2map_transform = TransformStamped()
        world2map_transform.header.stamp = rp.Time.now()
        world2map_transform.header.frame_id = "world"
        world2map_transform.child_frame_id = "map"
        
        world2map_transform.transform.translation.x = gps_in_crs1[0]
        world2map_transform.transform.translation.y = gps_in_crs1[1]
        world2map_transform.transform.translation.z = gps_in_crs1[2]
        
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
        world2map_transform.transform.rotation.x = quaternion[0]
        world2map_transform.transform.rotation.y = quaternion[1]
        world2map_transform.transform.rotation.z = quaternion[2]
        world2map_transform.transform.rotation.w = quaternion[3]
        
        self._broadcaster.sendTransform(world2map_transform)
        rp.loginfo("Published static transform from world to map frame.")
    
    
    def reset(self):
        
        self._gps_positions = []
        self._mean_gps_position = None
        self._gps_variances = []
        # self._compass_headings = []
        self._magn_fields = []
        # self._mean_compass_heading = None
        self._mean_magn_field = None
        self._imu_data = []
        self._mean_imu_data = None
        self._imu_variances = []
        
        self._transformer = None
        self._map_orientation = None
        
        self._publisher = None
        
        self._reregister()
        
    def _reregister(self):
        
        if self._gps_sub.callback is None:
            self._gps_sub = rp.Subscriber("/mavros/global_position/global", NavSatFix, self._gps_aggregation_callback)
        # if self._compass_sub.callback is None:
        #     self._compass_sub = rp.Subscriber("/mavros/global_position/compass_hdg", Float64, self._compass_callback)
        if self._magn_field_sub.callback is None:
            self._magn_field_sub = rp.Subscriber("/mavros/imu/mag", MagneticField, 
                                                 self._magn_field_aggregation_callback)
        if self._imu_sub.callback is None:
            self._imu_sub = rp.Subscriber("/mavros/imu/data", Imu, self._imu_aggregation_callback)
        
    def _gps_aggregation_callback(self, msg: NavSatFix):
        
        self._gps_positions.append([msg.longitude, msg.latitude, msg.altitude])
        # variances along diagonal (idx 0, 4, 8)
        self._gps_variances.append(np.array(msg.position_covariance)[[0, 4, 8]])
    
    # def _compass_callback(self, msg: Float64):
        
    #     self._compass_headings.append(msg.data)
    
    def _magn_field_aggregation_callback(self, msg: MagneticField):
        
        self._magn_fields.append(unpack_msg(msg.magnetic_field))
    
    def _imu_aggregation_callback(self, msg: Imu):
        
        self._imu_data.append(unpack_msg(msg.linear_acceleration))
        # variances along diagonal (idx 0, 4, 8)
        self._imu_variances.append(np.array(msg.linear_acceleration_covariance)[[0, 4, 8]])
        
    def _gps_conversion_callback(self, msg: NavSatFix):
        
        if self._transformer is None:
            rp.logwarn("No CRS transformer available. Cannot convert GPS position.")
            return
        if self._publisher is None:
            rp.logwarn("No publisher available. Cannot publish GPS position in map frame.")
            return
        
        gps_in_crs1 = self._transformer.transform(msg.longitude, msg.latitude, msg.altitude)
        # rp.logdebug(f"GPS position in map frame: {gps_in_crs1}")
        point = PointStamped()
        point.header.stamp = rp.Time.now()
        point.header.frame_id = "world"
        point.point.x = gps_in_crs1[0]
        point.point.y = gps_in_crs1[1]
        point.point.z = gps_in_crs1[2]
        self._publisher.publish(point)
        
    def _aggregate_data(self):
        
        # Wait until all subscribers have received at least one message
        rp.loginfo("Waiting for GPS and IMU data...")
        rate = rp.Rate(10)
        # all_received = self._gps_positions and self._compass_headings and self._imu_data
        all_received = self._gps_positions and self._magn_fields and self._imu_data
        while not all_received:
            rate.sleep()
            # all_received = self._gps_positions and self._compass_headings and self._imu_data
            all_received = self._gps_positions and self._magn_fields and self._imu_data
        rp.loginfo("GPS and IMU data available. Collecting data over aggregation window...")
        # reset collected data
        self._gps_positions = []
        self._gps_variances = []
        # self._compass_headings = []
        self._magn_fields = []
        self._imu_data = []
        self._imu_variances = []
        # collect data for a while
        rp.sleep(self._aggregation_window)
        # unregister subscribers
        self._gps_sub.unregister()
        # self._compass_sub.unregister()
        self._magn_field_sub.unregister()
        self._imu_sub.unregister()
        rp.loginfo("Data collection finished. Averaging data...")
        # inverse-variance weighted average (unweighted mean for compass because no variance is available) 
        #   of collected data
        self._mean_gps_position = np.average(self._gps_positions, axis=0, weights=1 / np.array(self._gps_variances))
        rp.logdebug(f"Averaged GPS position: {self._mean_gps_position}")
        # self._mean_compass_heading = np.average(self._compass_headings)
        # rp.logdebug(f"Averaged compass heading: {self._mean_compass_heading}")
        self._mean_magn_field = np.average(self._magn_fields, axis=0)
        rp.logdebug(f"Averaged magnetic field: {self._mean_magn_field}")
        self._mean_imu_data = np.average(self._imu_data, axis=0, weights=1 / np.array(self._imu_variances))
        rp.logdebug(f"Averaged IMU data: {self._mean_imu_data}")
        
        ## Unweighted average of collected data
        # self._mean_gps_position = np.mean(self._gps_positions, axis=0)
        # self._mean_compass_heading = np.mean(self._compass_headings)
        # self._mean_imu_data = np.mean(self._imu_data, axis=0)
        
    def compute_map_orientation(self):
        """
        Compute orientation of map frame in world frame (ENU) using compass heading and IMU data.
        """        
        
        # assert self._mean_compass_heading is not None and self._mean_imu_data is not None, \
        #     "Mean compass heading and IMU data must be available. Call _aggregate_data() first."
        assert self._mean_magn_field is not None and self._mean_imu_data is not None, \
               "Mean IMU data (accel and magnetic) must be available. Call _aggregate_data() first."
        
        # rp.loginfo("Computing orientation of map frame in world frame via compass and IMU data...")
        # # compass hdg 0: south -> rotate ENU by 270 deg (yaw)
        # #             90: west -> rotate ENU by 180 deg (yaw)
        # #             180: north -> rotate ENU by 90 deg (yaw)
        # #             270: east -> rotate ENU by 0 deg (yaw)

        # yaw = np.deg2rad((270. - self._mean_compass_heading) % 360)
        
        rp.loginfo("Computing orientation of map frame in world frame via IMU data (acceleration + magnetic)...")
        
        # static_yaw_correction = np.deg2rad(-158.)  # -> for Wesel, experimentally determined
        # static_yaw_correction = np.deg2rad(-55.)  # -> for Blumau, experimentally determined
        static_yaw_correction = 0.
        # Compute yaw from magnetic field data (ENU -> east is zero, magnetic field points north -> correct by 90 deg)
        yaw = np.pi / 2 - np.arctan2(self._mean_magn_field[0], self._mean_magn_field[1]) + static_yaw_correction
        
        # Compute pitch and roll from IMU data (gravity vector)
        roll = np.arctan2(self._mean_imu_data[1], self._mean_imu_data[2])
        pitch = np.arctan2(-self._mean_imu_data[0], np.sqrt(self._mean_imu_data[1] ** 2 + self._mean_imu_data[2] ** 2))
        
        self._map_orientation = (roll, pitch, yaw)
        
        rp.logdebug(f"Map orientation: roll = {np.rad2deg(roll):.2f} deg, " 
                    f"pitch = {np.rad2deg(pitch):.2f} deg, "
                    f"yaw = {np.rad2deg(yaw):.2f} deg")
    
    def publish_gps_conversion(self):
        """
        Publish transformed GPS data in world frame.
        """
        
        self._publisher = rp.Publisher("/mavros/global_position/transformed", PointStamped, queue_size=1)
        rp.Subscriber("/mavros/global_position/global", NavSatFix, self._gps_conversion_callback, queue_size=1)
    
    def __call__(self, continual_transform=False):
        self.compute_map_orientation()
        self.transform_world2map()
        if continual_transform:
            self.publish_gps_conversion()
        rp.spin()

if __name__ == "__main__":
    
    transformer = World2MapTransformer(aggregation_window=5.)
    transformer.set_geodetic_transform("EPSG:4326", "EPSG:25833")
    # alternatively
    # transformer.set_geodetic_transform("WGS84", "UTM_ETRS89")
    transformer(continual_transform=False)