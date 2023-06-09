#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation


class RobotState():
    def __init__(self):
        self.state = np.zeros(12)
        rospy.init_node('robot_state_listener', anonymous=True)
        rospy.Subscriber("/mavros/local_position/velocity_body",
                         TwistStamped, self.body_velocity_info)
        rospy.Subscriber("/mavros/local_position/pose",
                         PoseStamped, self.local_pose_info)
        # rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.local_pose_info)
        self.pub = rospy.Publisher(
            '/robot_state', Float32MultiArray, queue_size=10)
        self.robot_state_pub()

    def quaternion_to_euler(self, q):
        # Extract components of the quaternion
        w, x, y, z = q

        # Compute Euler angles using the ZYX convention
        sinr_cosp = 2.0 * (w*x + y*z)
        cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w*y - z*x)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi/2, sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Return Euler angles in radians
        return np.array([yaw, pitch, roll])

    def body_velocity_info(self, data):
        u, v, w = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        lx, ly, lz = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z
        self.state[3:6] = u, v, w
        self.state[9:12] = lx, ly, lz
        data = Float32MultiArray()
        data.data = self.state
        self.pub.publish(data)

    def local_pose_info(self, data):
        q = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        position = data.pose.position.x, data.pose.position.y, data.pose.position.z
        r = Rotation.from_quat(q)
        self.state[:3] = position
        self.state[6:9] = r.as_euler('XYZ')
        data = Float32MultiArray()
        data.data = self.state
        self.pub.publish(data)

    def robot_state_pub(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    RS = RobotState()
