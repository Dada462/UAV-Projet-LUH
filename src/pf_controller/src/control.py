#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, Vector3, PoseStamped, Point, Quaternion
from mavros_msgs.msg import PositionTarget
from numpy import pi
from controller_tools.tools import Path_3D, sawtooth
from scipy.spatial.transform import Rotation
from controller_tools.RobotStateMachine import RobotModeState
from controller_tools.ActionServer import ActionServer
from controller_tools.OA import oa_alg
from sensor_msgs.msg import Imu
from time import time, perf_counter
from sensor_msgs.msg import PointCloud2
import ros_numpy as rn
# from controller_tools.MissionDisplayer import plot2D
# from pyqtgraph.Qt import QtWidgets
# import threading,sys


class PID():
    def __init__(self):
        self.data = 0

    def __call__(self, data, bound=np.inf):
        if bound <= 0:
            raise ValueError('Bound must be positive')
        self.data = self.data+data
        self.data = np.clip(self.data, -bound, bound)


class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        self.imuData = np.zeros(3)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)
        rospy.Subscriber('/velodyne', PointCloud2, self.velodyneCallback)
        rospy.Subscriber('/md/params', Float32MultiArray, self.param_callback)

        self.sm = RobotModeState()
        self.pathAction = ActionServer(self)
        self.pathIsComputed = False
        # app = QtWidgets.QApplication(sys.argv)
        # self.p=plot2D()
        # ros_thread = threading.Thread(target=self.main, daemon=True)
        # ros_thread.start()
        self.main()
        # sys.exit(app.exec_())

    def param_callback(self, msg):
        self.params = msg.data

    def velodyneCallback(self, msg):
        self.vel = rn.point_cloud2.pointcloud2_to_xyz_array(msg)

    def imuCallback(self, msg):
        self.lastImuData = self.imuData
        self.imuData = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def adj(self, w):
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    def adj_inv(self, A):
        return np.array([A[2, 1], A[0, 2], A[1, 0]])

    def create_publishing_message(self):
        command = PositionTarget()
        command.header.stamp = rospy.Time().now()
        command.coordinate_frame = PositionTarget.FRAME_BODY_NED
        command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
            PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
        command.type_mask = command.type_mask+PositionTarget.IGNORE_YAW
        return command

    def main(self):
        speed_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        accel_command_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        go_home_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        path_info = rospy.Publisher('/path/info', Quaternion, queue_size=10)
        acceleration_command = self.create_publishing_message()
        f = 30
        rate = rospy.Rate(f)
        self.s = 0
        self.sOA = 0
        self.ds = 0
        self.dsOA = 0
        s_pos = np.zeros(3)
        self.error = 0
        self.state = np.zeros(12)
        self.closest_obstacle_distance = np.inf
        last_heading = self.state[8]
        self.t0 = rospy.Time.now().to_time()
        self.e = 0
        while not rospy.is_shutdown():
            Rm = Rotation.from_euler(
                'XYZ', angles=self.state[6:9], degrees=False)
            if self.pathIsComputed:
                s_pos = self.path_to_follow.local_info(self.s).X
                self.pathAction.distance_to_goal = np.linalg.norm(
                    s_pos-self.state[:3])+self.path_to_follow.s_max-self.s
            if self.sm.userInput == 'KEYBOARD':
                pass
            elif self.sm.state == 'ATCP':
                self.oa(self.state, self.sOA, self.vel, self.s)
                # Vr = self.state[3:6]
                # acceleration_command.acceleration_or_force = Vector3(*(-2*Vr))
                # acceleration_command.yaw_rate = 0.5 * \
                #     sawtooth(last_heading-self.state[8])
                # accel_command_pub.publish(acceleration_command)
            elif self.sm.state == 'GOOZ':
                self.oa(self.state, self.sOA, self.vel, self.s)
                target = self.oa.target
                position_to_join = self.joint_point(target=target)
                acceleration_command.acceleration_or_force = Vector3(
                    *position_to_join)
                acceleration_command.yaw_rate = 0.5 * \
                    sawtooth(last_heading-self.state[8])
                accel_command_pub.publish(acceleration_command)
            elif (self.sm.state in ['CONTROL', 'OA']) and self.sm.userInput != 'HOME' and self.sm.userInput != 'WAIT' and self.pathIsComputed:
                OA = self.oa(self.state, self.sOA, self.vel, self.s)
                if OA:
                    u, heading, ds = self.control_pid_1(
                        self.sOA, self.oa.oa_ptf, OA=True)
                    Rm = Rotation.from_euler(
                        'XYZ', angles=self.state[6:9], degrees=False).as_matrix()
                    F = self.path_to_follow.local_info(self.s)
                    Rpath = F.R
                    Rtheta = Rpath.T@Rm
                    Vp = Rtheta@self.state[3:6]
                    ks = 2
                    e = Rpath.T@(self.state[:3]-F.X)
                    s1, _, _ = e
                    self.ds = Vp[0]+ks*s1
                    self.dsOA = ds
                    self.oa.sOA = self.sOA
                    self.sOA = self.sOA+1/f*self.dsOA
                    self.sOA = max(0, self.sOA)
                    s_pos = self.oa.oa_ptf.local_info(self.sOA).X
                else:
                    u, heading, ds = self.control_pid_1(
                        self.s, self.path_to_follow)
                    self.ds = ds
                    self.sOA = 0
                    self.dsOA = 0
                if np.isnan(u).any() or np.isnan(heading).any() or np.isinf(u).any() or np.isinf(heading).any():
                    print(
                        '[WARNING] Commands are NAN or INFINITE, check the path ! Zero will be sent as command as a security measure.')
                    u, heading = np.zeros(3), 0
                ############################## Acceleration Topic ##############################
                acceleration_command.acceleration_or_force = Vector3(*u)
                acceleration_command.yaw_rate = sawtooth(
                    heading-self.state[8])
                last_heading = heading
                accel_command_pub.publish(acceleration_command)
                ############################## Acceleration Topic ##############################
            elif self.sm.userInput == 'HOME' and self.sm.state != 'PILOT':
                position_to_join = self.joint_point(
                    target=np.array([0, 0, 0.5]))
                heading = 0
                acceleration_command.acceleration_or_force = Vector3(
                    *position_to_join)
                acceleration_command.yaw_rate = 0.5 * \
                    sawtooth(heading-self.state[8])
                accel_command_pub.publish(acceleration_command)
                self.s = self.ds = 0
            elif self.sm.userInput == 'WAYPOINT' and self.sm.state != 'PILOT':
                position_to_join = self.joint_point()
                heading = 0
                acceleration_command.acceleration_or_force = Vector3(
                    *position_to_join)
                acceleration_command.yaw_rate = 0.5 * \
                    sawtooth(heading-self.state[8])
                accel_command_pub.publish(acceleration_command)
            elif self.sm.userInput == 'WAIT':
                wait_pos = self.end_of_path
                command = PoseStamped()
                command.pose.position = Point(*wait_pos)
                q = Rotation.from_euler(
                    'XYZ', [0, 0, last_heading], degrees=False).as_quat()
                command.pose.orientation = Quaternion(*q)
                go_home_pub.publish(command)
                self.ds = 0
            elif self.sm.state == 'HOVERING' or self.sm.state == 'STOPPING':
                # speed_pub.publish(TwistStamped())
                Vr = self.state[3:6]
                acceleration_command.acceleration_or_force = Vector3(*(-2*Vr))
                acceleration_command.yaw_rate = 0.5 * \
                    sawtooth(last_heading-self.state[8])
                accel_command_pub.publish(acceleration_command)
                self.s = self.ds = 0
            elif self.sm.state == 'TAKEOFF':
                height = self.state[2]
                if height > 0.15:
                    takeoff_command = self.takeoff_control()
                    if np.isnan(takeoff_command).any() or np.isnan(self.takeoff_heading).any() or np.isinf(takeoff_command).any() or np.isinf(self.takeoff_heading).any():
                        print(
                            '[WARNING] Takeoff commands are NAN or INFINITE ! Zero will be sent as command as a security measure.')
                        takeoff_command, self.takeoff_heading = np.zeros(3), 0
                    ############################## Acceleration Topic ##############################
                    acceleration_command.acceleration_or_force = Vector3(
                        *takeoff_command)
                    acceleration_command.yaw_rate = 0.5 * \
                        sawtooth(self.takeoff_heading-self.state[8])
                    accel_command_pub.publish(acceleration_command)
            path_info.publish(Quaternion(*s_pos, self.error))
            self.s = self.s+1/f*self.ds
            self.s = max(0, self.s)
            rate.sleep()

    def init_path(self, points=[], speeds=[], headings=[]):
        path_computed_successfully = True
        if len(points) != 0:
            self.pathIsComputed = False
            self.path_to_follow = Path_3D(
                points, speeds=speeds, headings=headings, type='waypoints')
            if not self.path_to_follow.compute_path_properties_PTF():
                path_computed_successfully = False
                return path_computed_successfully
            self.oa = oa_alg(ptf=self.path_to_follow, r0=0.75, pfc=self)
            self.pathIsComputed = True
        else:
            path_computed_successfully = False
            return path_computed_successfully
        self.s = 0
        self.sOA = 0
        self.ds = 0
        self.OAds = 0
        return path_computed_successfully

    def control_lpf(self):
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        wr = self.state[9:12]
        dt = 1/30

        Rm = Rotation.from_euler(
            'XYZ', angles=self.state[6:9], degrees=False).as_matrix()
        dRm = Rm@self.adj(wr)

        X = X+3*dt*Rm@Vr
        s = s+3*self.ds*dt
        Rm = Rm+3*dRm*dt
        # Path properties
        F = self.path_to_follow.local_info(s)
        Rpath = F.R

        Rtheta = Rpath.T@Rm
        # Error and its derivatives
        e = Rpath.T@(X-F.X)
        s1, y1, w1 = e
        ks = 1.5
        ds = (Rtheta@Vr)[0]+ks*s1
        if s < 0.05 and ds < 0:
            ds = 0
        self.ds = ds
        dRpath = F.dR*ds
        dRtheta = dRpath.T@Rm+Rpath.T@dRm

        # ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        S = np.array([F.k2*w1+F.k1*y1-1, -F.k1*s1, -F.k2*s1])  # PTF
        de = Vp+ds*S
        ds1, dy1, dw1 = de

        Vpath, k0, k1, kpath, _, c1, amax = 0.5, 1.5, 1.5, 0.4, 1.5, 50, 0.5
        nu_d = F.speed
        heading = F.heading

        e = np.array([s1, y1, w1])

        e1 = np.array([0, y1, w1])
        de1 = np.array([0, dy1, dw1])
        self.error = 100*np.linalg.norm(e, ord=np.inf)
        speed = np.linalg.norm(Vr)
        self.speed = speed

        s = np.linspace(s, s+k0, 250)
        Fahead = self.path_to_follow.local_info(s)
        Cahead = np.max(Fahead.C)
        a = np.sqrt(1/(1e-6+Cahead))
        a = np.clip(a, 0.25, 1.75)
        nu_d = np.clip(nu_d, 0.25, a)

        d_path = np.linalg.norm(e1/kpath)
        ve = nu_d*(1-np.tanh(d_path))
        dve = -nu_d/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        safety_distance = np.clip(1.5*nu_d, 1, 2.5)
        if ((self.path_to_follow.s_max-self.s) < safety_distance):
            ve = np.clip(self.path_to_follow.s_max-self.s, -0.5, 0.5)
            dve = -np.clip(ds, -0.5, 0.5)*(np.abs(ds) <= 0.5)
        Vp = -Vpath*np.tanh(e1/kpath)+np.array([ve, 0, 0])

        Rd = Rotation.from_euler(
            'XYZ', angles=self.state[6:9], degrees=False).as_matrix()
        data = Rd@self.imuData
        Rd1 = Rotation.from_euler(
            'XYZ', [0, 0, self.state[8]], degrees=False).as_matrix()
        data = Rd1.T@data
        data[2] = data[2]-9.81

        dVp = -Vpath/kpath*(1-np.tanh(e1/kpath)**2)*de1+np.array([dve, 0, 0])

        Vd = Rtheta.T@Vp
        dVd = dRtheta.T@Vp+Rtheta.T@dVp
        dVr = dVd+k1*(Vd-Vr)
        dVr = dVr+self.adj(wr)@Vr
        dVr = np.tanh(dVr/3)*3
        self.last_dVr = dVr

        angle = np.tanh(F.dC/c1)*amax
        r = Rotation.from_rotvec(F.w1*angle)
        dVr = r.apply(dVr)
        return dVr, heading

    def control_pid(self):
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        wr = self.state[9:12]

        Rm = Rotation.from_euler(
            'XYZ', angles=self.state[6:9], degrees=False).as_matrix()
        dRm = Rm@self.adj(wr)

        # Path properties
        F = self.path_to_follow.local_info(s)
        Rpath = F.R
        Rtheta = Rpath.T@Rm

        # Error and its derivatives
        e = Rpath.T@(X-F.X)
        s1, y1, w1 = e
        # S=np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        S = np.array([F.k2*w1+F.k1*y1-1, -F.k1*s1, -F.k2*s1])  # PTF
        Vp = Rtheta@Vr
        ks = 2
        ds = Vp[0]+ks*s1
        if s < 0.05 and ds < -1:
            ds = 0
        self.ds = ds
        dRpath = ds*F.dR
        dRtheta = dRpath.T@Rm+Rpath.T@dRm
        de = Vp+ds*S
        ds1, dy1, dw1 = de
        # dS=np.array([-F.dC*ds*y1-F.C*dy1, F.dC*ds*s1 +F.C*ds1 -dw1*F.Tr-w1*F.dTr*ds,F.Tr*dy1+F.dTr*ds*y1])

        self.error = 100*np.linalg.norm(e, ord=np.inf)

        e1 = np.array([0, y1, w1])
        de1 = np.array([0, dy1, dw1])
        Ke, _, k0, k1, Kth = 2.25, 1.5, 2, 2, 3
        vc = F.speed
        heading = F.heading

        # Slowing down term when highly curved turn is encountered

        # Look ahead curvature
        s = np.linspace(s, s+1.5, 50)
        Fahead = self.path_to_follow.local_info(s)
        Cahead = np.max(Fahead.C)
        a = np.sqrt(1.5/(1e-6+Cahead))
        a = np.clip(a, 0.2, 1.75)
        vc = np.clip(vc, 0.2, a)

        kpath = 0.55
        d_path = np.linalg.norm(e1/kpath)
        ve = vc*(1-np.tanh(d_path))
        dve = -vc/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        safety_distance = np.clip(1.5*vc, 1, 2.5)
        if ((self.path_to_follow.s_max-self.s) < safety_distance):
            ve = np.clip(self.path_to_follow.s_max-self.s, -0.5, 0.5)
            dve = -np.clip(ds, -0.5, 0.5)*(np.abs(ds) <= 0.5)
        d_path1 = np.linalg.norm(e/kpath)
        t = -Ke*np.clip(Vp[0]**2, -2, 2)*np.array([1, 0, 0]) * \
            np.tanh(F.C/5)*6/(1+d_path1)

        dVp = np.array([dve+2*(ve-Vp[0]), 0, 0])-k1 * \
            np.clip(de1, -2, 2)-k0*np.clip(e1, -1.5, 1.5)+t

        # Acceleration commands
        dVr = Rtheta.T@(dVp-dRtheta@Vr)
        dVr = dVr+self.adj(wr)@Vr
        dVr = Kth*np.tanh(dVr/Kth)

        return dVr, heading

    def control_pid_1(self, s, ptf, OA=False):
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        wr = self.state[9:12]

        Rm = Rotation.from_euler(
            'XYZ', angles=self.state[6:9], degrees=False).as_matrix()
        dRm = Rm@self.adj(wr)

        # Path properties
        F = ptf.local_info(s)
        Rpath = F.R

        Rtheta = Rpath.T@Rm

        # Error and its derivatives
        e = Rpath.T@(X-F.X)

        s1, y1, w1 = e

        S = np.array([F.k2*w1+F.k1*y1-1, -F.k1*s1, -F.k2*s1])  # PTF
        Vp = Rtheta@Vr
        ks = 2
        ds = Vp[0]+ks*s1
        end_points = ((s < 0.05) and (ds < 0)) or (
            (ptf.s_max-s < 0.03) and (ds > 0))
        ds = (1-end_points)*ds
        # ds=0
        # if s < 0.05 and ds < 0 or ptf.s_max-s < 0.03 and ds > 0:
        #     ds = 0
        dRpath = ds*F.dR
        dRtheta = dRpath.T@Rm+Rpath.T@dRm
        de = Vp+ds*S
        ds1, dy1, dw1 = de
        # print('de',np.round(de,2),'e',np.round(e,2))
        # print("|de|",(dy1**2+dw1**2)**0.5,'de',de,'s',s,'ds',ds,'S',S,Rtheta@Vr)
        # dS=np.array([-F.dC*ds*y1-F.C*dy1, F.dC*ds*s1 +F.C*ds1 -dw1*F.Tr-w1*F.dTr*ds,F.Tr*dy1+F.dTr*ds*y1])
        self.error = 100*np.linalg.norm(e, ord=np.inf)
        e1 = np.array([0, y1, w1])
        de1 = np.array([0, dy1, dw1])
        Ke, _, k0, k1, Kth = 2.25, 1.5, 2, 2, 3
        vc = F.speed
        heading = F.heading
        # Slowing down term when highly curved turn is encountered
        # Look ahead curvature
        s = np.linspace(s, s+1.5, 50)
        Fahead = ptf.local_info(s)
        Cahead = np.max(Fahead.C)
        a = np.sqrt(1.5/(1e-6+Cahead))
        vc = np.clip(vc, 0.2, a)
        # if not OA:
        #     if np.abs(self.oa.obstacle_distance-self.oa.r0)>1:
        #         vc_max=np.inf
        #     else:
        #         vc_max=0.2+0.5*np.abs(self.oa.obstacle_distance-self.oa.r0)
        #     vc = np.clip(vc,0.2,vc_max)

        kpath = 0.55
        d_path = np.linalg.norm(e1/kpath)
        ve = vc*(1-np.tanh(d_path))
        dve = -vc/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        if ((ptf.s_max-self.s) < vc*2) and not OA:
            ve = (ptf.s_max-self.s)-0.75*Vp[0]
        d_path1 = np.linalg.norm(e/kpath)
        t = -Ke*np.clip(Vp[0]**2, -2, 2)*np.array([1, 0, 0]) * \
            np.tanh(F.C/5)*6/(1+d_path1)
        dVp = np.array([dve+2*(ve-Vp[0]), 0, 0])-k1 * \
            de1 - k0*np.clip(e1, -0.5, 0.5)+t*0

        # Acceleration commands
        dVr = Rtheta.T@(dVp-dRtheta@Vr)
        dVr = dVr+self.adj(wr)@Vr
        dVr = Kth*np.tanh(dVr/Kth)

        # t = rospy.Time.now().to_time()-self.t0
        # dt=1/15
        # dem=(e-self.e)/dt
        # self.e=e
        # self.p.plot(t,dem[0],id='dem 1',color='red')
        # self.p.plot(t,dem[1],id='dem 2',color='tomato')
        # self.p.plot(t,dem[2],id='dem 3',color='orange')
        # self.p.plot(t,de[0],id='de 1',color='blue')
        # self.p.plot(t,de[1],id='de 2',color='navy')
        # self.p.plot(t,de[2],id='de 3',color='cornflowerblue')

        return dVr, heading, ds

    def takeoff_control(self):
        Xd = self.takeoff_XYZ+np.array([0, 0, self.sm.takeoff_alt])
        Vd = 0.25  # takeoff speed in (m/s)
        X = self.state[:3]
        R = Rotation.from_euler('XYZ', angles=self.state[6:9], degrees=False)
        Vr = self.state[3:6]
        V = R.apply(Vr)
        k1 = 2
        k0 = Vd*k1
        u = k1*(-V)+k0*np.tanh(1.5*(Xd-X))
        return R.apply(u, inverse=True)

    def land_control(self, land_XY):
        land_target = 0.07
        Xd = np.array([*land_XY, land_target])
        Vd = 0.25
        X = self.state[:3]
        R = Rotation.from_euler('XYZ', angles=self.state[6:9], degrees=False)
        Vr = self.state[3:6]
        V = R.apply(Vr)
        k1 = 2
        k0 = Vd*k1
        u = k1*(-V)+k0*np.tanh(Xd-X)
        return R.apply(u, inverse=True)

    def joint_point(self, target=np.array([0, -5, 0.5])):
        X = self.state[:3]
        R = Rotation.from_euler('XYZ', self.state[6:9], degrees=False)
        V = R.apply(self.state[3:6])
        k1 = 1.75
        k0 = 1
        dV = -k1*V+k0*np.clip((target-X), -2, 2)
        dVr = R.apply(dV, inverse=True)
        return dVr

    def update_state(self, data):
        self.state = np.array([*data.data])


if __name__ == '__main__':
    try:
        C = PFController()
    except rospy.ROSInterruptException:
        pass
