#!/usr/bin/env python3
"""
Package incoporating the PID path-following controller.
"""

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
from sensor_msgs.msg import Imu


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

        self.sm = RobotModeState()
        self.pathAction = ActionServer(self)
        self.pathIsComputed = False
        self.init_path()
        self.main()

    def imuCallback(self, msg):
        self.lastImuData = self.imuData
        self.imuData = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def adj(self, w):
        return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    def adj_inv(self, A):
        return np.array([A[2, 1], A[0, 2], A[1, 0]])

    def main(self):
        speed_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        accel_command_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        go_home_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        path_info = rospy.Publisher('/path/info', Quaternion, queue_size=10)
        f = 30
        rate = rospy.Rate(f)
        i = 0
        self.s = 0
        self.ds = 0
        s_pos = self.end_of_path = np.zeros(3)
        self.error = 0.
        last_heading = 0
        self.closest_obstacle_distance=np.inf
        while not rospy.is_shutdown():
            if self.pathIsComputed:
                s_pos = self.path_to_follow.local_info(self.s).X
                self.pathAction.distance_to_goal = np.linalg.norm(
                    s_pos-self.state[:3])+self.path_to_follow.s_max-self.s
            if self.sm.state == 'CONTROL' and self.sm.userInput != 'HOME' and self.sm.userInput != 'WAIT' and self.pathIsComputed:
                u, heading = self.control_pid()
                if np.isnan(u).any() or np.isnan(heading).any() or np.isinf(u).any() or np.isinf(heading).any():
                    self.ActionServer.pathError=True
                    print(
                        '[WARNING] Commands are NAN or INFINITE, check the path ! Zero will be sent as command as a security measure.')
                    u, heading = np.zeros(3), 0
                ############################## Acceleration Topic ##############################
                command = PositionTarget()
                command.header.stamp = rospy.Time().now()
                command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                    PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
                command.type_mask = command.type_mask+PositionTarget.IGNORE_YAW
                command.acceleration_or_force = Vector3(*u)
                command.yaw_rate = 0.5*sawtooth(heading-self.state[8])
                last_heading = heading
                accel_command_pub.publish(command)
                ############################## Acceleration Topic ##############################
            elif self.sm.userInput == 'HOME':
                command = PoseStamped()
                command.pose.position = Point(0, 0, 0.5)
                q = Rotation.from_euler(
                    'XYZ', [0, 0, 0], degrees=True).as_quat()
                command.pose.orientation = Quaternion(*q)
                go_home_pub.publish(command)
                self.s = self.ds = 0
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
                speed_pub.publish(TwistStamped())
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
                    command = PositionTarget()
                    command.header.stamp = rospy.Time().now()
                    command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                    command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                        PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
                    command.type_mask = command.type_mask+PositionTarget.IGNORE_YAW
                    command.acceleration_or_force = Vector3(*takeoff_command)
                    command.yaw_rate = 0.5 * \
                        sawtooth(self.takeoff_heading-self.state[8])
                    accel_command_pub.publish(command)
            path_info.publish(Quaternion(*s_pos, self.error))
            self.s = self.s+1/f*self.ds
            self.s = max(0, self.s)
            i += 1
            rate.sleep()

    def init_path(self, points=[], speeds=[], headings=[]):
        """
        Create the path to follow
        """
        if len(points) != 0:
            print('Speed received:',np.mean(speeds),np.min(speeds),np.max(speeds),' m/s')
            speeds=np.clip(speeds,0.01,2)
            self.pathIsComputed = False
            self.path_to_follow = Path_3D(
                points, speeds=speeds, headings=headings, type='waypoints')
            if not self.path_to_follow.compute_path_properties_PTF():
                path_computed_successfully = False
                return path_computed_successfully
        else:
            path_computed_successfully = False
            return path_computed_successfully
        self.s = 0
        self.ds = 0
        self.pathIsComputed = True
        path_computed_successfully = True
        return path_computed_successfully

    def control_pid(self):
        """
        Path-following algorithm.
        The parameters are explained & can be changed further down.
        """
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        wr = self.state[9:12]

        Rm = Rotation.from_euler(
            'XYZ', angles=self.state[6:9], degrees=False).as_dcm()
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
        end_points= ((s < 0.05) and (ds < 0)) or ((self.path_to_follow.s_max-s < 0.03) and (ds > 0))
        ds=(1-end_points)*ds
        self.ds = ds
        dRpath = ds*F.dR
        dRtheta = dRpath.T@Rm+Rpath.T@dRm
        de = Vp+ds*S
        ds1, dy1, dw1 = de

        self.error = 100*np.linalg.norm(e, ord=np.inf)

        e1 = np.array([0, y1, w1])
        de1 = np.array([0, dy1, dw1])
        """
        Paramaters:
            -Ke: how much the robot should slow down in highly curved turns. Should not be changed normally.
            -k0: proportional gain regarding the commands in the PD controller.
            -k1: derivative gain regarding the command in the PD controller.
            -Kth: saturation of the acceleration commands (currently maxed at 3 m/s²)
            -kpath: critical distance to the path/agressivity of the controller with regards to the path. 
                    A small value will result in a more agressive control to stay on the path.
                    Bigger values will result in a smoother converge to the path and but slower.
        """
        Ke, k0, k1, Kth = 2.25, 1, 1.55, 3
        kpath = .85
        
        vc = F.speed
        heading = F.heading

        # Look ahead curvature
        s = np.linspace(s, s+1.5, 50)
        Fahead = self.path_to_follow.local_info(s)
        Cahead = np.max(Fahead.C)
        a = np.sqrt(1.5/(1e-6+Cahead))
        a = np.clip(a, 0.2, 1.75)
        vc = np.clip(vc, 0.2, a)

        d_path = np.linalg.norm(e1/kpath)
        ve = vc*(1-np.tanh(d_path))
        dve = -vc/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        safety_distance = np.clip(1.5*vc, 1, 2.5)
        """
        This is what slows down the drone towards the end of the path.
        The bigger the speeds the sooner on the path the drone should start breaking to not overshoot the end point.
        This deals with the fact that the UAV takes time to change orientation, 
        thus creating a delay on the thrust.
        example: if the robot is tilted left and needs to go right fast, it
        first needs to turn right such that the thrust is aligned with the direction.
        This change in orientation is a delay that can be cause the drone to overshoot in
        higly curved turns.
        """

        if ((self.path_to_follow.s_max-self.s) < safety_distance):
            ve = np.clip(self.path_to_follow.s_max-self.s, -0.5, 0.5)
            dve = -np.clip(ds, -0.5, 0.5)*(np.abs(ds) <= 0.5)

        d_path1 = np.linalg.norm(e/kpath)
        # Slowing down term when highly curved turn is encountered
        t = -Ke*np.clip(Vp[0]**2, -2, 2)*np.array([1, 0, 0]) * \
            np.tanh(F.C/5)*6/(1+d_path1)

        dVp = np.array([dve+2*(ve-Vp[0]), 0, 0])-k1*de1-k0*np.clip(e1, -1.5, 1.5)+t

        # Acceleration commands
        dVr = Rtheta.T@(dVp-dRtheta@Vr)
        dVr = dVr+self.adj(wr)@Vr
        dVr = Kth*np.tanh(dVr/Kth)

        return dVr, heading

    def takeoff_control(self):
        Xd = self.takeoff_XYZ+np.array([0, 0, self.sm.takeoff_alt])
        Vd = 0.25 # speeds with which the UAV takesoff (m/s)
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

    def update_state(self, data):
        self.state = np.array([*data.data])


if __name__ == '__main__':
    try:
        C = PFController()
    except rospy.ROSInterruptException:
        pass
