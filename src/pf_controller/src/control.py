#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from mavros_msgs.msg import PositionTarget,AttitudeTarget
from numpy import cos, sin, tanh, pi
import threading
from tools import sawtooth, R, R3, path_info_update,mat_reading
from MissionDisplayer import MainWindow
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
import sys
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

class PID():
    def __init__(self):
        self.I=0

class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        self.state=np.zeros(6)
        self.init_path()
        
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow(self)

        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()

        sys.exit(app.exec_())
   
    def main(self):
        command_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        accel_command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        f=20
        rate = rospy.Rate(f)
        i=0
        self.PID=PID()
        while not rospy.is_shutdown():
            s_pos=path_info_update(self.path_to_follow, self.s).X.reshape((2,1))
            self.displayer.update_state(self.state,s_pos)
            # u=self.LPF_control_kin(self.state)
            u=self.LPF_control_dyn()
            # u=4*np.tanh(u)
            if self.displayer.mission_state['start']:
                # Kin Lyapunov PF
                # command=TwistStamped()
                # command.twist.linear=Vector3(u[0],u[1],0.)
                # command.twist.angular=Vector3(0.,0.,u[-1])
                # command_pub.publish(command)
                

                # Dyn Lyapunov PF
                # command = PositionTarget()
                # command.header.stamp=rospy.Time().now()
                # command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                # command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
                # command.acceleration_or_force=Vector3(u[0],u[1],0.)
                # command.yaw=u[2]
                # accel_command_pub.publish(command)
                
                msg = AttitudeTarget()
                msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                msg.thrust=0.5
                msg.body_rate=Vector3(*u)
                attitude_pub.publish(msg)


                # # Speed control
                # u=self.speed_control()
                # msg = AttitudeTarget()
                # msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                # msg.thrust=0.5
                # msg.body_rate=Vector3(*u)
                # attitude_pub.publish(msg)

                # # Using point joining of the FC
                # command=PoseStamped()
                # command.pose.position=Point(*s_pos,10)
                # go_home_pub.publish(command)
            elif self.displayer.mission_state['keyboard']:
                key=np.array([self.displayer.keyboard]).reshape(-1,2)
                D=np.array([[0,-1,0],[1,0,0],[0,0,-1]])
                u=0.09*D@(key[:,0]-key[:,1])
                # thrust+=0.05/f*u[0]
                msg = AttitudeTarget()
                msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                # msg.thrust=0.5+0.1*u[0]/0.5
                msg.thrust=0.5
                w=np.flip(self.full_state[6:9])
                w[2]=0
                msg.body_rate=Vector3(*2*(u-w))
                attitude_pub.publish(msg)
            else:
                command=PoseStamped()
                command.pose.position=Point(0,0,10)
                go_home_pub.publish(command)
            self.s=self.s+1/f*self.ds
            self.s=max(0,self.s)
            i+=1
            rate.sleep()
    
    def init_path(self):
        # self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(2*t)]),[0,15]) # The path the robot has to follow
        # self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(0.9*t)])) # The path the robot has to follow
        # self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(t)])) # The path the robot has to follow
        self.path_to_follow=mat_reading(lambda t : 10*(2+sin(10*t))*np.array([cos(t),sin(t)])) # The path the robot has to follow
        # self.path_to_follow=mat_reading(lambda t : np.array([t,-10+t*0]),[-20,20]) # The path the robot has to follow
        self.s=0
        self.vars=1.25,1.5,1.8,0.09,3,1
    
    def update_state(self,data):
        speed=data.data[3:6]
        r = Rotation.from_euler('ZYX', data.data[6:9],degrees=False)
        r2=Rotation.from_euler('ZYX',[-data.data[6],0,0],degrees=False)
        speed=r.apply(speed)
        speed=r2.apply(speed)
        self.state=np.array([*data.data])[[0,1,3,4,6,11]]
        self.state[2:4]=speed[0:2]
        self.full_state=np.array([*data.data])
        self.full_state[3:6]=speed
    
    def speed_control(self):
        V=self.full_state[3:6]
        w=np.flip(self.full_state[6:9])
        Vd=np.array([0,1,0])
        k=0.2
        wd=k*np.tanh(Vd-V)
        # print(wd)
        D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
        wd=D@wd
        return 2*(wd-w)
    
    def LPF_control_kin(self,x):
        X = x[0:2]
        theta_m = x[4]
        s = self.s
        F = path_info_update(self.path_to_follow, s)
        theta_c = F.psi
        theta = sawtooth(theta_m-theta_c)
        s1, y1 = R(theta_c).T@(X-F.X)
        # psi_a,Kdy1=self.vars
        Kdy1=0.5
        psi_a=pi/2
        delta = -psi_a*tanh(Kdy1*y1)
        beta=sawtooth(delta-theta)
        
        psi = delta
        nu=0.5
        ks = 1
        ds = cos(psi)*nu + ks*s1
        # theta_m_d=-pi/4
        # dtheta_m_d=0
        # k2=1
        # dtheta_m=dtheta_m_d+k2*sawtooth(theta_m_d-theta_m)
        u,v=nu*cos(beta),nu*sin(beta)
        self.ds=ds
        return np.array([u,v,0])
    
    def LPF_control_dyn(self):
        X = self.full_state[0:2]
        Vt = self.full_state[3:5]
        s = self.s
        theta_m, dtheta_m = self.full_state[6],self.full_state[9]
        u, v = Vt
        nu = np.linalg.norm(Vt)
        beta=np.arctan2(v,u)

        print('Speed:',np.round(nu,2))
        
        F = path_info_update(self.path_to_follow, s)
        theta_c = F.psi
        s1, y1 = R(theta_c).T@(X-F.X)
        theta = sawtooth(theta_m-theta_c)
        psi = sawtooth(theta+beta)

        ks = 0.7
        ds = cos(psi)*nu + ks*s1
        self.ds=ds
        dtheta_c = F.C_c*ds
        ds1, dy1 = R(theta)@Vt-ds*np.array([1-F.C_c*y1, F.C_c*s1])

        psi_a=1.3
        y10,k1,k2,k3,k4,k5=self.vars
        delta = -psi_a*tanh(y1/y10)
        ddelta = -psi_a*(1-tanh(y1/y10)**2)*dy1/y10
        dpsi=ddelta+k2*sawtooth(delta-psi)
        dbeta=dpsi-dtheta_m+dtheta_c

        nu_d=1
        dnu=k1*(nu_d-nu)
        # theta_m_d=pi/4
        # dtheta_m_d=0
        # ddtheta_m_d=0
        # ddtheta_m=ddtheta_m_d+2*(dtheta_m_d-dtheta_m)+1*(theta_m_d-theta_m)
        # dbeta=-0.25*beta
        # dnu=(2-nu)
        C=R3(beta)@np.array([dnu,nu*dbeta,0])
        w=np.flip(self.full_state[6:9])
        # k=0.07
        wd=k3*np.tanh(C/k5)
        D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
        wd=D@wd
        u=k4*(wd-w)
        from time import time
        self.displayer.data_to_plotx.append(time())
        # self.displayer.data_to_ploty1.append(s1)
        self.displayer.data_to_ploty2.append(y1)
        return u


if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass