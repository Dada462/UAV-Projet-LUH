#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped
from numpy import cos, sin, tanh, pi
import threading
from tools import sawtooth, R, R3, path_info_update,mat_reading
from MissionDisplayer import MainWindow
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
import sys

class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        self.state=np.zeros(3)
        self.init_path()
        
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow(self)

        ros_thread = threading.Thread(target=self.main)
        ros_thread.start()

        sys.exit(app.exec_())

    def main(self):
        command_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        accel_command_pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        f=20
        rate = rospy.Rate(f)
        i=0
        while not rospy.is_shutdown():
            s_pos=path_info_update(self.path_to_follow, self.s).X.reshape((2,1))
            self.displayer.update_state(self.state,s_pos)
            # u=self.controller(self.state)
            u=self.controller_Non_Col(self.state)
            if self.displayer.mission_state['start']:
                # Kin Lyapunov PF
                command=TwistStamped()
                command.twist.linear=Vector3(u[0],u[1],0.)
                command.twist.angular=Vector3(0.,0.,u[-1])
                command_pub.publish(command)
                
                # Dyn Lyapunov PF
                
                command=Vector3Stamped()
                command.vector=Vector3(u[0],u[1],0.)
                accel_command_pub.publish(command)

                # Using point joining of the FC
                # command=PoseStamped()
                # command.pose.position=Point(*s_pos,10)
                # go_home_pub.publish(command)
            else:
                command=PoseStamped()
                command.pose.position=Point(0.,0.,10)
                go_home_pub.publish(command)
            self.s=self.s+1/f*self.ds
            self.s=max(0,self.s)
            i+=1
            rate.sleep()
    
    def init_path(self):
        # self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(2*t)]),[0,15]) # The path the robot has to follow
        self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(0.9*t)])) # The path the robot has to follow
        # self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(t)])) # The path the robot has to follow
        # self.path_to_follow=mat_reading(lambda t : 2.5*(2+sin(10*t))*np.array([cos(t),sin(t)])) # The path the robot has to follow
        self.s=0
        self.vars=1,10
    
    def update_state(self,data):
        self.state=np.array([*data.data])[[0,1,6]]
        speed=np.array([*data.data])[[3,4]]
        print('Speed: ',np.linalg.norm(speed))
    
    def controller(self,x):
        #For the case when at least two motors are not colinear
        X = x[0:2]
        theta_m = x[2]
        s = self.s
        F = path_info_update(self.path_to_follow, s)
        theta_c = F.psi
        theta = sawtooth(theta_m-theta_c)
        s1, y1 = R(theta_c).T@(X-F.X)
        psi_a,Kdy1=self.vars
        # Kdy1=12
        # psi_a=1.2
        delta = -psi_a*tanh(Kdy1*y1)
        beta=sawtooth(delta-theta)
        psi = sawtooth(theta+beta)
        nu=0.5
        ks = 0.5
        ds = cos(psi)*nu + ks*s1
        theta_m_d=-pi/4
        dtheta_m_d=0
        k2=1
        dtheta_m=dtheta_m_d+k2*sawtooth(theta_m_d-theta_m)
        u,v=nu*cos(beta),nu*sin(beta)
        self.ds=ds
        return np.array([u,v,dtheta_m])
    
    def controller_Non_Col(self,x):
        #For the case when at least two motors are not colinear
        X = x[0:2]
        Vt = x[3:5]
        s = self.s
        theta_m, dtheta_m = x[6],x[11]
        u, v = Vt

        Pd=self.vars
        P=X
        dP = R(theta_m)@Vt
        alpha1 = 2
        alpha0 = 1
        dPd = 0
        ddPd = 0
        ddP = ddPd+alpha1*(dPd-dP)+alpha0*(Pd-P)
        u = (ddP-dtheta_m*R(pi/2)@dP)
        return u[0], u[1],0

        nu = (u**2+v**2)**0.5
        beta=np.arctan2(v,u)
        
        F = path_info_update(self.path_to_follow, s)
        theta_c = F.psi
        s1, y1 = R(theta_c).T@(X-F.X)
        theta = sawtooth(theta_m-theta_c)
        psi = sawtooth(theta+beta)
        ks = 1
        ds = cos(psi)*nu + ks*s1
        dtheta_c = F.C_c*ds
        ds1, dy1 = R(theta)@Vt-ds*np.array([1-F.C_c*y1, F.C_c*s1])

        psi_a=pi/2
        Kdy1=1
        delta = -psi_a*tanh(Kdy1*y1)
        ddelta = -psi_a*Kdy1*(1-tanh(Kdy1*y1)**2)*dy1

        dnu=2*(1-nu)
        dpsi=ddelta+2*sawtooth(delta-psi)
        dbeta=dpsi-dtheta_m+dtheta_c
        theta_m_d=pi/4
        dtheta_m_d=0
        ddtheta_m_d=0
        ddtheta_m=ddtheta_m_d+2*(dtheta_m_d-dtheta_m)+1*(theta_m_d-theta_m)
        C=R3(beta+theta_m)@np.array([dnu,nu*dbeta,ddtheta_m])
        return C


if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass