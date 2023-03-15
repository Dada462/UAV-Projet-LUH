#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point
from numpy import cos, sin, tanh, pi
import threading
from tools import sawtooth, R, path_info_update,mat_reading
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
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        f=20
        rate = rospy.Rate(f)
        i=0
        while not rospy.is_shutdown():
            s_pos=path_info_update(self.path_to_follow, self.s).X.reshape((2,1))
            self.displayer.update_state(self.state,s_pos)
            u=self.controller(self.state)
            if self.displayer.mission_state['start']:
                command=TwistStamped()
                # x,y=2*np.cos(i/10),2*np.sin(2*i/10)
                # self.state[:]=x,y,i*2
                # print((self.state[:2].reshape((2,1))-s_pos))
                command.twist.linear=Vector3(u[0],u[1],0.)
                command.twist.angular=Vector3(0.,0.,u[-1])
                command_pub.publish(command)
            else:
                command=PoseStamped()
                command.pose.position=Point(0.,0.,10)
                go_home_pub.publish(command)
            self.s=self.s+2/f*self.ds
            self.s=max(0,self.s)
            i+=1
            rate.sleep()
    
    def init_path(self):
        # self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(2*t)]),[0,15]) # The path the robot has to follow
        self.path_to_follow=mat_reading(lambda t : 5*np.array([cos(t),sin(0.9*t)])) # The path the robot has to follow
        self.s=0
        self.vars=1,10
    
    def update_state(self,data):
        self.state=np.array([*data.data])[[0,1,6]]
    
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
    
    def calculate_metrics(self):
        pass
if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass