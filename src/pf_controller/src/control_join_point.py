#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped
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
        self.state=np.zeros(6)
        self.main()

    def main(self):
        command_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        f=20
        rate = rospy.Rate(f)
        i=0
        while not rospy.is_shutdown():
            u=self.controller(self.state)
            command=TwistStamped()
            command.twist.linear=Vector3(u[0],u[1],0.)
            command.twist.angular=Vector3(0.,0.,u[-1])
            command_pub.publish(command)
            i+=1
            rate.sleep()

    def update_state(self,data):
        self.state=np.array([*data.data])[[0,1,3,4,6,11]]
    
    # def controller(self,state):
    #     x,y,u,v,theta,dtheta=state
    #     print(R(theta)@np.array([u,v]))
    #     X=np.array([x,y])
    #     X_d=np.array([10,10])
    #     V=2*(X_d-X)
    #     return np.array([-1,0,1])

    def controller(self,x):
        #For the case when at least two motors are not colinear
        X = x[0:2]
        Vt = x[2:4]
        theta_m, dtheta_m = x[4],x[5]
        u, v = Vt

        Pd=np.array([0,-10])
        P=X
        dP = R(theta_m)@Vt
        alpha1 = 2
        alpha0 = 1
        dPd = 0
        ddPd = 0
        ddP = ddPd+alpha1*(dPd-dP)+alpha0*(Pd-P)
        u = (ddP)
        return u[0], u[1],0


if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass