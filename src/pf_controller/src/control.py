#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from sensor_msgs.msg import Imu
from mavros_msgs.msg import PositionTarget,AttitudeTarget
from numpy import cos, sin, tanh, pi
import threading
from tools_3D import Path_3D,R,sawtooth
from MissionDisplayer3D import MainWindow, plot2D
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
import sys
from scipy.spatial.transform import Rotation
from scipy import signal
import matplotlib.pyplot as plt
from time import time
from scipy.linalg import expm,logm

class PID():
    def __init__(self):
        self.data=0

class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        self.init_path()
        
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow(self)

        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()

        sys.exit(app.exec_())
    
    def adj(self,w):
        return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

    def adj_inv(self,A):
        return np.array([A[2,1],A[0,2],A[1,0]])
    
    
    def main(self):
        accel_command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        f=30
        rate = rospy.Rate(f)
        i=0
        self.I=PID()
        self.displayer.clickMethod()
        while not rospy.is_shutdown():
            s_pos=self.path_to_follow.local_info(self.s).X
            self.displayer.update_state(self.state,s_pos)
            u=self.LPF_control_3D_v3()
            if np.linalg.norm(self.state[:2],ord=np.inf)>100:
                self.displayer.mission_state['start']=False
                self.displayer.mission_state['keyboard']=False
            if self.displayer.mission_state['start']:            
                ############################## Acceleration Topic ##############################
                command = PositionTarget()
                command.header.stamp=rospy.Time().now()
                command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
                command.acceleration_or_force=Vector3(*u)
                try:
                    accel_command_pub.publish(command)
                except:
                    command.acceleration_or_force=Vector3(0,0,0)
                    accel_command_pub.publish(command)

                ############################## Acceleration Topic ##############################
            elif self.displayer.mission_state['keyboard']:
                key=np.array([self.displayer.keyboard]).reshape(-1,2)
                D=np.array([[0,-1,0],[1,0,0],[0,0,-1]])
                u=0.1*D@(key[:,0]-key[:,1])

                msg = AttitudeTarget()
                msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                msg.thrust=0.5
                w=self.state[6:9]
                msg.body_rate=Vector3(*2*(u-w)[:2],0)
                attitude_pub.publish(msg)

            else:
                command=PoseStamped()
                command.pose.position=Point(0,0,10)
                q=Rotation.from_euler('XYZ',[0,0,90],degrees=True).as_quat()
                command.pose.orientation=Quaternion(*q)
                go_home_pub.publish(command)
            self.s=self.s+1/f*self.ds
            self.s=max(0,self.s)
            i+=1
            rate.sleep()
    
    def init_path(self):
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(2*t),0*t+15]),[0,15],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(0.9*t),10+0*t]),[-10,10],type='parametric')
        
        ############################### Sphere Path ###############################
        f=lambda t : R(0.1*t,'x')@(np.array([5*cos(t),5*sin(t),0*t]))+np.array([0,0,15])
        # f=lambda t : np.array([1*cos(t),1*sin(t),0*t])+np.array([0,0,10])
        # f=lambda t : R(t,'y')@np.array([5,0,0])+np.array([0,0,10])+np.array([0*t,sin(15*t),0*t])
        points=[]
        for t in np.linspace(-10,20,4000):
            points.append(f(t))
        points=np.array(points).T
        self.path_to_follow=Path_3D(points,type='waypoints')
        ############################### Sphere Path ###############################
        # self.path_to_follow=Path_3D(lambda t : np.array([2*cos(t),2*sin(t),0*t+10]),[-10,30],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([t+7,3*cos(2*pi*t/2)+5,0*t+10]),[-10,30],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([t+7,3*cos(2*pi*t/7)+5,2*cos(2*pi*t/3)+10]),[-10,30],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([5*(2+sin(10*t))*cos(t),5*(2+sin(10*t))*sin(t),0*t+10]),[-10,10],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([t,-10+0.01*t**2,0.01*t+10]),[-20,20],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(t),3*(t+10)+10]),[-10,10],type='parametric')
        
        ################################ Real Robot ################################
        # self.path_to_follow=Path_3D(lambda t : np.array([cos(t),sin(t),0.7+0*t]),[0,10],type='parametric')
        ################################ Real Robot ################################
        self.s=0
        self.ds=0

    def LPF_control_3D_v3(self):
        Ke,k0,k1,Ks,Kth,nu_d,_,vc=self.displayer.values
        
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        wr=self.state[9:12]

        Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        dRm=Rm@self.adj(wr)
        
        # Path properties
        F=self.path_to_follow.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        
        pos=F.X
        s1=np.vstack((pos,pos+2*F.s1))
        y1=np.vstack((pos,pos+2*F.y1))
        w1=np.vstack((pos,pos+2*F.w1))
        self.displayer.s1_arrow.setData(pos=s1)
        self.displayer.y1_arrow.setData(pos=y1)
        self.displayer.w1_arrow.setData(pos=w1)
       
        Rtheta = Rpath.T@Rm
        

        # Error and its derivatives
        e = Rpath.T@(X-F.X)
        s1, y1, w1 = e
        S=np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        Vp=Rtheta@Vr
        ks=1
        ds=Vp[0]+ks*s1
        if s<0.05 and ds < -1:
            ds=0
        self.ds=ds
        dRpath=ds*F.dR
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        de = Rtheta@Vr-ds*S
        ds1, dy1,dw1 = de

        self.error=100*np.linalg.norm(e,ord=np.inf)
        
        e1=np.array([0,y1,w1])
        de1=np.array([0,dy1,dw1])
        # Ke,vc,k1,k0,Kth=2.4,_,1.7,2.5,4
        t=-Ke*vc*Vp[0]**2*np.array([1,0,0])*np.tanh(F.C/5)*6
        dVp=2*np.array([(vc-Vp[0]),0,0])-k1*de1-k0*e1+t

        # Acceleration commands
        dVr=Rtheta.T@(dVp-dRtheta@Vr)
        dVr=dVr+self.adj(wr)@Vr
        dVr=Kth*np.tanh(dVr/Kth)
        self.v1=np.linalg.norm(Vp)
        u=Rm@dVr
        u=np.clip(u,-2,2)
        pos=X
        dir=u
        arrow=np.vstack((pos,pos+2*dir))
        self.displayer.control_output.setData(pos=arrow)
        return dVr

    
    def update_state(self,data):
        self.state=np.array([*data.data])



if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass