#!/usr/bin/env python
from typing import Any
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from mavros_msgs.msg import PositionTarget,AttitudeTarget
from numpy import cos, sin, tanh, pi
import threading
from controller_tools.tools import Path_3D,R,sawtooth
from controller_tools.MissionDisplayer import MainWindow, plot2D
from pyqtgraph.Qt import QtWidgets
import sys
from scipy.spatial.transform import Rotation
from scipy import signal
from scipy.linalg import expm,logm
from controller_tools.RobotStateMachine import RobotModeState
from controller_tools.ActionServer import ActionServer
from sensor_msgs.msg import Imu
from time import time
from scipy.signal import square as sq

class PID():
    def __init__(self):
        self.data=0
    
    def __call__(self,data,bound):
        if bound<=0:
            raise ValueError('Bound must be positive')
        self.data=self.data+data
        self.data=np.clip(self.data,-bound,bound)

class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        self.imuData=np.zeros(3)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imuCallback)
        
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow(self)
        self.sm=RobotModeState()
        self.pathAction=ActionServer(self)
        self.pathIsComputed=False
        self.init_path()
        # self.p=plot2D()

        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()

        sys.exit(app.exec_())

    def imuCallback(self,msg):
        self.lastImuData=self.imuData
        self.imuData=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
    
    def adj(self,w):
        return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

    def adj_inv(self,A):
        return np.array([A[2,1],A[0,2],A[1,0]])
    
    def main(self):
        speed_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        accel_command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        # attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        f=30
        rate = rospy.Rate(f)
        i=0
        self.s=0
        self.ds=0
        self.I=PID()
        self.displayer.clickMethod()
        s_pos=np.zeros(3)
        self.error=0
        self.last_dVr=np.zeros(3)
        self.t0=time()
        while not rospy.is_shutdown():
            if self.pathIsComputed:
                s_pos=self.path_to_follow.local_info(self.s).X
                self.pathAction.distance_to_goal=np.linalg.norm(s_pos-self.state[:3])+self.path_to_follow.s_max-self.s
            self.displayer.update_state(self.state,s_pos,self.error)
            if self.sm.state=='CONTROL' and self.sm.userInput!='HOME' and self.sm.userInput!='WAIT' and self.pathIsComputed:
                u=self.LPF_control_PID()
                ############################## Acceleration Topic ##############################
                command = PositionTarget()
                command.header.stamp=rospy.Time().now()
                command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
                command.acceleration_or_force=Vector3(*u)
                accel_command_pub.publish(command)
                ############################## Acceleration Topic ##############################        
            elif self.sm.userInput=='HOME':
                command=PoseStamped()
                command.pose.position=Point(0,0,0.5)
                q=Rotation.from_euler('XYZ',[0,0,0],degrees=True).as_quat()
                command.pose.orientation=Quaternion(*q)
                go_home_pub.publish(command)
                self.s=self.ds=0
            elif self.sm.userInput=='WAIT':
                wait_pos=self.path_to_follow.local_info(self.path_to_follow.s_max).X
                command=PoseStamped()
                command.pose.position=Point(*wait_pos)
                q=Rotation.from_euler('XYZ',self.state[6:9],degrees=True).as_quat()
                command.pose.orientation=Quaternion(*q)
                go_home_pub.publish(command)
                self.ds=0
            elif self.sm.state=='HOVERING' or self.sm.state=='STOPPING':
                speed_pub.publish(TwistStamped())
                self.s=self.ds=0

            self.s=self.s+1/f*self.ds
            self.s=max(0,self.s)
            i+=1
            rate.sleep()
    
    def init_path(self,points=[]):
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(2*t),0*t+15]),[0,15],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(0.9*t),10+0*t]),[-10,10],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([2*cos(t),2*sin(t),0*t+10]),[-10,30],type='parametric')
        if len(points)!=0:
            try:
                self.path_to_follow=Path_3D(points,type='waypoints')
                self.displayer.path.setData(pos=self.path_to_follow.points[:,:3])
                self.pathIsComputed=True
            except:
                print('[ERROR] Path properties were not possible to compute [ERROR]')
        ############################### Sphere Path ###############################
        # f=lambda t : R(0.1*t,'x')@(np.array([5*cos(t),5*sin(t),0*t]))+np.array([0,0,15])
        # f=lambda t : np.array([1*cos(t),1*sin(t),0*t])+np.array([0,0,10])
        # f=lambda t : R(t,'y')@np.array([5,0,0])+np.array([0,0,10])+np.array([0*t,sin(15*t),0*t])
        # points=[]
        # for t in np.linspace(-10,20,4000):
        #     points.append(f(t))
        # points=np.array(points).T
        # self.path_to_follow=Path_3D(points,type='waypoints')
        ############################### Sphere Path ###############################
       
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


    def LPF_control_PID(self):
        
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        # ds=self.ds
        wr=self.state[9:12]

        Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        dRm=Rm@self.adj(wr)
        # dt=1/30
        # X=X+3*dt*Rm@Vr
        # s=s+3*self.ds*dt
        # Rm=Rm+3*dRm*dt
        
        # Path properties
        F=self.path_to_follow.local_info(s)
        Rpath=F.R
        
        Rtheta = Rpath.T@Rm
        

        # Error and its derivatives
        e = Rpath.T@(X-F.X)
        s1, y1, w1 = e
        # S=np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        S=np.array([F.k2*w1+F.k1*y1+1,-F.k1*s1,-F.k2*s1]) # PTF
        Vp=Rtheta@Vr
        ks=2
        ds=Vp[0]+ks*s1
        if s<0.05 and ds < -1:
            ds=0
        self.ds=ds
        dRpath=ds*F.dR
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        de = Rtheta@Vr-ds*S
        ds1, dy1,dw1 = de
        # dS=np.array([-F.dC*ds*y1-F.C*dy1, F.dC*ds*s1 +F.C*ds1 -dw1*F.Tr-w1*F.dTr*ds,F.Tr*dy1+F.dTr*ds*y1])

        self.error=100*np.linalg.norm(e,ord=np.inf)
        
        e1=np.array([0,y1,w1])
        de1=np.array([0,dy1,dw1])
        Ke,vc,k0,k1,Kth=2.25,1.5,2,2,3
        # Ke,vc,k0,k1,Kth=self.displayer.values
        
        # Slowing down term when highly curved turn is encountered
        
        # Look ahead curvature
        s=np.linspace(s,s+1.5,50)
        Fahead=self.path_to_follow.local_info(s)
        Cahead=np.max(Fahead.C)
        a=np.sqrt(1.5/(1e-6+Cahead))
        vc=np.clip(vc,0.2,a)

        kpath=0.55
        d_path=np.linalg.norm(e1/kpath)
        ve=vc*(1-np.tanh(d_path))
        dve=-vc/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)

        d_path1=np.linalg.norm(e/kpath)
        t=-Ke*np.clip(Vp[0]**2,-2,2)*np.array([1,0,0])*np.tanh(F.C/5)*6/(1+d_path1)

        dVp=np.array([dve+2*(ve-Vp[0]),0,0])-k1*np.clip(de1,-2,2)-k0*np.clip(e1,-1.5,1.5)+t
        
        # Acceleration commands
        dVr=Rtheta.T@(dVp-dRtheta@Vr)
        dVr=dVr+self.adj(wr)@Vr
        dVr=Kth*np.tanh(dVr/Kth)

        # c1,amax=self.displayer.values[-2:]
        # angle=np.tanh(F.dC/c1)*amax
        # r=Rotation.from_rotvec(F.w1*angle)
        # dVr=r.apply(dVr)

        # self.p.plot(time()-self.t0,F.C,'C','#f5300d')
        # self.p.plot(time()-self.t0,F.dC,'dC','#f5af0d')
        
        # self.p.plot(time()-self.t0,s1,'s1','#49f50d')
        # self.p.plot(time()-self.t0,y1,'y1','#0df5c0')
        # self.p.plot(time()-self.t0,w1,'w1','#0dd5f5')

        return dVr
    
    def update_state(self,data):
        self.state=np.array([*data.data])
if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass