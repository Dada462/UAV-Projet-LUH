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
from RobotStateMachine import RobotModeState

class PID():
    def __init__(self):
        self.data=0

class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        # rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        self.init_path()
        
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow(self)
        self.sm=RobotModeState()
        # self.p=plot2D()

        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()

        sys.exit(app.exec_())
    
    def adj(self,w):
        return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

    def adj_inv(self,A):
        return np.array([A[2,1],A[0,2],A[1,0]])
    
    def imu_callback(self,data):
        self.imu_data=data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z
        a_mes=np.array([*self.imu_data])
        R=Rotation.from_euler('XYZ',self.state[6:9],degrees=False).as_matrix()
        self.a_mes=R@a_mes
        # print('Imu:',*np.round(R@a_mes,2))
    
    def main(self):
        speed_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # accel_command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # data_view=rospy.Publisher('/dataView', Vector3, queue_size=10)

        f=30
        rate = rospy.Rate(f)
        i=0
        self.I=PID()
        self.displayer.clickMethod()
        while not rospy.is_shutdown():
            s_pos=self.path_to_follow.local_info(self.s).X
            self.displayer.update_state(self.state,s_pos)
            u=self.LPF_control_3D_v3()
            # self.ds=0
            # u=self.LPF_control_3D()
            # u=self.LPF_control_kin()
            if np.linalg.norm(self.state[:2],ord=np.inf)>100:
                self.displayer.mission_state['start']=False
                self.displayer.mission_state['keyboard']=False
            if self.displayer.mission_state['start']:
                ############################## Speed Topic ##############################
                # command=TwistStamped()
                # command.twist.linear=Vector3(*u)
                # command.twist.angular=Vector3(0.,0.,0)
                # speed_pub.publish(command)
                ############################## Speed Topic ##############################
    
                

                ############################## Attitude Topic ##############################
                # msg = AttitudeTarget()
                # msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                # k3,k4=0.01,2
                # w=self.state[6:9]
                # wd=k3*u
                # D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
                # wd=D@wd
                # u=k4*(wd-w)
                # msg.body_rate=Vector3(*u[:2],0)
                # # try:
                # #     k,k1=self.vars[0]
                # # except:
                # #     k,k1=0.2,1
                # # ddz=k*np.tanh(k1*u[2])
                # msg.thrust=0.5
                # attitude_pub.publish(msg)
                ############################## Attitude Topic ##############################
                
                

                ############################## Waypoint Topic ##############################
                # # Join a point
                # T=20
                # t=i/f
                # # P=signal.square(2 * np.pi * t/(2*T))*np.array([-1,1])*10
                # P=R(2 * np.pi * t/T)[:,0]*10
                # command=PoseStamped()
                # command.pose.position=Point(*P,10)
                # go_home_pub.publish(command)
               
                # # Using point joining of the FC
                # command=PoseStamped()
                # command.pose.position=Point(*s_pos,10)
                # go_home_pub.publish(command)
                ############################## Waypoint Topic ##############################
                
                self.sm.robotStateCallback(self.state[2])
                self.sm(u)
                
                
                ############################## Acceleration Topic ##############################
                # command = PositionTarget()
                # command.header.stamp=rospy.Time().now()
                # command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                # command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
                # command.acceleration_or_force=Vector3(*u)
                # try:
                #     accel_command_pub.publish(command)
                # except:
                #     command.acceleration_or_force=Vector3(0,0,0)
                #     accel_command_pub.publish(command)

                ############################## Acceleration Topic ##############################
            elif self.displayer.mission_state['keyboard']:
                key=np.array([self.displayer.keyboard]).reshape(-1,2)
                D=np.array([[0,-1,0],[1,0,0],[0,0,-1]])
                u=0.1*D@(key[:,0]-key[:,1])

                msg = AttitudeTarget()
                # msg.type_mask=AttitudeTarget.IGNORE_PITCH_RATE+AttitudeTarget.IGNORE_YAW_RATE+AttitudeTarget.IGNORE_ROLL_RATE
                msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                msg.thrust=0.5
                # q=Rotation.from_euler('XYZ',angles=u,degrees=True).as_quat()
                # msg.orientation=Quaternion(*q)
                w=self.state[6:9]
                # w[2]=0
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
        # ds=self.ds
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
        # dRpath=ds*np.vstack((F.ds1,F.dy1,F.dw1)).T
       
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
        # dS=np.array([-F.dC*ds*y1-F.C*dy1, F.dC*ds*s1 +F.C*ds1 -dw1*F.Tr-w1*F.dTr*ds,F.Tr*dy1+F.dTr*ds*y1])

        self.error=100*np.linalg.norm(e,ord=np.inf)
        


        # ############# 1 #############
        # # Moving point
        # dds=-ds+ks*s1+0.4*ds1
        # # dds=0
        # # if s<0.05 and ds < -1:
        # #     dds=0
        # self.dds=dds
        # # Acceleration commands
        # Vp=Rtheta@Vr
        # dVp=-Vp+ds*dS-k1*np.tanh(Ks*e)+2*np.array([1.5-Vp[0],0,0])
        # # dVr=Rtheta.T@(dVp-dRtheta@Vr)
        # dVr=dRtheta.T@Vp+Rtheta.T@dVp

        # ############# 2 #############
        # # Moving point
        # Vp=Rtheta@Vr
        # # print(dS)
        # dVp=(-Vp+ds*dS-k1*np.tanh(Ks*e))+2*np.array([1-Vp[0],0,0])
        # dds=-ds-s1*((Vp+dVp-ds*dS)[0])+ks*s1+0.7*ds1
        # # dds=0
        # # self.s=1
        # # dds=0
        # # if s<0.05 and ds < -1:
        # #     dds=0
        # self.dds=dds
        # # Acceleration commands
        # dVr=Rtheta.T@(dVp-dRtheta@Vr)
        # # dVr=dRtheta.T@Vp+Rtheta.T@dVp
        # # print((dRtheta.T@Vp).all()==(Rtheta.T@dRtheta@Vr).all())

        ############# 3 #############
        
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

    def LPF_control_3D_v2(self):
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        # phi,theta,psi=self.state[6:9]
        wr=self.state[9:12]

        Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        dRm=Rm@self.adj(wr)
        
        F=self.path_to_follow.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        Rtheta = Rpath.T@Rm
        s1, y1, w1 = Rpath.T@(X-F.X)
        ks = 1
        ds = (Rtheta@Vr)[0]+ks*s1
        dRpath=ds*F.dR
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        
        
        self.error=100*np.linalg.norm([s1,y1,w1],ord=np.inf)
        self.v1=0
        speed=np.linalg.norm(Vr)
        
        # self.p.plot(time(),F.C,1)
        # self.p.plot(time(),F.dC,2)
        # ds=0
        # self.s=5
        if s<0.05 and ds < -1:
            ds=0
        self.ds=ds
        ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])

        Ke,k0,k1,K_s,Kth,nu_d,Kc,v_e=self.displayer.values
        # self.p.plot(time(),speed)
        # self.p.plot(time(),nu_c,1)
        
        # Low speed
        # 0.45, 0.15, 4.0, 0.15, 0.4, 0.1
        
        # Ke=3
        # k1=3
        # nu_d=1

        e1=np.array([0,y1,w1])
        de1=np.array([0,dy1,dw1])
        e=np.array([s1,y1,w1])

        # v_e=1
        Vp=np.array([nu_d*(1-np.tanh(K_s*np.linalg.norm(e1))),-v_e*np.tanh(Ke*y1),-v_e*np.tanh(Ke*w1)])
        # Vp=np.array([-K_s*s1,-v_e*np.tanh(Ke*y1),-v_e*np.tanh(Ke*w1)])
        # print(np.round(e,2))
        pos=F.X
        ss1=np.vstack((pos,pos+2*F.s1))
        sy1=np.vstack((pos,pos+2*F.y1))
        sw1=np.vstack((pos,pos+2*F.w1))
        self.displayer.s1_arrow.setData(pos=ss1)
        self.displayer.y1_arrow.setData(pos=sy1)
        self.displayer.w1_arrow.setData(pos=sw1)
        # print(Vp)
        # Vp=Vp+np.array([nu_c*(1.2-np.tanh(Kc*F.C)),-0.75*np.tanh(Ke*dy1),-0.75*np.tanh(Ke*dw1)])
        t=-2.5*ds**2*F.C*np.array([1,0,0])
        dVp=-(1-np.tanh(np.array([K_s*np.linalg.norm(e1),Ke*y1,Ke*w1]))**2)*np.array([e1@de1,dy1,dw1])+t
        # dVp=-(1-np.tanh(np.array([K_s*np.linalg.norm(e1),-Ke*y1,-Ke*w1]))**2)*np.array([1,dy1,dw1])
        D=np.diag((K_s*nu_d,v_e*Ke,v_e*Ke))
        dVp=D@dVp
        # print(np.round(dVp,2))
        Vd=Rtheta.T@Vp
        # Vd=Rpath@Vp
        # return Vd
        dVd=dRtheta.T@Vp+Rtheta.T@dVp
        if self.error>30:
            self.I.data=0
        dVr=dVd+k1*(Vd-Vr)+k0*self.I.data
        # print(dVd)
        self.I.data+=(Vd-Vr)
        self.I.data=np.clip(self.I.data,-1,1)
        # dVr=dVr+self.adj(wr)@Vr
        dVr=Kth*np.tanh(dVr/Kth)
        u=Rm@dVr
        u=np.clip(u,-2,2)
        pos=X
        dir=u
        arrow=np.vstack((pos,pos+2*dir))
        self.displayer.control_output.setData(pos=arrow)
        return dVr
    
    def update_state(self,data):
        # speed=data.data[3:6]
        # r = Rotation.from_euler('XYZ', data.data[6:9],degrees=False)
        # r2=Rotation.from_euler('XYZ',[0,0,-data.data[9]],degrees=False)
        # speed=r.apply(speed)
        # speed=r2.apply(speed)
        self.state=np.array([*data.data])
        # self.state[3:6]=speed
    
    def speed_control(self):
        V=self.state[3:6]
        w=self.state[6:9]
        Vd=np.array([0.3,0,0])
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
        # F = path_info_update(self.path_to_follow, s)
        F=self.path_to_follow.local_info(s)
        theta_c = F.psi
        theta = sawtooth(theta_m-theta_c)
        s1, y1 = R(theta_c).T@(X-F.X)
        # psi_a,Kdy1=self.vars
        Kdy1=0.5
        psi_a=pi/3
        delta = -psi_a*tanh(Kdy1*y1)
        beta=sawtooth(delta-theta)
        
        psi = delta
        nu=0.1
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
        X = self.state[0:2]
        Vt = self.state[3:5]
        s = self.s
        theta_m, dtheta_m = self.state[8],self.state[11]
        u, v = Vt
        nu = np.linalg.norm(Vt)
        beta=np.arctan2(v,u)

        # print('Speed:',np.round(nu,2))
        
        # F = path_info_update(self.path_to_follow, s)
        F=self.path_to_follow.local_info(s)

        theta_c = F.psi
        s1, y1 = R(theta_c).T@(X-F.X)
        theta = sawtooth(theta_m-theta_c)
        psi = sawtooth(theta+beta)

        ks = 0.7
        ds = cos(psi)*nu + ks*s1
        self.ds=ds
        # return 0
        dtheta_c = F.C*ds
        ds1, dy1 = R(theta)@Vt-ds*np.array([1-F.C*y1, F.C*s1])

        psi_a=1.3
        y10,k1,k2,k3,k4,k5=1.25,1.5,1.8,0.07,3,1
        delta = -psi_a*tanh(y1/y10)
        ddelta = -psi_a*(1-tanh(y1/y10)**2)*dy1/y10
        dpsi=ddelta+k2*sawtooth(delta-psi)
        dbeta=dpsi-dtheta_m+dtheta_c

        nu_d=0.25
        dnu=k1*(nu_d-nu)
        # theta_m_d=pi/4
        # dtheta_m_d=0
        # ddtheta_m_d=0
        # ddtheta_m=ddtheta_m_d+2*(dtheta_m_d-dtheta_m)+1*(theta_m_d-theta_m)
        # dbeta=-0.25*beta
        # dnu=(2-nu)
        C=R(beta,'z')@np.array([dnu,nu*dbeta,0])
        w=self.state[6:9]
        # k=0.07
        wd=k3*np.tanh(C/k5)
        # D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
        D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
        wd=D@wd
        u=k4*(wd-w)

        return u

    def PLF_3D_Kin(self):
        X = self.state[:3]
        s=self.s

        F=self.path_to_follow.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        s1, y1, w1 = Rpath.T@(X-F.X)
        
        print(100*np.linalg.norm([s1,y1,w1],ord=np.inf))
        
        psi_a=pi/2.2
        e=np.array([s1,y1,w1])
        cross_e=np.cross(np.array([1,0,0]),e)
        k_delta=1
        delta=-psi_a*np.tanh(k_delta*cross_e)
        Rpsi=expm(self.adj(delta))
        Rs=Rpath@Rpsi
        nu=1
        Vr=Rs@np.array([nu,0,0])

        ks=3
        ds = (np.array([nu,0,0])@Rpsi)[0]+ks*s1
        self.ds=ds

        dX=Vr
    
        return dX
            
    def LPF_control_3D(self):
        X = self.state[0:3]
        Vr = self.state[3:6]
        s = self.s
        wr=self.state[9:12]
        Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        dRm=Rm@self.adj(wr)
        
        u, v, w = Vr
        nu = np.linalg.norm(Vr)
        beta=np.arctan2(v,u)
        gamma=np.arcsin(w/(nu+1e-6)) # Check
        Rs=R(beta,'z')@R(gamma,'y')

        
        F=self.path_to_follow.local_info(s)
        Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        dRpath=F.dR
        
        Rtheta = Rpath.T@Rm
        Rpsi=Rtheta@Rs
        
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        
        s1, y1, w1 = Rpath.T@(X-F.X)
        self.error=100*np.linalg.norm([s1,y1,w1],ord=np.inf)


        ks = 2
        # ds = (np.array([nu,0,0])@Rpsi)[0]+ks*s1
        ds=0
        self.ds=ds


        ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])

        # psi_a=1.1

        e=np.array([s1,y1,w1])
        de=np.array([ds1,dy1,dw1])

        cross_e=np.cross(np.array([1,0,0]),e)
        dcross_e=np.cross(np.array([1,0,0]),de)
        try:
            # Ke,psi_a,Kpsi,knu,nu_d,k3,k4,k5=self.vars # Attitude control
            Ke,psi_a,Kpsi,knu,nu_d=self.vars  # Acceleration control
        except:
            # Ke,psi_a,Kpsi,knu,nu_d,k3,k4,k5=0.6,1,2.5,3,1,0.25,4.5,0.9 # Attitude control
            Ke,psi_a,Kpsi,knu,nu_d=0.4,1,2.5,1,1.5 # Acceleration control
            # 0.4,1,2.5,1,1.5 # for nu=1.5
            print('it didnt work')
        # Ke=0.8
        # delta=-psi_a*np.tanh(Ke*cross_e*nu)
        delta=-psi_a*np.tanh(Ke*cross_e)
        # nu_d=1
        # knu=1
        dnu=knu*(nu_d-nu)
        # ddelta=-psi_a*Ke*(1-np.tanh(Ke*cross_e*nu)**2)*(dcross_e*nu+dnu*cross_e)
        ddelta=-psi_a*Ke*(1-np.tanh(Ke*cross_e)**2)*dcross_e
        Rdelta=expm(self.adj(delta))

        # Kpsi=2
        dRpsi=-Kpsi*logm(Rpsi@Rdelta.T)@Rpsi-Rpsi@self.adj(Rdelta.T@ddelta)

        dRs=Rtheta.T@(dRpsi-dRtheta@Rs)
        dVr=nu*dRs[:,0]+dnu*Rs[:,0]
        # dVr=dVr+self.adj(wr)@Vr
        u=dVr
        # u=np.clip(u,-2,2)
        # k3,k4=0.07,2
        # w=self.state[6:9]
        # wd=k3*np.tanh(k5*dVr)
        # D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
        # wd=D@wd
        # u=k4*(wd-w)
        return u


if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass