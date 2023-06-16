#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from mavros_msgs.msg import PositionTarget,AttitudeTarget
from numpy import pi
import threading
from controller_tools.tools import Path_3D,R,sawtooth
from controller_tools.MissionDisplayer import MainWindow, plot2D
from pyqtgraph.Qt import QtWidgets
import sys
from scipy.spatial.transform import Rotation
from controller_tools.RobotStateMachine import RobotModeState
from controller_tools.ActionServer import ActionServer
from controller_tools.Map import Map
from controller_tools.OA import oa_alg
from sensor_msgs.msg import Imu
from time import time
from sensor_msgs.msg import PointCloud2
import ros_numpy as rn


class PID():
    def __init__(self):
        self.data=0
    
    def __call__(self,data,bound=np.inf):
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
        rospy.Subscriber('/velodyne', PointCloud2, self.velodyneCallback)
        
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

    def velodyneCallback(self,msg):
        self.vel=rn.point_cloud2.pointcloud2_to_xyz_array(msg)
    
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
        path_info = rospy.Publisher('/path_info', Quaternion, queue_size=10)
        f=30
        rate = rospy.Rate(f)
        i=0
        self.s=0
        self.sOA=0
        self.ds=0
        self.dsOA=0
        self.I=PID()
        s_pos=vel=np.zeros(3)
        self.error=0
        self.last_dVr=np.zeros(3)
        self.t0=time()
        self.m=Map(50,500)
        kpath=0.55
        # rep=np.zeros(3)
        # vel1=np.zeros(3)
        while not rospy.is_shutdown():
            Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False)
            if self.pathIsComputed:
                s_pos=self.path_to_follow.local_info(self.s).X
                # self.sm.AS.distance_to_goal=np.linalg.norm(s_pos-self.state[:3])+self.path_to_follow.s_max-self.s
                self.pathAction.distance_to_goal=np.linalg.norm(s_pos-self.state[:3])+self.path_to_follow.s_max-self.s
            # t=(distances>0.4)*(distances<3)*(vel[:,2]>-0.4)
            # vel=vel[t]
            # distances=distances[t]

            # print(vel.shape)
            # kvel=0.0015
            # rep=vel.T/distances*1/(1+0.01*distances**10)
            # rep=-kvel*np.sum(rep.T,axis=0)
            # rep,vel,Rm=OA(self.state,self.vel)
            vel1=Rm.apply(self.vel)
            vel1=vel1+self.state[:3]

            # dir=Rm.apply(u)
            # arrow=np.vstack((self.state[:3],self.state[:3]+dir))
            # self.displayer.control_output.setData(pos=arrow)
            # self.m(vel1.T)
            # if i%90==0:
            #     vel2=self.m.array_to_points()
            # map3D=np.where((self.m.data==1))
            self.displayer.update_state(self.state,s_pos,self.error,vel1)
            # print(1/(time()-self.t0))
            # self.t0=time()
            if self.sm.userInput=='KEYBOARD':
                # pass
                u=self.displayer.keyboard
                u=np.array([[1,-1,0,0,0,0,0,0],
                            [0,0,1,-1,0,0,0,0],
                            [0,0,0,0,0,0,1,-1],
                            [0,0,0,0,1,-1,0,0]])@u
                # msg=AttitudeTarget()
                # msg.thrust=dz
                # msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
                # w=self.state[6:9]
                # D=np.array([[0,-1,0],[1,0,0],[0,0,0]])
                # wd=0.15*D@(u[[0,1,3]])
                # msg.body_rate=Vector3(*2*(wd-w)[:2],u[3])
                # attitude_pub.publish(msg)

                msg=TwistStamped()
                u[:3]=Rm.apply(u[:3])
                dz=1.5*(1.5-self.state[2])-1*self.state[5]
                msg.twist.linear=Vector3(*1*u[:2],dz)
                msg.twist.angular=Vector3(0,0,3.5*u[3])
                speed_pub.publish(msg)
            elif self.sm.state=='CONTROL' and self.sm.userInput!='HOME' and self.sm.userInput!='WAIT' and self.pathIsComputed:
                OA=self.oa(self.state,self.sOA,self.vel,self.s)
                if OA:
                    u,heading,ds=self.control_pid_1(self.sOA,self.oa.oa_ptf)
                    Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
                    F=self.path_to_follow.local_info(self.s)
                    Rpath=F.R
                    Rtheta = Rpath.T@Rm
                    Vp=Rtheta@self.state[3:6]
                    ks=2
                    e = Rpath.T@(self.state[:3]-F.X)
                    s1, _,_ = e
                    self.ds=Vp[0]+ks*s1
                    self.dsOA=ds
                    self.oa.sOA=self.sOA
                    self.sOA=self.sOA+1/f*self.dsOA
                    self.sOA=max(0,self.sOA)
                else:
                    u,heading,ds=self.control_pid_1(self.s,self.path_to_follow)
                    self.ds=ds
                    self.sOA=0
                    self.dsOA=0
                ############################## Acceleration Topic ##############################
                command = PositionTarget()
                command.header.stamp=rospy.Time().now()
                command.coordinate_frame = PositionTarget.FRAME_BODY_NED
                command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
                command.type_mask = command.type_mask+PositionTarget.IGNORE_YAW
                command.acceleration_or_force=Vector3(*u)
                command.yaw_rate=command.yaw_rate=1*sawtooth(heading-self.state[8])
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
            path_info.publish(Quaternion(*s_pos,self.error))
            self.s=self.s+1/f*self.ds
            self.s=max(0,self.s)
            i+=1
            rate.sleep()
    
    def init_path(self,points=[],speeds=[],headings=[]):
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(2*t),0*t+15]),[0,15],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([5*cos(t),5*sin(0.9*t),10+0*t]),[-10,10],type='parametric')
        # self.path_to_follow=Path_3D(lambda t : np.array([2*cos(t),2*sin(t),0*t+10]),[-10,30],type='parametric')
        if len(points)!=0:
            # try:
            self.path_to_follow=Path_3D(points,speeds=speeds,headings=headings,type='waypoints')
            self.displayer.path.setData(pos=self.path_to_follow.points[:,:3])
            self.oa=oa_alg(ptf=self.path_to_follow,dt=1/30,r0=1,displayer=self.displayer)
            self.pathIsComputed=True
            # except:
            # print('[ERROR] Path properties were not possible to compute [ERROR]')
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
        self.sOA=0
        self.ds=0
        self.OAds=0

    def control_lpf(self):
        state=self.state
        X = state[0:3]
        Vr = state[3:6]
        s = self.s
        wr=state[9:12]
        dt=1/30

        Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        dRm=Rm@self.adj(wr)
        
        X=X+3*dt*Rm@Vr
        s=s+3*self.ds*dt
        Rm=Rm+3*dRm*dt
        
        F=self.path_to_follow.local_info(s)
        # Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        Rpath=F.R
        
        Rtheta = Rpath.T@Rm
        s1, y1, w1 = Rpath.T@(X-F.X)
        ks = 1.5
        ds = (Rtheta@Vr)[0]+ks*s1
        # ds=0.5
        if s<0.01 and ds<0:
            ds=0
        self.ds=ds
        dRpath=F.dR*ds
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        

        # ds1, dy1,dw1 = Rtheta@Vr-ds*np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        S=np.array([F.k2*w1+F.k1*y1+1,-F.k1*s1,-F.k2*s1]) # PTF
        de=Rtheta@Vr-ds*S
        ds1, dy1,dw1 = de

        # Vpath,k0,k1,kpath,nu_d,c1,amax=self.displayer.values
        Vpath,k0,k1,kpath,_,c1,amax=0.5,1.5,1.5,0.4,1.5,50,0.5
        nu_d=F.speed
        heading=F.heading
        
        
        e=np.array([s1,y1,w1])
        # de=np.array([ds1,dy1,dw1])
        
        e1=np.array([0,y1,w1])
        de1=np.array([0,dy1,dw1])
        self.error=100*np.linalg.norm(e,ord=np.inf)
        speed=np.linalg.norm(Vr)
        self.speed=speed
        
        s=np.linspace(s,s+k0,250)
        Fahead=self.path_to_follow.local_info(s)
        Cahead=np.max(Fahead.C)
        a=np.sqrt(1/(1e-6+Cahead))
        a=np.clip(a,0.25,np.inf)
        nu_d=np.clip(nu_d,0.25,a)
        # nu_d=0.3
        
        vplin=(Rtheta@Vr)[0]
        d_path=np.linalg.norm(e1/kpath)
        ve=nu_d*(1-np.tanh(d_path))
        dve=-nu_d/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        if ((self.path_to_follow.s_max-self.s)<1):
            ve=np.clip(self.path_to_follow.s_max-self.s,-0.5,0.5)
            dve=-np.clip(ds,-0.5,0.5)*(np.abs(ds)<=0.5)
        Vp=-Vpath*np.tanh(e1/kpath)+np.array([ve,0,0])
        Rd=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        data=Rd@self.imuData
        Rd1=Rotation.from_euler('XYZ',[0,0,self.state[8]],degrees=False).as_matrix()
        data=Rd1.T@data
        data[2]=data[2]-9.81
        
        # t=-2*vp1**2*np.array([1,0,0])*np.tanh((Cahead+F.C)/5)*10
        Ke=6*2.4
        d_path1=np.linalg.norm(e/kpath)
        # t=-Ke*np.clip(vplin**2,-3,3)*np.array([1,0,0])*np.tanh(F.C/2)
        # d1=np.argmax(Fahead.C)
        # d1=s[d1]-self.s

        # t=-Ke*np.clip(vplin**2,-2,2)*np.array([1,0,0])*np.tanh(F.C/5)
        # t=t/(1+d_path1)*((self.path_to_follow.s_max-self.s)>0.5)
        # np.clip(t,-1,1)
        # print(t[0],vplin,F.C)
        dVp=-Vpath/kpath*(1-np.tanh(e1/kpath)**2)*de1+np.array([dve,0,0])
        
        Vd=Rtheta.T@Vp
        dVd=dRtheta.T@Vp+Rtheta.T@dVp
        # self.I(0.05*(Vd-Vr),0.25)
        dVr=dVd+k1*(Vd-Vr)
        dVr=dVr+self.adj(wr)@Vr
        dVr=np.tanh(dVr/3)*3
        self.last_dVr=dVr

        # T=0.25
        # dVr=np.sin(pi*(time()-self.t0)/T)*np.array([1,0,0])*1
        

        # self.p.plot(time()-self.t0,t[0],'t','#f5300d')
        # self.p.plot(time()-self.t0,vplin,'dp','#f5af0d')
        # self.p.plot(time()-self.t0,dVr[0],'xd','#f5300d')
        # self.p.plot(time()-self.t0,data[0],'xr','#f5af0d')
        
        # self.p.plot(time()-self.t0,dVr[1],'yd','#49f50d')
        # self.p.plot(time()-self.t0,data[1],'yr','#0df5c0')

        # self.p.plot(time()-self.t0,dVr[2],'zd','#0dd5f5')
        # self.p.plot(time()-self.t0,data[2],'zr','#610df5')

        # self.p.plot(time()-self.t0,F.C,'C','#f50d81')
        # self.p.plot(time()-self.t0,s1,'s1','#f50d4c')
        # self.p.plot(time()-self.t0,y1,'y1','#f50d4c')
        # self.p.plot(time()-self.t0,w1,'w1','#f50d4c')
        # c1,amax=50,0.3
        angle=np.tanh(F.dC/c1)*amax
        # dVr=R(angle,'z')@dVr
        r=Rotation.from_rotvec(F.w1*angle)
        dVr=r.apply(dVr)
        u=Rtheta@dVr
        pos=X
        dir=dVr
        arrow=np.vstack((pos,pos+3*dir))
        self.displayer.control_output.setData(pos=arrow)
        return dVr,heading

    def control_pid(self,kpath=0.55):
        # Ke,k0,k1,Ks,Kth,nu_d,_,vc=self.displayer.values
        
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
        # Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        Rpath=F.R
        
        pos=F.X
        # s1=np.vstack((pos,pos+2*F.s1))
        # y1=np.vstack((pos,pos+2*F.y1))
        # w1=np.vstack((pos,pos+2*F.w1))
        # self.displayer.s1_arrow.setData(pos=s1)
        # self.displayer.y1_arrow.setData(pos=y1)
        # self.displayer.w1_arrow.setData(pos=w1)
       
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
        Ke,_,k0,k1,Kth=2.25,1.5,2,2,3
        vc=F.speed
        heading=F.heading
        # Ke,vc,k0,k1,Kth=self.displayer.values
        
        # Slowing down term when highly curved turn is encountered
        
        # Look ahead curvature
        s=np.linspace(s,s+1.5,50)
        Fahead=self.path_to_follow.local_info(s)
        Cahead=np.max(Fahead.C)
        a=np.sqrt(1.5/(1e-6+Cahead))
        vc=np.clip(vc,0.2,a)

        # kpath=0.55
        d_path=np.linalg.norm(e1/kpath)
        ve=vc*(1-np.tanh(d_path))
        dve=-vc/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        if ((self.path_to_follow.s_max-self.s)<1):
            ve=(self.path_to_follow.s_max-self.s)-0.5*Vp[0]
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

        return dVr,heading

    def control_pid_1(self,s,ptf):
        # Ke,k0,k1,Ks,Kth,nu_d,_,vc=self.displayer.values
        
        # Robot state
        X = self.state[0:3]
        Vr = self.state[3:6]
        # ds=self.ds
        wr=self.state[9:12]

        Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False).as_matrix()
        dRm=Rm@self.adj(wr)
        # dt=1/30
        # X=X+3*dt*Rm@Vr
        # s=s+3*self.ds*dt
        # Rm=Rm+3*dRm*dt
        
        # Path properties
        F=ptf.local_info(s)
        # Rpath=np.vstack((F.s1,F.y1,F.w1)).T
        Rpath=F.R
        
        # pos=F.X
        # s1=np.vstack((pos,pos+2*F.T))
        # y1=np.vstack((pos,pos+2*F.N))
        # w1=np.vstack((pos,pos+2*F.B))
        # self.displayer.s1_arrow.setData(pos=s1)
        # self.displayer.y1_arrow.setData(pos=y1)
        # self.displayer.w1_arrow.setData(pos=w1)
       
        Rtheta = Rpath.T@Rm
        

        # Error and its derivatives
        e = Rpath.T@(X-F.X)
        s1, y1, w1 = e
        # S=np.array([1-F.C*y1, F.C*s1-w1*F.Tr,F.Tr*y1])
        S=np.array([F.k2*w1+F.k1*y1+1,-F.k1*s1,-F.k2*s1]) # PTF
        Vp=Rtheta@Vr
        ks=2
        ds=Vp[0]+ks*s1
        if s<0.05 and ds < 0:
            ds=0
        # self.OAds=ds
        dRpath=ds*F.dR
        dRtheta=dRpath.T@Rm+Rpath.T@dRm
        de = Rtheta@Vr-ds*S
        ds1, dy1,dw1 = de
        # print("|de|",(dy1**2+dw1**2)**0.5,'de',de,'s',s,'ds',ds,'S',S,Rtheta@Vr)
        # dS=np.array([-F.dC*ds*y1-F.C*dy1, F.dC*ds*s1 +F.C*ds1 -dw1*F.Tr-w1*F.dTr*ds,F.Tr*dy1+F.dTr*ds*y1])

        self.error=100*np.linalg.norm(e,ord=np.inf)
        
        
        
        e1=np.array([0,y1,w1])
        de1=np.array([0,dy1,dw1])
        Ke,_,k0,k1,Kth=2.25,1.5,2,2,3
        vc=F.speed
        heading=F.heading
        # Ke,vc,k0,k1,Kth=self.displayer.values
        
        # Slowing down term when highly curved turn is encountered
        
        # Look ahead curvature
        s=np.linspace(s,s+1.5,50)
        Fahead=ptf.local_info(s)
        Cahead=np.max(Fahead.C)
        a=np.sqrt(1.5/(1e-6+Cahead))
        vc=np.clip(vc,0.2,a)

        kpath=0.55
        d_path=np.linalg.norm(e1/kpath)
        ve=vc*(1-np.tanh(d_path))
        dve=-vc/kpath*(1-np.tanh(d_path)**2)*de1@e1/(1e-6+d_path)
        if ((ptf.s_max-self.s)<vc*2):
            ve=(ptf.s_max-self.s)-0.75*Vp[0]
        d_path1=np.linalg.norm(e/kpath)
        t=-Ke*np.clip(Vp[0]**2,-2,2)*np.array([1,0,0])*np.tanh(F.C/5)*6/(1+d_path1)

        dVp=np.array([dve+2*(ve-Vp[0]),0,0])-k1*np.clip(de1,-2,2)-k0*np.clip(e1,-1.5,1.5)+t
        
        # Acceleration commands
        dVr=Rtheta.T@(dVp-dRtheta@Vr)
        dVr=dVr+self.adj(wr)@Vr
        dVr=Kth*np.tanh(dVr/Kth)

        return dVr,heading,ds
    
    def update_state(self,data):
        self.state=np.array([*data.data])
if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass