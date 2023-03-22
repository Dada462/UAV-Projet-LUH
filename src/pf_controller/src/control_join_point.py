#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from numpy import cos, sin, tanh, pi
from tools import sawtooth, R, path_info_update,mat_reading
from mavros_msgs.msg import PositionTarget
from scipy import signal
from scipy.spatial.transform import Rotation

class PFController():
    def __init__(self):
        rospy.init_node('pf_controller', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state)
        self.state=np.zeros(6)
        self.main()

    def main(self):
        command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        desired_pos= rospy.Publisher('/desired_pos', Vector3, queue_size=10)
        go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        f=20
        rate = rospy.Rate(f)
        i=0
        while not rospy.is_shutdown():
            T=5
            t=i/f
            self.Pd=signal.square(2 * np.pi * t/(2*T))*np.array([-1,1])*10

            # PID
            # u=self.controller(self.state)
            # u=6*np.tanh(u)
            # command = PositionTarget()
            # command.header.stamp=rospy.Time().now()
            # command.coordinate_frame = PositionTarget.FRAME_BODY_NED
            # command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
            # command.acceleration_or_force=Vector3(u[0],u[1],0.)
            # command.yaw=u[2]
            # command_pub.publish(command)


            # # With FC
            command=PoseStamped()
            command.pose.position=Point(*self.Pd,10)
            r=Rotation.from_euler('ZYX',[135+90,0,0],degrees=True).as_quat()
            command.pose.orientation=Quaternion(*r)
            go_home_pub.publish(command)


            dp=Vector3(*self.Pd,0)
            desired_pos.publish(dp)
            i+=1
            rate.sleep()

    def update_state(self,data):
        self.state=np.array([*data.data])[[0,1,3,4,6,11]]
        r = Rotation.from_euler('ZYX', data.data[6:9],degrees=False)
        r2=Rotation.from_euler('ZYX',[-data.data[6],0,0],degrees=False)
        speed=r.apply(data.data[3:6])
        speed=np.round(r2.apply(speed),2)
        print('Heading: ',*np.round(np.array(data.data[6:9])/pi*180,2))
        print('Speed: ',*speed)
    
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

        Pd=self.Pd
        P=X
        dP = R(theta_m)@Vt
        alpha1 = 0.4
        alpha0 = 3
        dPd = 0
        ddPd = 0
        ddP = ddPd+alpha1*(dPd-dP)+alpha0*tanh((Pd-P)/10)
        # print('Parameters',Pd-P,dPd-dP)
        # ddP=self.Pd
        u = R(theta_m).T@ddP
        return u[0], u[1],1


if __name__ == '__main__':
    try:
        C=PFController()
    except rospy.ROSInterruptException:
        pass