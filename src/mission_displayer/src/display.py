import sys
import threading
import rospy
from controller_tools.MissionDisplayer import MainWindow, plot2D
from pyqtgraph.Qt import QtWidgets
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,Quaternion
from scipy.spatial.transform import Rotation
import numpy as np

class MD():
    def __init__(self):
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow()
        self.state=np.zeros(12)
        self.s_pos=np.zeros(3)
        self.error=0
        self.vel=np.zeros(3)
        rospy.init_node('mission_displayer', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state_1)
        rospy.Subscriber('/path_info', Quaternion, self.path_info_callback)
        rospy.Subscriber('/path', Float32MultiArray, self.path_points_callback)
        # rospy.Subscriber('/velodyne', PointCloud2, self.velodyneCallback)
        
        
        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()
        sys.exit(app.exec_())
    
    def main(self):
        speed_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        f=10
        rate = rospy.Rate(f)

        while not rospy.is_shutdown():
            Rm=Rotation.from_euler('XYZ',angles=self.state[6:9],degrees=False)
            if False:
                u=self.displayer.keyboard
                u=np.array([[1,-1,0,0,0,0,0,0],
                            [0,0,1,-1,0,0,0,0],
                            [0,0,0,0,0,0,1,-1],
                            [0,0,0,0,1,-1,0,0]])@u

                msg=TwistStamped()
                u[:3]=Rm.apply(u[:3])
                dz=1.5*(0.5-self.state[2])-1*self.state[5]
                msg.twist.linear=Vector3(*1*u[:2],dz)
                msg.twist.angular=Vector3(0,0,3.5*u[3])
                speed_pub.publish(msg)
            rate.sleep()
    
    def update_state_1(self,data):
        self.state=np.array([*data.data])
        self.displayer.update_state(self.state, self.s_pos, self.error, self.vel)
    
    def path_info_callback(self,msg):
        self.s_pos=np.array([msg.x,msg.y,msg.z])
        self.error=msg.w
    
    def path_points_callback(self,msg):
        data=np.array(msg.data)
        self.displayer.path_points=data.reshape((3,6000)).T
        

if __name__=='__main__':
    m=MD()