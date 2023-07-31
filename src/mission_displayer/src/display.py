import sys
import threading
import rospy
from controller_tools.MissionDisplayer import MainWindow, plot2D
from pyqtgraph.Qt import QtWidgets
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,Quaternion
from scipy.spatial.transform import Rotation
import numpy as np
import ros_numpy as rn
from sensor_msgs.msg import PointCloud2

class MD():
    def __init__(self):
        app = QtWidgets.QApplication(sys.argv)
        self.displayer=MainWindow()
        self.state=np.zeros(12)
        self.s_pos=np.zeros(3)
        self.error=0
        self.vel=np.zeros(3)
        self.closest_vel_point=np.zeros(3)
        self.closest_vel_point_distance=np.inf
        rospy.init_node('mission_displayer', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state_1)
        rospy.Subscriber('/path/info', Quaternion, self.path_info_callback)
        rospy.Subscriber('/path/points/ptf', Float32MultiArray, self.path_points_callback)
        rospy.Subscriber('/path/points/oa', Float32MultiArray, self.oa_path_points_callback)
        rospy.Subscriber('/velodyne', PointCloud2, self.velodyneCallback)
        # self.p=plot2D()
        
        
        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()
        sys.exit(app.exec_())
    
    def velodyneCallback(self, msg):
        vel = rn.point_cloud2.pointcloud2_to_xyz_array(msg)
        distances = np.linalg.norm(vel, axis=1)
        t = (distances > 0.4)*(vel[:, 2] > -0.35)
        distances=distances[t]
        p=np.argmin(distances)
        vel = vel[t]
        Rm = Rotation.from_euler('XYZ', angles=self.state[6:9], degrees=False)
        X = self.state[:3]
        self.vel = (Rm.apply(vel)+X)
        self.closest_vel_point=self.vel[p]
        self.closest_vel_point_distance=distances[p]
    
    def main(self):
        f=10
        rate = rospy.Rate(f)
        # t0=rospy.Time.now().to_time()
        while not rospy.is_shutdown():
            # t=rospy.Time.now().to_time()-t0
            # self.p.plot(t,1,id='1',color='cornflowerblue')
            # print(t)
            rate.sleep()
    
    def update_state_1(self,data):
        self.state=np.array([*data.data])
        self.displayer.update_state(self.state, self.s_pos, self.error, self.vel,self.closest_vel_point,self.closest_vel_point_distance)
    
    def path_info_callback(self,msg):
        self.s_pos=np.array([msg.x,msg.y,msg.z])
        self.error=msg.w
    
    def path_points_callback(self,msg):
        data=np.array(msg.data)
        n=len(data)//3
        self.displayer.path_points=data.reshape((3,n)).T

    def oa_path_points_callback(self,msg):
        data=np.array(msg.data)
        n=len(data)//3
        self.displayer.oa_path_points=data.reshape((3,n)).T
        

if __name__=='__main__':
    m=MD()