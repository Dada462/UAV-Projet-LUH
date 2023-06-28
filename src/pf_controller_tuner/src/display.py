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
        self.p=plot2D()
        self.state=np.zeros(12)
        rospy.init_node('tuning_diplayer', anonymous=True)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.update_state_callback)
        
        ros_thread = threading.Thread(target=self.main,daemon=True)
        ros_thread.start()
        sys.exit(app.exec_())
    
    def main(self):
        f=10
        rate = rospy.Rate(f)
        t0=rospy.Time.now().to_time()
        while not rospy.is_shutdown():
            t=rospy.Time.now().to_time()-t0
            self.p.plot(t,self.state[0],id='x',color='cornflowerblue')
            self.p.plot(t,self.state[1],id='y',color='lime')
            # self.p.plot(t,self.state[2],id='z',color='cornflowerblue')
            self.p.plot(t,2,id='2',color='tomato')
            rate.sleep()
    
    def update_state_callback(self,data):
        self.state=np.array([*data.data])

        

if __name__=='__main__':
    m=MD()