import rospy
import numpy as np
from mavros_msgs.msg import PositionTarget,AttitudeTarget,Thrust
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs
import ros_numpy as rn
import threading

def update_state(data):
    global state
    state=np.array([*data.data])


def velodyneCallback(data):
    global vel
    vel=rn.point_cloud2.pointcloud2_to_xyz_array(data)

def talker():
    global state,vel
    rospy.init_node('talker', anonymous=True)
    # attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
    
    rospy.Subscriber('/velodyne', PointCloud2, velodyneCallback)
    state=np.zeros(12)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

