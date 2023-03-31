import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Quaternion
from scipy.spatial.transform import Rotation


rospy.init_node('test', anonymous=True)


def position_callback(data):
    position=np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
    q=np.array([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
    angles=Rotation.from_quat(q).as_euler('xyz',degrees=True)
    position=np.round(position,2)
    angles=np.round(angles,2)
    print('Position',position,'Angles',angles)

rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_callback)



position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

x=np.array([1,0,0])
y=np.array([0,1,0])
z=np.array([0,0,1])


rate=rospy.Rate(10)
while not rospy.is_shutdown():
    msg=PoseStamped()
    X=1*x+1*z
    Q=Rotation.from_euler('xyz',angles=[0,0,10],degrees=True).as_quat()
    msg.pose.position=Point(*X)
    msg.pose.orientation=Quaternion(*Q)
    position_pub.publish(msg)
    rate.sleep()
