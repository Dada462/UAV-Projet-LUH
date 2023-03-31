import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3,Vector3Stamped,PoseStamped,Point,Quaternion
from mavros_msgs.msg import AttitudeTarget
from scipy.spatial.transform import Rotation
from scipy.signal import square

rospy.init_node('test', anonymous=True)


def position_callback(data):
    position=np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
    q=np.array([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
    angles=Rotation.from_quat(q).as_euler('xyz',degrees=True)
    position=np.round(position,2)
    angles=np.round(angles,2)
    print('Position',position,'Angles',angles)

rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_callback)

pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

x=np.array([1,0,0])
y=np.array([0,1,0])
z=np.array([0,0,1])


f=10
rate=rospy.Rate(f)
t=0

T=5

while not rospy.is_shutdown():
    if t<10:
        msg=Vector3Stamped()
        ar=5*z
        msg.vector=Vector3(*ar)
        pub.publish(msg)
    else:
        msg=PoseStamped()
        X=1*x+1*z
        msg.pose.position=Point(*X)
        position_pub.publish(msg)
    rate.sleep()
    t+=1/f
