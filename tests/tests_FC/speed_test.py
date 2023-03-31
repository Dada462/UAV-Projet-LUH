import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped,Vector3



rospy.init_node('test', anonymous=True)
def update_state(data):
    pass

def speed_callback(data):
    speed=np.array([data.twist.linear.x,data.twist.linear.y,data.twist.linear.z])
    angular_speed=np.array([data.twist.angular.x,data.twist.angular.y,data.twist.angular.z])
    speed=np.round(speed,2)
    angular_speed=np.round(angular_speed,2)
    print('Speed',speed,'Angular',angular_speed)

rospy.Subscriber('/robot_state', Float32MultiArray, update_state)
rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, speed_callback)



speed_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
vx=np.array([1,0,0])
vy=np.array([0,1,0])
vz=np.array([0,0,1])

rate=rospy.Rate(10)
while not rospy.is_shutdown():
    msg=TwistStamped()
    Vr=0.1*vx
    Wr=0.1*vz
    msg.twist.linear=Vector3(*Vr)
    msg.twist.angular=Vector3(*Wr)
    speed_pub.publish(msg)
    rate.sleep()
