import rospy
import numpy as np
from mavros_msgs.msg import PositionTarget,AttitudeTarget,Thrust
from geometry_msgs.msg import TwistStamped,Vector3,PoseStamped,Point,Vector3Stamped,Quaternion
from std_msgs.msg import Float32MultiArray

def update_state(data):
    global state
    state=np.array([*data.data])

def talker():
    global state
    rospy.init_node('talker', anonymous=True)
    # attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
    rospy.Subscriber('/robot_state', Float32MultiArray, update_state)
    state=np.zeros(12)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        u=np.zeros(3)
        msg=Thrust()
        msg.header.stamp=rospy.Time.now()
        msg.thrust=100.

        # msg = AttitudeTarget()
        # msg.type_mask=AttitudeTarget.IGNORE_ATTITUDE
        # print(state[2])
        # z=state[2]
        # zd=10
        # msg.thrust=0.5+2*(zd-z)
        # msg.body_rate=Vector3(*u)
        attitude_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass