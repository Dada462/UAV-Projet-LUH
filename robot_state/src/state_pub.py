#!/usr/bin/env python3
"""
Package gathering the state of the robot and republishing it in one array.
The state is published as a Float32MultiArray:
state=[x,y,z,vx,vy,vz,euler_angle_X,euler_angle_Y,euler_angle_Z,w_x,w_y,w_z]
The Euler angles (rd) convention is 'XYZ'. 
w_x,w_y,w_z, are the angular speeds (rd/s) expressed in the body-frame.
The speeds vx,vy and vz (m/s) are expressed in the body-frame of the UAV.
"""

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation


class RobotState():
    def __init__(self):
        self.state = np.zeros(12)
        rospy.init_node('robot_state_listener', anonymous=True)
        rospy.Subscriber("/mavros/local_position/velocity_body",  # This is where the speeds are obtained
                         TwistStamped, self.body_velocity_info)
        # This is where the position is obtained
        rospy.Subscriber("/mavros/vision_pose/pose",
                         PoseStamped, self.local_pose_info)
        self.pub = rospy.Publisher(
            '/robot_state', Float32MultiArray, queue_size=10)
        self.robot_state_pub()

    def body_velocity_info(self, data):
        u, v, w = data.twist.linear.x, data.twist.linear.y, data.twist.linear.z
        lx, ly, lz = data.twist.angular.x, data.twist.angular.y, data.twist.angular.z
        self.state[3:6] = u, v, w
        self.state[9:12] = lx, ly, lz
        data = Float32MultiArray()
        data.data = self.state
        self.pub.publish(data)

    def local_pose_info(self, data):
        q = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        position = data.pose.position.x, data.pose.position.y, data.pose.position.z
        r = Rotation.from_quat(q)
        self.state[:3] = position
        self.state[6:9] = r.as_euler('XYZ')
        data = Float32MultiArray()
        data.data = self.state
        self.pub.publish(data)

    def robot_state_pub(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    RS = RobotState()
