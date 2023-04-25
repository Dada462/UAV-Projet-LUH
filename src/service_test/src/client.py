#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg
from uavr_nav_msgs.msg import FollowPathAction,Path,FollowPathGoal
from time import sleep
# print(Path())

def feedback_cb(msg):
    print('Feedback received:', msg)

def fibonacci_client():
    client = actionlib.SimpleActionClient('followPath', FollowPathAction)
    client.wait_for_server()
    print('it worked')
    goal = FollowPathGoal(path=Path())
    # print(goal.poses)

    # client.send_goal(goal,feedback_cb=feedback_cb)
    # sleep(1)
    client.cancel_goal()
    # r=rospy.Rate(2)
    # for i in range(5):
    #     client.get_state()
    #     r.sleep()
    # client.send_goal(goal)
    # client.wait_for_result()

    # return client.get_result()  # A FibonacciResult

def feedback_callback(self, msg):
    print(msg)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)