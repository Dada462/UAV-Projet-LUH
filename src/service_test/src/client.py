import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction,Path,FollowPathGoal
from geometry_msgs.msg import Pose, Point,Quaternion
import numpy as np

def feedback_cb(msg):
    print('Feedback received:', msg)

def main():
    client = actionlib.SimpleActionClient('followPath', FollowPathAction)
    client.wait_for_server()
    print('Server Active')
    print('Sending path')
    p=Path()
    f=lambda t : np.array([3*np.cos(t),3*np.sin(t),0*t+10])
    for t in np.linspace(0,6,4000):
        p.poses.append(Pose(Point(*f(t)),Quaternion()))
    goal = FollowPathGoal(path=p)
    client.send_goal(goal,feedback_cb=feedback_cb)
    print('Path sent')
    # client.cancel_goal()
    client.wait_for_result()
    print('The result is: ',client.get_result())  # A FibonacciResult

if __name__ == '__main__':
    rospy.init_node('pathClient')
    result = main()