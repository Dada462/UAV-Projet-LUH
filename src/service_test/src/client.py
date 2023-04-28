import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction,Path,FollowPathGoal
from geometry_msgs.msg import Pose, Point,Quaternion
from controller_tools.tools import R
import numpy as np
from numpy import pi

def feedback_cb(msg):
    print('Feedback received:', msg)

def main():
    client = actionlib.SimpleActionClient('followPath', FollowPathAction)
    client.wait_for_server()
    print('Sending path')
    p=Path()
    f=lambda t : R(0.15,'x')@np.array([1*(1+0.25*np.sin(4*t))*np.cos(t),1*(1+0.25*np.sin(4*t))*np.sin(t),0*t+0.5])
    # f=lambda t : np.array([t+7,3*np.cos(2*pi*t/7)+5,2*np.cos(2*pi*t/3)+10])

    # f=lambda t : R(0.2,'x')@np.array([1.25*np.cos(t),1.25*np.sin(t),0*t+0.5])
    # f=lambda t : np.array([1.25*np.cos(t),1.25*np.sin(t),0*t+0.5])
    # f=lambda t : R(0.1*t,'x')@(np.array([5*np.cos(t),5*np.sin(t),0*t]))+np.array([0,0,15])
    # f=lambda t : np.array([5*(2+np.sin(10*t))*np.cos(t),5*(2+np.sin(10*t))*np.sin(t),0*t+10])

    for t in np.linspace(-10,30,4000):
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