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
    # f=lambda t : R(0.15,'x')@np.array([1*(1+0.25*np.sin(5*t))*np.cos(t),1*(1+0.25*np.sin(5*t))*np.sin(t),0*t+0.5])
    # f=lambda t : np.array([t+7,1*np.cos(2*pi*t/2.5)+5,0*np.cos(2*pi*t/3)+3])

    # f=lambda t : R(0.2*t,'x')@np.array([1*np.cos(t),1*np.sin(t),0.5])+np.array([0,0,2.5])
    # f=lambda t : np.array([t,-6*t*(t-3)-6,0])+np.array([0,0,1.5])
    n=10
    r=10
    f=lambda t : np.array([r*np.cos(t),np.sign(np.sin(t))*(r**n-(r*np.cos(t))**n)**(1/n),0*t+1.5])
    
    # f=lambda t : R(0.1*t,'x')@(np.array([5*cos(t),5*sin(t),0*t]))+np.array([0,0,15])
    # f=lambda t : np.array([1*cos(t),1*sin(t),0*t])+np.array([0,0,10])
    # f=lambda t : R(t,'y')@np.array([5,0,0])+np.array([0,0,10])+np.array([0*t,sin(15*t),0*t])
    # points=[]
    # for t in np.linspace(-10,20,4000):
    #     points.append(f(t))
    # points=np.array(points).T
    # self.path_to_follow=Path_3D(points,type='waypoints')
    # f=lambda t : np.array([(0.5+0.5*t)*1.25*np.cos(t),(0.5+0.5*t)*1.25*np.sin(t),0*t+1])
    # f=lambda t : np.array([1.25*np.cos(t),1.25*np.sin(t),0*t+0.5])
    # f=lambda t : R(0.1*t,'x')@(np.array([5*np.cos(t),5*np.sin(t),0*t]))+np.array([0,0,15])
    # f=lambda t : np.array([2*(1.5+np.sin(3.5*t))*np.cos(t),2*(1.5+np.sin(3.5*t))*np.sin(t),0*t+5])

    for t in np.linspace(0,2*pi,4000):
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