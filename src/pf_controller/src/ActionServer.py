import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction,FollowPathFeedback,FollowPathResult
from std_msgs.msg import String
import threading

class ActionServer():
    def __init__(self,pfc=None):
        self.feedback=FollowPathFeedback()
        self.server= actionlib.SimpleActionServer('followPath', FollowPathAction, execute_cb=self.execute_cb)
        self.user_input_pub=rospy.Publisher('/user_input', String, queue_size=10)
        self.path=None
        self.distance_to_goal=1
        self.pfc=pfc
        print('server started')
        self.server.start()

    def execute_cb(self,goal):
        print('Path received')
        r = rospy.Rate(1)
        self.path=goal.path
        # points=[]
        points=-1
        # self.pfc.init_path(points)
        pathInterrupted=False
        while self.distance_to_goal>0.1 and not rospy.is_shutdown():
            if pathInterrupted:
                self.user_input_pub(String('HOVERING'))
                break
            self.user_input_pub.publish(String('CONTROL'))
            self.feedback.distance_to_goal=self.distance_to_goal
            self.server.publish_feedback(self.feedback)
            r.sleep()
        if pathInterrupted:
            self.server.set_aborted()
        else:
            self.server.set_succeeded(FollowPathResult())
        

if __name__ == '__main__':
    rospy.init_node('test')
    def test(s):
        from time import sleep
        s.distance_to_goal=10
        while s.distance_to_goal>0:
            s.distance_to_goal-=0.25
            print(s.distance_to_goal)
            sleep(0.1)
    s=ActionServer()
    # test(s)