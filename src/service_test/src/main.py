import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction,FollowPathFeedback,FollowPathResult



class ActionServer():
    def __init__(self):
        self.feedback=FollowPathFeedback()
        self.server= actionlib.SimpleActionServer('test_server', FollowPathAction, execute_cb=self.execute_cb, auto_start = True)
        self.path=None

    def execute_cb(self,goal):
        print('Path received')
        r = rospy.Rate(10)
        path=goal
        while self.distance_to_goal>0.5:
            self.feedback.distance_to_goal=self.distance_to_goal
            self.server.publish_feedback(self.feedback)
            r.sleep()
        self.server.set_succeeded(FollowPathFeedback())

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
    test(s)