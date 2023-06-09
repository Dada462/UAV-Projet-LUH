#!/usr/bin/env python
import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction, FollowPathFeedback, FollowPathResult
from std_msgs.msg import String, Float32MultiArray
from numpy import array, inf


class ActionServer():
    def __init__(self, pfc=None):
        self.feedback = FollowPathFeedback()
        self.result = FollowPathResult()
        self.server = actionlib.SimpleActionServer(
            'followPath', FollowPathAction, execute_cb=self.execute_cb)
        self.user_input_pub = rospy.Publisher(
            '/user_input', String, queue_size=10)
        self.user_input = rospy.Subscriber(
            '/user_input', String, self.userInputCallback)
        self.path_pub = rospy.Publisher(
            '/path', Float32MultiArray, queue_size=1)
        self.userInput = ''
        self.path = None
        self.distance_to_goal = inf
        self.pfc = pfc
        self.server.start()

    def userInputCallback(self, msg):
        self.userInput = msg.data

    def execute_cb(self, goal):
        print('Path received')
        r = rospy.Rate(10)
        self.path = goal.path
        poses = self.path.poses
        velocities = self.path.velocities
        points = []
        speeds = []
        headings = []
        for i, pose in enumerate(poses):
            points.append([pose.position.x, pose.position.y, pose.position.z])
            speed = velocities[i].linear.x
            heading = velocities[i].angular.z
            speeds.append(speed)
            headings.append(heading)
        points = array(points).T
        path_points = Float32MultiArray()
        path_points.data = points.flatten()
        self.path_pub.publish(path_points)
        while self.userInput != 'FOLLOWPATH' and not rospy.is_shutdown():
            self.user_input_pub.publish(String('FOLLOWPATH'))
            r.sleep()
        self.pfc.init_path(points, speeds, headings)
        pathInterrupted = False
        while self.distance_to_goal > 0.1 and not rospy.is_shutdown():
            if self.server.is_preempt_requested() or self.userInput != 'FOLLOWPATH':
                pathInterrupted = True
                break
            self.feedback.distance_to_goal = self.distance_to_goal
            self.server.publish_feedback(self.feedback)
            r.sleep()

        self.result.distance_to_goal = self.distance_to_goal
        if pathInterrupted:
            print('[FAIL] Following the path was interrupted')
            self.result.result = 1
            self.server.set_aborted(self.result)

        elif not pathInterrupted and not rospy.is_shutdown():
            print('[SUCCES] The path was successfully followed')
            self.result.result = 0
            self.server.set_succeeded(self.result)
        self.pfc.ds = 0
        self.pfc.s = 0
        while self.userInput != 'WAIT' and not rospy.is_shutdown() and not pathInterrupted:
            self.user_input_pub.publish(String('WAIT'))
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('ActionServer')
    s = ActionServer()
