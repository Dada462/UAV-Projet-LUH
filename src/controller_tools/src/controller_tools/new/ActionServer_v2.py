#!/usr/bin/env python
import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction, FollowPathFeedback, FollowPathResult
from uavr_nav_msgs.msg import TakeoffAction, TakeoffFeedback, TakeoffResult
from uavr_nav_msgs.msg import LandAction, LandFeedback, LandResult
from std_msgs.msg import String, Float32MultiArray
from numpy import array, inf
from time import sleep,time

class ActionServer():
    def __init__(self, pfc=None,SM=None):
        global INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING, LANDING
        global STABILIZE, GUIDED, LAND, LOITER
        global HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER
        # States
        INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING, LANDING = 'INIT', 'LANDED', 'HOVERING', 'CONTROL', 'PILOT', 'TAKEOFF', 'STOPPING', 'LANDING'
        # Modes
        STABILIZE, GUIDED, LAND, LOITER = 'STABILIZE', 'GUIDED', 'LAND', 'LOITER'
        # User inputs
        HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER = 'HOVER', 'LAND', 'FOLLOWPATH', 'PILOT_TAKEOVER'

        self.SM=SM
        self.followPath_result=None
        self.takeoff_successful=None
        self.land_result=None

        self.accept_takeoff=True
        self.accept_landing=True

        self.followPathServer = actionlib.SimpleActionServer('followPath', FollowPathAction, execute_cb=self.followPathExecute_cb)
        self.TakeoffServer = actionlib.SimpleActionServer('Takeoff', TakeoffAction, execute_cb=self.TakeoffExecute_cb)
        self.LandServer = actionlib.SimpleActionServer('Land', LandAction, execute_cb=self.LandExecute_cb)
        
        self.user_input_pub = rospy.Publisher('/user_input', String, queue_size=10)
        self.user_input = rospy.Subscriber('/user_input', String, self.userInputCallback)
        self.path_pub = rospy.Publisher('/path', Float32MultiArray, queue_size=1)
        
        self.userInput = ''
        self.path = None
        self.distance_to_goal = inf
        self.height = -inf
        self.pfc = pfc
        
        self.followPathServer.start()
        self.TakeoffServer.start()
        self.LandServer.start()
    
    def userInputCallback(self, msg):
        self.userInput = msg.data

    def followPathExecute_cb(self, goal):
        print('Path received')
        r = rospy.Rate(10)
        result=FollowPathResult()
        followPathFeedback = FollowPathFeedback()
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
        self.SM.userInput='FOLLOWPATH'
        # while self.userInput != 'FOLLOWPATH' and not rospy.is_shutdown():
        #     self.user_input_pub.publish(String('FOLLOWPATH'))
        #     r.sleep()
        self.pfc.init_path(points, speeds, headings)
        pathInterrupted = False
        while self.distance_to_goal > 0.1 and not rospy.is_shutdown():
            if self.followPathServer.is_preempt_requested() or self.SM.userInput != 'FOLLOWPATH':
                pathInterrupted = True
                break
            followPathFeedback.distance_to_goal = self.distance_to_goal
            self.followPathServer.publish_feedback(followPathFeedback)
            r.sleep()
        
        result.distance_to_goal = self.distance_to_goal
        if pathInterrupted:
            print('[FAIL] Following the path was interrupted')
            result.result = 1
            self.followPathServer.set_aborted(result)

        elif not pathInterrupted and not rospy.is_shutdown():
            print('[SUCCES] The path was successfully followed')
            result.result = 0
            self.followPathServer.set_succeeded(result)
        self.pfc.ds = 0
        self.pfc.s = 0
        self.SM.userInput='WAIT'
        # while self.SM.userInput=='WAIT' and not rospy.is_shutdown() and not pathInterrupted:
        #     self.user_input_pub.publish(String('WAIT'))
        #     r.sleep()

    def LandExecute_cb(self,goal):
        self.landing_successful=None
        r = rospy.Rate(15)
        result=LandResult()
        Landing_fb = LandFeedback()
        delay = goal.delay
        delay=delay.to_sec()
        sleep(delay)
        self.SM.userInput=LAND
        if self.SM.state in [PILOT,LANDED]:
            result.result = result.UNKNOWN_ERROR
            self.LandServer.set_aborted(result)
            print('LANDING problem here 2',self.landing_successful,self.accept_landing)
            return
        height=self.SM.altitude
        LandingInterrupted = False
        while height > 0.1 and not rospy.is_shutdown():
            if self.LandServer.is_preempt_requested() or self.SM.state==PILOT:
                LandingInterrupted = True
                print('LANDING problem here 1')
                break
            height=self.SM.altitude
            Landing_fb.distance_to_ground = height
            self.LandServer.publish_feedback(Landing_fb)
            r.sleep()
        
        if LandingInterrupted:
            print('[FAIL] Landing was interrupted')
            result.result = result.UNKNOWN_ERROR
            self.LandServer.set_aborted(result)

        elif not LandingInterrupted and not rospy.is_shutdown():
            print('[SUCCES] Landing was successful')
            result.result = result.SUCCESS
            self.LandServer.set_succeeded(result)

    def TakeoffExecute_cb(self,goal):
        self.takeoff_successful=None
        r = rospy.Rate(15)
        result=TakeoffResult()
        Takeoff_fb = TakeoffFeedback()
        desired_height,delay = goal.agl,goal.delay
        self.SM.takeoff_alt=desired_height
        delay=delay.to_sec()
        sleep(delay)
        self.SM.userInput=HOVER
        # Waiting for a response
        t0=time()
        while self.takeoff_successful==None:
            print('TAKEOFF WAITING FOR RESPONSE')
            if self.takeoff_successful==True:
                break
            elif self.takeoff_successful==False or self.accept_takeoff==False:
                result.result = result.UNKNOWN_ERROR
                self.TakeoffServer.set_aborted(result)
                self.takeoff_successful=None
                self.accept_takeoff=True
                print('problem here',self.takeoff_successful,self.accept_takeoff)
                return
            if (time()-t0)>2:
                print('timeout')
                self.takeoff_successful=False
                break
            r.sleep()
        self.accept_takeoff=False
        height=self.SM.altitude
        TakeoffInterrupted = False
        while abs(height-desired_height) > 0.1 and not rospy.is_shutdown():
            if self.TakeoffServer.is_preempt_requested() or self.SM.state==PILOT:
                TakeoffInterrupted = True
                print('problem here 1')
                break
            height=self.SM.altitude
            Takeoff_fb.current_agl = height
            self.TakeoffServer.publish_feedback(Takeoff_fb)
            r.sleep()
        
        if TakeoffInterrupted:
            print('[FAIL] Takeoff was interrupted')
            result.result = result.UNKNOWN_ERROR
            self.TakeoffServer.set_aborted(result)

        elif not TakeoffInterrupted and not rospy.is_shutdown():
            print('[SUCCES] Takeoff was successful')
            result.result = result.SUCCESS
            self.TakeoffServer.set_succeeded(result)
        self.accept_takeoff=True
        self.takeoff_successful=None

    def set_result(self,action,result):
        if action == 'TAKEOFF':
            self.takeoff_successful=result
        elif action == 'ACCEPT_TAKEOFF':
            self.accept_takeoff=result
        elif action == 'LANDING':
            self.landing_successful=result
        elif action == 'FOLLOWPATH':
            self.follow_path_successful=result
        else:
            print('[ERROR] Action must be TAKEOFF, LANDING or FOLLOWPATH')

if __name__ == '__main__':
    try:
        rospy.init_node('ActionServer')
        s = ActionServer()
    except rospy.ROSInterruptException:
        pass