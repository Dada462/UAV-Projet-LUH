#!/usr/bin/env python3
"""
Package dealing with the actions sent by the user.
"""

import rospy
import actionlib
from uavr_nav_msgs.msg import FollowPathAction, FollowPathFeedback, FollowPathResult
from uavr_nav_msgs.msg import TakeoffAction, TakeoffFeedback, TakeoffResult
from uavr_nav_msgs.msg import LandAction, LandFeedback, LandResult
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray
from numpy import array, inf, round
from rospy import sleep


class ActionServer():
    def __init__(self, pfc=None):
        """
        The action server takes into account the state of the robot.
        The action server is initialised in the controllers and takes as the parameter the path following controller (pfc).
        That allows it to access all the attributes, such as for example the state machine (SM).
        """
        global INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING, LANDING
        global STABILIZE, GUIDED, LAND, LOITER
        global HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER
        # States
        INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING, LANDING = 'INIT', 'LANDED', 'HOVERING', 'CONTROL', 'PILOT', 'TAKEOFF', 'STOPPING', 'LANDING'
        # Modes
        STABILIZE, GUIDED, LAND, LOITER = 'STABILIZE', 'GUIDED', 'LAND', 'LOITER'
        # User inputs
        HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER = 'HOVER', 'LAND', 'FOLLOWPATH', 'PILOT_TAKEOVER'

        self.followPath_result = None
        self.takeoff_successful = None
        self.land_result = None

        self.accept_takeoff = True
        self.accept_landing = True

        self.path = None
        self.distance_to_goal = inf
        self.pfc = pfc
        self.SM = self.pfc.sm
        self.battery_threshold = 20  # 20 V for the real drone, 12 V in simulation
        self.battery_voltage = -inf
        self.battery_too_low = False
        self.pathError = False

        rospy.Subscriber('/mavros/battery', BatteryState, self.batteryCallback)
        self.path_pub = rospy.Publisher(
            '/path/points/ptf', Float32MultiArray, queue_size=1)

        self.followPathServer = actionlib.SimpleActionServer(
            'followPath', FollowPathAction, execute_cb=self.followPathExecute_cb)
        self.TakeoffServer = actionlib.SimpleActionServer(
            'takeoff', TakeoffAction, execute_cb=self.TakeoffExecute_cb)
        self.LandServer = actionlib.SimpleActionServer(
            'land', LandAction, execute_cb=self.LandExecute_cb)

        self.followPathServer.start()
        self.TakeoffServer.start()
        self.LandServer.start()

    def followPathExecute_cb(self, goal):
        """
        This is the callback from the path-following Action Server.
        It receives the path, after which, it calls the init_path(path) method from the pfc.
        This init_path(path) method returns true or false depending on wheter or not the path was successfuly computed.
        """
        print('[INFO] Path received')
        r = rospy.Rate(10)
        result = FollowPathResult()
        followPathFeedback = FollowPathFeedback()
        self.path = goal.path
        poses = self.path.poses
        velocities = self.path.velocities
        self.distance_to_goal = inf
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
        # Check that the robot is in air
        if self.SM.state not in [HOVERING, CONTROL]:
            if not self.battery_too_low:
                if self.SM.state == PILOT:
                    result.result = result.USER_TOOK_OVER
                else:
                    result.result = result.UNKNOWN_ERROR
            else:
                result.result = result.LOW_BATTERY
                print('[INFO] Cannot follow the path, battery is too low: ', round(
                    self.battery_voltage, 2), 'V', '< ', self.battery_threshold, 'V')
            self.followPathServer.set_aborted(result)
            return
        # This become true if the user cancels the path, the battery is too low etc.
        pathInterrupted = False
        # if the path computation failed
        if not self.pfc.init_path(points, speeds, headings):
            print('[FAIL] Path computation, check path planner')
            result.result = result.UNKNOWN_ERROR
            self.followPathServer.set_aborted(result)
            return
        else:
            self.SM.userInput = FOLLOWPATH
        """ This also check for path computation errors.
        It was put in place because some errors in the path received
        could not be detected directly. Thus a special variable was needed to detect that.
        """
        if self.pathError:
            self.pfc.end_of_path = self.pfc.state[:3]
            print('[FAIL] Path computation, check path planner')
            result.result = result.UNKNOWN_ERROR
            self.followPathServer.set_aborted(result)
            self.SM.userInput = 'WAIT'
            self.pathError = False
            return
        while self.distance_to_goal > 0.1 and not rospy.is_shutdown():
            if self.followPathServer.is_preempt_requested() or self.SM.userInput != 'FOLLOWPATH' or self.battery_too_low or self.pathError:
                if self.followPathServer.is_preempt_requested():
                    print('[FAIL] Path was cancelled by the planner')
                pathInterrupted = True
                break
            followPathFeedback.distance_to_goal = self.distance_to_goal
            self.followPathServer.publish_feedback(followPathFeedback)
            r.sleep()

        result.distance_to_goal = self.distance_to_goal
        if pathInterrupted:
            self.pfc.end_of_path = self.pfc.state[:3]
            print('[FAIL] Following the path was interrupted')
            if self.SM.state == PILOT:
                result.result = result.USER_TOOK_OVER
            elif self.battery_too_low:
                print('[INFO] Cannot follow the path, battery is too low: ', round(
                    self.battery_voltage, 2), 'V', '< ', self.battery_threshold, 'V')
                result.result = result.LOW_BATTERY
            else:
                result.result = result.UNKNOWN_ERROR
            self.followPathServer.set_aborted(result)
            self.SM.userInput = 'HOVER'

        elif not pathInterrupted and not rospy.is_shutdown():
            F = self.pfc.path_to_follow
            self.pfc.end_of_path = F.local_info(F.s_max).X
            print('[SUCCES] The path was successfully followed')
            result.result = result.SUCCESS
            self.followPathServer.set_succeeded(result)
            self.SM.userInput = 'WAIT'
        self.pfc.ds = 0
        self.pfc.s = 0
        self.SM.userInput = 'WAIT'
        self.pfc.pathIsComputed = False
        self.pathError = False

    def LandExecute_cb(self, goal):
        """
        Same as path-following action server callback but for the landing.
        """
        self.landing_successful = None
        r = rospy.Rate(15)
        result = LandResult()
        Landing_fb = LandFeedback()
        delay = goal.delay
        delay = delay.to_sec()
        sleep(delay)
        if self.SM.state in [PILOT, LANDED]:
            if self.SM.state == PILOT:
                result.result = result.USER_TOOK_OVER
            else:
                result.result = result.UNKNOWN_ERROR
            self.LandServer.set_aborted(result)
            return
        self.SM.userInput = LAND
        height = self.pfc.state[2]
        LandingInterrupted = False
        while height > 0.1 and not rospy.is_shutdown():
            landing_error = abs(height) > 0.1 and self.SM.state != LANDING
            if self.LandServer.is_preempt_requested() or self.SM.state == PILOT or landing_error:
                LandingInterrupted = True
                break
            height = self.pfc.state[2]
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

    def TakeoffExecute_cb(self, goal):
        """
        Same as path-following action server callback but for the takeoff.
        """
        self.takeoff_successful = None
        r = rospy.Rate(15)
        result = TakeoffResult()
        Takeoff_fb = TakeoffFeedback()
        desired_height, delay = goal.agl, goal.delay
        height = self.pfc.state[2]
        self.pfc.takeoff_XYZ = self.pfc.state[:3]
        self.pfc.takeoff_heading = self.pfc.state[8]
        self.SM.takeoff_alt = desired_height
        self.SM.first_stage_alt = height+0.25
        delay = delay.to_sec()
        sleep(delay)
        if self.SM.state != LANDED:
            if self.SM.state == PILOT:
                result.result = result.USER_TOOK_OVER
            else:
                result.result = result.UNKNOWN_ERROR
            self.TakeoffServer.set_aborted(result)
            return
        else:
            if self.battery_too_low:
                result.result = result.LOW_BATTERY
                self.TakeoffServer.set_aborted(result)
                print('[INFO] Cannot takeoff, battery is too low: ', round(
                    self.battery_voltage, 2), 'V', '< ', self.battery_threshold, 'V')
                return
            elif not self.SM.takeoff_successful:
                result.result = result.UNKNOWN_ERROR
                self.TakeoffServer.set_aborted(result)
                return
        self.SM.userInput = HOVER
        height = self.pfc.state[2]
        TakeoffInterrupted = False
        while abs(height-desired_height) > 0.1 and not rospy.is_shutdown():
            height = self.pfc.state[2]
            Takeoff_fb.current_agl = height
            takeoff_error = abs(
                height-desired_height) > 0.1 and self.SM.state != TAKEOFF and self.SM.state not in [HOVERING, LANDED]
            if self.TakeoffServer.is_preempt_requested() or self.SM.state == PILOT or takeoff_error or not self.SM.takeoff_successful:
                TakeoffInterrupted = True
                break
            self.TakeoffServer.publish_feedback(Takeoff_fb)
            r.sleep()

        if TakeoffInterrupted:
            print('[FAIL] Takeoff was interrupted')
            result.result = result.UNKNOWN_ERROR
            self.TakeoffServer.set_aborted(result)
            self.SM.userInput = LAND

        elif not TakeoffInterrupted and not rospy.is_shutdown():
            print('[SUCCES] Takeoff was successful')
            result.result = result.SUCCESS
            self.TakeoffServer.set_succeeded(result)

    def set_result(self, action, result):
        if action == 'TAKEOFF':
            self.takeoff_successful = result
        elif action == 'LANDING':
            self.landing_successful = result
        elif action == 'FOLLOWPATH':
            self.follow_path_successful = result
        else:
            print('[ERROR] Action must be TAKEOFF, LANDING or FOLLOWPATH')

    def batteryCallback(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_too_low = self.battery_voltage < self.battery_threshold


if __name__ == '__main__':
    try:
        rospy.init_node('ActionServer')
        s = ActionServer()
    except rospy.ROSInterruptException:
        pass
