#!/usr/bin/env python3
from time import sleep
import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, VehicleInfoGet
from mavros_msgs.msg import State
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from controller_tools.ActionServer import ActionServer
from std_msgs.msg import String
import numpy as np
import threading


class RobotModeState():
    def __init__(self):
        global INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING, LANDING
        global STABILIZE, GUIDED, LAND, LOITER
        global HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER, NOT_GUIDED
        global INIT_LAND, INIT_FOLLOWPATH, INIT_HOVER

        # States
        INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING, LANDING = 'INIT', 'LANDED', 'HOVERING', 'CONTROL', 'PILOT', 'TAKEOFF', 'STOPPING', 'LANDING'
        # Modes
        STABILIZE, GUIDED, LAND, LOITER = 'STABILIZE', 'GUIDED', 'LAND', 'LOITER'
        # User inputs
        HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER, NOT_GUIDED = 'HOVER', 'LAND', 'FOLLOWPATH', 'PILOT_TAKEOVER', 'NOT_GUIDED'
        # Init inputs
        INIT_LAND, INIT_FOLLOWPATH, INIT_HOVER = 'INIT_LAND', 'INIT_FOLLOWPATH', 'INIT_HOVER'

        self.state = INIT
        self.blockCommands = True
        self.armed = None
        self.mode = None
        self.takeoff_alt = 0.5
        self.altitude = -1
        self.userInput = None
        self.takeoffAccepted = False
        self.landAccepted = False
        self.takeoff_successful = True

        rospy.Subscriber("/mavros/state", State, self.setState)
        rospy.Subscriber('user_input', String, self.userInputCallback)
        rospy.Subscriber('/robot_state', Float32MultiArray, self.stateCallback)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_srv = rospy.ServiceProxy(
            '/mavros/cmd/takeoff', CommandTOL)
        self.land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.getInfo_srv = rospy.ServiceProxy(
            '/mavros/vehicle_info_get', VehicleInfoGet)

        self.r = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        ros_thread = threading.Thread(target=self.main, daemon=True)
        ros_thread.start()

    def userInputCallback(self, msg):
        userInput = msg.data
        self.userInput = userInput

    def setState(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def main(self):
        f = 15
        rate = rospy.Rate(f)
        i = 0
        while not rospy.is_shutdown():
            self.stateMachine()
            if i % (2*f) == 0:
                print(self)
                i = 0
            i += 1
            rate.sleep()

    def stateMachine(self):
        alt_error = 0.1
        if self.altitude == -1:
            go_on = False
        else:
            go_on = True
        if go_on:
            if self.state == INIT:
                if abs(self.altitude) < alt_error:
                    self.state = LANDED
                else:
                    self.state = HOVERING
            elif self.state == PILOT:
                if self.mode == LAND:
                    self.state == LANDING
                elif self.mode == GUIDED or (self.altitude < 0.1 and self.mode == STABILIZE):
                    self.state = INIT
                self.userInput = ''
            elif self.state == LANDED:
                if self.userInput == HOVER:
                    self.set_mode_srv(0, GUIDED)
                    sleep(0.05)
                    while not self.armed:
                        arm_successful = self.arming_srv(True)
                        if not arm_successful:
                            print(
                                '[INFO] Not able to arm, check the system\'s state')
                            break
                        else:
                            sleep(0.5)
                    resp = self.takeoff_srv(0, 0, 0, 0, self.takeoff_alt)
                    self.takeoffAccepted = resp.success
                    if self.mode == GUIDED and self.armed and self.takeoffAccepted:
                        self.state = TAKEOFF
                        self.takeoff_successful = True
                    else:
                        print('Takeoff not successful', '|Mode = GUIDED?:', self.mode ==
                              GUIDED, '|Is armed?:', self.armed, '|Takeoff Accepted?:', self.takeoffAccepted)
                        self.takeoff_successful = False
                elif self.userInput == LAND:
                    if self.mode != 'LAND':
                        self.land_srv()
                elif self.mode not in [GUIDED, LAND]:
                    if self.mode != STABILIZE or abs(self.altitude) >= alt_error:
                        self.state = PILOT
            elif self.state == TAKEOFF:
                expectedMode = GUIDED
                if self.mode != expectedMode or self.mode == LOITER:
                    self.state = PILOT
                elif self.userInput == LAND:
                    self.state = STOPPING
                else:
                    if abs(self.altitude-self.takeoff_alt) < alt_error:
                        self.state = HOVERING
            elif self.state == HOVERING:
                expectedMode = GUIDED
                if self.mode != expectedMode or self.mode == LOITER:
                    self.state = PILOT
                else:
                    if self.userInput == LAND:
                        self.set_mode_srv(0, LAND)
                        resp = self.land_srv()
                        self.landAccepted = resp.success
                        sleep(0.05)
                        if self.landAccepted and self.mode == LAND:
                            self.state = LANDING
                    elif self.userInput == FOLLOWPATH:
                        self.state = CONTROL
            elif self.state == LANDING:
                expectedMode = LAND
                if self.mode != expectedMode or self.mode == LOITER:
                    self.state = PILOT
                else:
                    if abs(self.altitude) < alt_error:
                        sleep(1)
                        self.state = LANDED
            elif self.state == CONTROL:
                expectedMode = GUIDED
                if self.mode != expectedMode or self.mode == LOITER:
                    self.state = PILOT
                else:
                    if self.userInput == HOVER or self.userInput == LAND:
                        self.state = STOPPING
            elif self.state == STOPPING:
                expectedMode = GUIDED
                if self.mode != expectedMode or self.mode == LOITER:
                    self.state = PILOT
                else:
                    if self.speed >= 0.1:
                        pass
                    elif self.speed < 0.1 and self.userInput == HOVER:
                        self.state = HOVERING
                    elif self.speed < 0.1 and self.userInput == LAND:
                        self.set_mode_srv(0, LAND)
                        resp = self.land_srv()
                        self.landAccepted = resp.success
                        sleep(.05)
                        if self.landAccepted and self.mode == LAND:
                            self.state = LANDING
                    elif self.speed < 0.1:
                        self.state = HOVERING

    def stateCallback(self, msg):
        self.altitude = msg.data[2]
        s = np.array(msg.data[3:5])
        self.speed = np.linalg.norm(s)

    def __repr__(self) -> str:
        msg = 'Armed: '+str(self.armed)+'|Mode: ' + \
            str(self.mode) + '|State: ' + str(self.state)
        if self.userInput != None:
            msg = msg+'|UI:' + self.userInput
        return msg
