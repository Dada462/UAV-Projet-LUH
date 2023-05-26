#!/usr/bin/env python

from time import sleep
import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, VehicleInfoGet
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3, PoseStamped, Point
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

        self.r = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        ros_thread = threading.Thread(target=self.main, daemon=True)
        ros_thread.start()

    def userInputCallback(self, msg):
        userInput = msg.data
        self.userInput = userInput

    def setState(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def main(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            # print(self)
            self.stateMachine()
            rate.sleep()

    def stateMachine(self):
        if self.altitude == -1:
            go_on = False
        else:
            go_on = True
        if go_on:
            if self.state == INIT:
                if abs(self.altitude) < 0.05:
                    self.state = LANDED
                else:
                    self.state = HOVERING
            elif self.state == PILOT:
                if self.mode == LAND:
                    self.state == LANDING
                elif self.mode == GUIDED:
                    self.state = INIT
                self.userInput = ''
            elif self.state == LANDED:
                if self.userInput == HOVER:
                    self.set_mode_srv(0, GUIDED)
                    sleep(0.05)
                    if not self.armed:
                        self.arming_srv(True)
                        sleep(0.05)
                    resp = self.takeoff_srv(0, 0, 0, 0, self.takeoff_alt)
                    self.takeoffAccepted = resp.success
                    if self.mode == GUIDED and self.armed and self.takeoffAccepted:
                        self.state = TAKEOFF
                    else:
                        print('Takeoff unsuccessful',self.takeoffAccepted)
                elif self.userInput==LAND:
                    self.land_srv()
                elif self.mode not in [GUIDED, LAND]:
                    if self.mode != STABILIZE or abs(self.altitude) >= 0.05:
                        self.state = PILOT
            elif self.state == TAKEOFF:
                expectedMode = GUIDED
                if self.mode != expectedMode or self.mode == LOITER:
                    self.state = PILOT
                else:
                    if abs(self.altitude-self.takeoff_alt) < 0.05:
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
                    if abs(self.altitude) < 0.05:
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

    def stateCallback(self, msg):
        self.altitude = msg.data[2]
        s = np.array(msg.data[3:5])
        self.speed = np.linalg.norm(s)

    def __call__(self):
        pass

    def __repr__(self) -> str:
        return 'Armed: '+str(self.armed)+'|Mode: '+str(self.mode) + '|State: ' + str(self.state)


def main():
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(3)
    s = RobotModeState()
    while not rospy.is_shutdown():
        # s()
        rate.sleep()


if __name__ == "__main__":
    main()


# # Daniel
# if mode==STABILIZE:
#     if userInput==GUIDED:
#         mode='GUIDED'
#         set_mode_srv(0,mode)
#         sleep(0.1)
# elif mode==GUIDED:
#     if userInput==ARM:
#         arming_srv(True)
#         sleep(0.1)
# elif armed:
#     if userInput==TAKEOFF:
#         takeoff_srv(0,0,0,0,takeoff_alt)
#         sleep(0.1)
# elif state==TAKEOFF:
#     verifyTakeoff(takeoff_srv.altitude)
# elif state==HOVERING:
#     if userInput==LANDING:
#         land_srv(0,0,0,0,0)
#         sleep(0.1)
# elif state=='CONTROL':
#     if userInput=='LAND':
#         land_srv(0,0,0,0,0)
#         sleep(0.1)
#     else:
#         # publish command
#         pass
# elif state=='END':
#     if userInput=='RESTART':
#         mode='STABILIZE'
#         set_mode_srv(0,mode)
#         sleep(0.1)
# elif state=='PILOT':
#     pass
