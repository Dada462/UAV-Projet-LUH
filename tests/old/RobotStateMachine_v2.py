#!/usr/bin/env python

from time import sleep
import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, VehicleInfoGet
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3, PoseStamped, Point
import sys
states = ['INIT', 'LANDED', 'HOVERING', 'CONTROL', 'PILOT']
transitions = ['HOVER', 'LAND', 'FOLLOWPATH', 'PILOT_TAKEOVER', 'NOT_GUIDED']


class StateMachine():
    def __init__(self, states, transitions):
        self.states = states
        self.transitions = transitions
        self.transitionTree = {}
        for s in states:
            self.transitionTree[s] = {}
            for t in transitions:
                self.transitionTree[s][t] = s

    def goto(self, s1, s2, t):
        self.transitionTree[s1][t] = s2

    def __call__(self, state, transition):
        try:
            next_state = self.transitionTree[state][transition]
            return next_state
        except KeyError:
            print(
                '[WARNING] The transition or the state does not exist in the State Machine')
            print('[INFO] State: ', state,
                  'Transition: ', transition, '[INFO]')
            return state

# print(s('INIT','HOVER'))
# # States
# INIT, LANDED, HOVERING, CONTROL, PILOT='INIT', 'LANDED', 'HOVERING', 'CONTROL', 'PILOT'
# INIT_LAND,INIT_FOLLOWPATH,INIT_HOVER='INIT_LAND','INIT_FOLLOWPATH','INIT_HOVER'

# # Modes
# STABILIZE, GUIDED, LAND, LOITER = 'STABILIZE', 'GUIDED', 'LAND', 'LOITER'
# # User inputs
# HOVER,LAND,FOLLOWPATH,PILOT_TAKEOVER,NOT_GUIDED='HOVER','LAND','FOLLOWPATH','PILOT_TAKEOVER','NOT_GUIDED'


# sys.exit()

class RobotModeState():
    def __init__(self):
        global INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING
        global STABILIZE, GUIDED, LAND, LOITER
        global HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER, NOT_GUIDED
        global INIT_LAND, INIT_FOLLOWPATH, INIT_HOVER

        # States
        INIT, LANDED, HOVERING, CONTROL, PILOT, TAKEOFF, STOPPING = 'INIT', 'LANDED', 'HOVERING', 'CONTROL', 'PILOT', 'TAKEOFF', 'STOPPING'
        # Modes
        STABILIZE, GUIDED, LAND, LOITER = 'STABILIZE', 'GUIDED', 'LAND', 'LOITER'
        # User inputs
        HOVER, LAND, FOLLOWPATH, PILOT_TAKEOVER, NOT_GUIDED = 'HOVER', 'LAND', 'FOLLOWPATH', 'PILOT_TAKEOVER', 'NOT_GUIDED'
        # Init inputs
        INIT_LAND, INIT_FOLLOWPATH, INIT_HOVER = 'INIT_LAND', 'INIT_FOLLOWPATH', 'INIT_HOVER'

        s = StateMachine(states, transitions)
        s.goto(LANDED, HOVERING, HOVER)
        s.goto(HOVERING, LANDED, LAND)
        s.goto(HOVERING, CONTROL, FOLLOWPATH)

        s.goto(CONTROL, HOVERING, HOVER)
        s.goto(CONTROL, LANDED, LAND)

        s.goto(LANDED, PILOT, PILOT_TAKEOVER)
        s.goto(CONTROL, PILOT, PILOT_TAKEOVER)
        s.goto(HOVERING, PILOT, PILOT_TAKEOVER)
        s.goto(INIT, PILOT, PILOT_TAKEOVER)

        s.goto(PILOT, LANDED, LAND)
        s.goto(PILOT, CONTROL, FOLLOWPATH)
        s.goto(PILOT, HOVERING, HOVER)

        s.goto(INIT, LANDED, INIT_LAND)
        s.goto(INIT, CONTROL, INIT_FOLLOWPATH)
        s.goto(INIT, HOVERING, INIT_HOVER)

        self.s = s

        self.state = INIT
        self.blockCommands = True
        self.armed = None
        self.mode = None
        self.takeoff_alt = 1.5
        self.altitude = -1
        self.userInput = None

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

    def userInputCallback(self, msg):
        userInput = msg.data
        # print('Mode: ',self.mode,'| State: ',self.state)
        self.userInput = userInput
        # print(self)
        # if self.altitude == -1:
        # 	return 0
        # if userInput == HOVER or userInput==FOLLOWPATH:
        # 	if self.state != HOVERING and self.state != CONTROL:
        # 		mode = GUIDED
        # 		self.set_mode_srv(0, mode)
        # 		sleep(0.05)
        # 		self.arming_srv(True)
        # 		sleep(0.05)
        # 		self.takeoff_srv(0, 0, 0, 0, self.takeoff_alt)
        # 		sleep(1.5)
        # 	self.userInput=userInput
        # elif userInput == LAND:
        # 	if self.state != LANDED:
        # 		self.land_srv()
        # 	if self.verifyAltitude(self.altitude):
        # 		self.userInput=LAND
        # elif userInput == PILOT_TAKEOVER:
        # 	self.userInput=userInput
        # 	print('Pilot takeover mode')

    def setState(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode
        if self.state == INIT:
            if self.verifyAltitude(self.altitude):
                self.state = LANDED
            else:
                self.state = HOVERING
        elif self.state == LANDED:
            if self.userInput == HOVER:
                if self.mode == GUIDED and self.armed:
                    self.state = TAKEOFF

    def verifyAltitude(self, altitude, ref_alt=0):
        return abs(altitude-ref_alt) <= 0.1

    def stateCallback(self, msg):
        self.altitude = msg.data[2]

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
