#!/usr/bin/env python

from time import sleep
import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, VehicleInfoGet
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String
from geometry_msgs.msg import Vector3,PoseStamped,Point


class RobotModeState(rospy.Subscriber):
    def __init__(self):
        super().__init__("/mavros/state", State, self.setState)
        global LANDED, LANDING, TAKEOFF, HOVERING, CONTROL, ARM, PILOT, STABILIZE, GUIDED, LAND, END
        # States
        LANDED, LANDING, TAKEOFF, HOVERING, CONTROL, ARM, PILOT, END = 'LANDED', 'LANDING', 'TAKEOFF', 'HOVERING', 'CONTROL', 'ARM', 'PILOT', 'END'
        # Modes
        STABILIZE, GUIDED, LAND = 'STABILIZE', 'GUIDED', 'LAND'

        self.armed = False
        self.mode = None
        self.state = None
        self.takeoff_alt = 1.5
        self.altitude = 0
        self.userInput = ''

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.getInfo_srv = rospy.ServiceProxy('/mavros/vehicle_info_get', VehicleInfoGet)

        rospy.Subscriber('user_input', String, self.userInputCallback)
        self.command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.command = PositionTarget()
        self.command.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.command.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +PositionTarget.IGNORE_VX+PositionTarget.IGNORE_VY+PositionTarget.IGNORE_VZ
        self.go_home_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        

    def userInputCallback(self, msg):
        self.userInput = msg.data

    def robotStateCallback(self, msg):
        self.altitude = msg.pose.position.z

    def setState(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode
        if (self.userInput == TAKEOFF or self.userInput == '') and abs(self.altitude) > 0.5:
            self.state = HOVERING
        elif self.userInput == LAND or self.verifyAltitude(self.altitude):
            if self.verifyAltitude(self.altitude):
                self.state = LANDED
            else:
                self.state = LANDING
        elif self.userInput == CONTROL:
            self.state = CONTROL

        elif self.userInput == PILOT:
            self.state = PILOT

    def verifyAltitude(self, altitude, ref_alt=0):
        return abs(altitude-ref_alt) <= 0.1

    def __call__(self,msg):
        self.altitude = msg.data[2]

        if self.mode == None or self.state == None:
            print('waiting for the mode states')
            return 0
        # print(self.userInput, self)
        if self.userInput == ARM:
            mode = GUIDED
            self.set_mode_srv(0, mode)
            sleep(0.05)
            self.arming_srv(True)
            sleep(0.1)

        elif self.userInput == TAKEOFF:
            if self.state != HOVERING:
                mode = GUIDED
                self.set_mode_srv(0, mode)
                sleep(0.05)
                self.arming_srv(True)
                sleep(0.05)
                self.takeoff_srv(0, 0, 0, 0, self.takeoff_alt)
                sleep(0.1)

        elif self.userInput == LAND:
            if self.state == LANDED:
                mode = STABILIZE
                self.set_mode_srv(0, mode)
                sleep(0.05)
            elif self.mode == LAND:
                pass
            else:
                self.land_srv()

        elif self.userInput == CONTROL:
            if self.state != CONTROL:
                print('You need to first hover to control the robot')
                # self.command.header.stamp=rospy.Time().now()
                # self.command.acceleration_or_force=Vector3(*command)
                # self.command_pub.publish(self.command)

        elif self.userInput == PILOT:
            print('Pilot takeover mode')

        elif self.userInput == END:
            pass
            # command=PoseStamped()
            # command.pose.position=Point(0,0,self.takeoff_alt)
            # self.go_home_pub.publish(command)
            # self.command.pose.position = Point(0, 0, self.takeoff_alt)
            # self.command_pub.publish(self.command)

    def __repr__(self) -> str:
        return 'Armed: '+str(self.armed)+'|Mode: '+str(self.mode) + '|State: ' + str(self.state)


def main():
    rospy.init_node('test', anonymous=True)

    rate = rospy.Rate(3)
    s = RobotModeState()
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped, s.robotStateCallback)
    while not rospy.is_shutdown():
        command = [5, 5, 5]
        s(command)
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
