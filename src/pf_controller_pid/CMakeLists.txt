cmake_minimum_required(VERSION 3.0.2)
project(pf_controller_pid)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS src
#  LIBRARIES pf_controller
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
