cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)


## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
# catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener src/main.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})