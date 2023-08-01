# UAV-Projet-Leibniz-Universität-Hannover

## Overview
This repository is an implementation of 2 path-following algorithms:

    -Lyapunov based.
    -PID based.
This is the branch of the repository with the actual code to be put on the drone.

The main (first) method is based on the Lyapunov method. The PID algorithm is a transformation of the Lyapunov method which is quadratic locally. Because of this quadartic behaviour, when approching the path the command becomes zero quickly. A bias can be observed on straight lines for example because of this. 
The second method deals with this issue by being linear locally and is best suited in this scenario.


## Structure
There are three main packages: **pf_controller_lpf**, **pf_controller_pid** & **robot_state**. More detailed descriptions can be found in the *.py* files.

### Controllers
The first two packages are the two different controllers. They use the tools located in the **controller_tools** script package (contains .py files with useful functions). The parameters of the controllers can be found in the .py files. There are more details on what can be changed and an explanation for each parameter.

### Control Tools
The **controller_tools** has three main files: *ActionServer.py*, *RootStateMachine.py* & *tools.py*. The Action Server handles the path-following, landing and takeoff actions being sent by the user. The Robot State Machine handles the state of the robot and allows user takeover safely. At last, *tools.py* contains the functions necessary to compute the path properties such as curvature, twisting etc.

### State of the UAV
The third package simply gathers the state of the robot from the different topics and republishes it in the form of a multiarray such as:
state=[x,y,z,vx,vy,vz,euler_angle_X,euler_angle_Y,euler_angle_Z,w_x,w_y,w_z]

The Euler angles (rd) convention is 'XYZ'.*w_i* are the angular speeds (rd/s) expressed in the body-frame.

The speeds (m/s) are expressed in the body-frame of the UAV.



## Installation and Launching

### Installation
You can create a folder **uav_controller** inside **YOUR_CATKIN_WORKSPACE/src**.
Place all the files inside the **uav_controller** folder.

Build the packages:

```console
cd YOUR_CATKIN_WORKSPACE
catkin_make
```
### Launching
Depending on the method of control, run the following:

**PID**

```console
. devel/setup.bash
roslaunch pf_controller_pid setup.bash
```

**LPF**

```console
. devel/setup.bash
roslaunch pf_controller_lpf setup.bash
```
Note that the SLAM & Mavros should be running first.