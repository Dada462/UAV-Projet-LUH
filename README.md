# UAV-Projet-Leibniz-Universität-Hannover

## With the launch
```console
cd launch
. launch_all.sh
```

## Manually
Open a new terminal for each line
```console
cd launch && . ../../../devel/setup.bash
roslaunch uav_launch mavlink.launch
rqt
gazebo --verbose iris_ardupilot.world
sim_vehicle.py -v ArduCopter -f gazebo-iris
```
