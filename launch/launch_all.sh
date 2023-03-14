gnome-terminal --tab -- bash -c "source ../devel/setup.bash && roslaunch uav_launch mavlink.launch"
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && rosrun robot_state state_pub.py"
gnome-terminal --tab -- bash -c "sleep 5 && rqt"
gnome-terminal --tab -- gazebo --verbose iris_ardupilot.world
gnome-terminal --tab -- sim_vehicle.py -v ArduCopter -f gazebo-iris
