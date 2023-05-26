# gnome-terminal --tab -- bash -c "roscore && rosparam set use_sim_time true"
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && roslaunch uav_launch mavlink.launch"
gnome-terminal --tab -- bash -c "source ../devel/setup.bash && rosrun robot_state state_pub.py"
gnome-terminal --tab -- bash -c "sleep 3 && rqt"
gnome-terminal --tab -- gazebo --verbose iris_ardupilot.world
gnome-terminal --tab -- bash -c "sleep 1 && source ../devel/setup.bash && roslaunch uav_gazebo uav.launch"
gnome-terminal --tab -- sim_vehicle.py -m --streamrate=20 -v ArduCopter -f gazebo-iris
