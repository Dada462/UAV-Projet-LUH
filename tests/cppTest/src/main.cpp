#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void chatterCallbackVelo(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", "hiii");
  // std::cout<<(msg->data)[0]<<std::endl;
  // std::cout<<"HIIIIIIIIII"<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub1 = n.subscribe("/velodyne", 1000, chatterCallbackVelo);
  ros::spin();

  return 0;
}