#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include "mono_odom.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_vo");
  ros::NodeHandle nh;
  monoodom vo;
  std::cout<<"Running"<<std::endl;
  ros::spin();
  std::cout<<"Exiting"<<std::endl;
  return 0; 
}