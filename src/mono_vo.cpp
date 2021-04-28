#include <ros/ros.h>
#include "mono_odom.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "mono_vo");

   monoodom vo;
   ros::spin();
   return 0; 
}