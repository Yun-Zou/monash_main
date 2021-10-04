

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "monash_main_node");

  ROS_INFO("Starting MMS Master Node");


  ros::spin();

  return 0;
}
