
#include "ros/ros.h"
#include <mavros_msgs/StreamRate.h>

ros::ServiceClient set_stream_rate;

void init(ros::NodeHandle nh) {

  std::string ros_namespace;

  if (!nh.hasParam("namespace")) {

    ROS_INFO("using default namespace");
  } else {
    nh.getParam("namespace", ros_namespace);
    ROS_INFO("using namespace %s", ros_namespace.c_str());
  }

  // mavros set_stream_rate sets if and how fast data is transferred
  set_stream_rate = nh.serviceClient<mavros_msgs::StreamRate>((ros_namespace + "/mavros/set_stream_rate").c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monash_main_node");
  ros::NodeHandle monash_main_node("~");

  ROS_INFO("Starting Monash Drone Main Node");

  init(monash_main_node);

  mavros_msgs::StreamRate stream_rate;
  stream_rate.request.stream_id = 0;
  stream_rate.request.message_rate = 10;
  stream_rate.request.on_off = true;
  ROS_INFO("Making mavros set_stream_rate request at %dHz", stream_rate.request.message_rate);

  // Need to call because data doesn't stream by default for some reason
  set_stream_rate.call(stream_rate);

  ros::spin();

  return 0;
}
