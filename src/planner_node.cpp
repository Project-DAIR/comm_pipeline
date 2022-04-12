#include <ros/ros.h>
#include "comm_pipeline/planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  Planner planner(&nh);

  ROS_INFO("Communication Initialized... ");
  ros::spin();

  return 0;
}
