#include "ros/ros.h"
#include "comm_pipeline/ActivateStag.h"

bool activate(comm_pipeline::ActivateStag::Request  &req,
         comm_pipeline::ActivateStag::Response &res)
{
  res.activated = true;
  ROS_INFO("request: x=%d", req.activate);
  ROS_INFO("sending back response: [%d]", res.activated);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("activate_stag", activate);
  ROS_INFO("Ready to receive activation.");
  ros::spin();

  return 0;
}