#include "ros/ros.h"
#include "comm_pipeline/ActivateStag.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "activator");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<comm_pipeline::ActivateStag>("activate_stag");
  comm_pipeline::ActivateStag srv;
  if (client.call(srv))
  {
    ROS_INFO("received: %d", srv.response.activated);
  }
  else
  {
    ROS_ERROR("Failed to call service activate_stag");
    return 1;
  }

  return 0;
}