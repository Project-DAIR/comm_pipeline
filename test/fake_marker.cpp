#include "ros/ros.h"
#include "comm_pipeline/ActivateStag.h"
#include "comm_pipeline/GetTarget.h"
#include "comm_pipeline/FoundMarker.h"

int counter = 15;
float final_x = 2.2;

bool activate(comm_pipeline::ActivateStag::Request &req,
              comm_pipeline::ActivateStag::Response &res)
{
  res.activated = true;
  ROS_INFO("ActivateStag: [%d]", res.activated);
  return true;
}

bool get_target(comm_pipeline::GetTarget::Request &req,
                comm_pipeline::GetTarget::Response &res)
{
  res.position.x = 1;
  res.position.y = 1;
  res.position.z = counter;
  counter -= 2;

  counter = std::max(counter, 7);
  res.isTracked = true;

  if(counter <= 7) {
    res.isTracked = false;
  }

  ROS_INFO("GetTarget: [%f, %f, %f], %d", res.position.x, res.position.y, res.position.z, res.isTracked);

  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "marker");
  ros::NodeHandle n("~");

  ros::ServiceServer activate_stag_service = n.advertiseService("activate_stag", activate);
  ros::ServiceServer get_target_service = n.advertiseService("get_target", get_target);
  ROS_INFO("Ready to receive service calls.");
  ros::spin();

  return 0;
}