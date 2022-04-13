#include "comm_pipeline/phase.h"

Phase::Phase() : is_transition_needed_(false)
{
  ros::NodeHandle nh;

  local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  get_target_client_ = nh.serviceClient<comm_pipeline::GetTarget>("/marker/get_target");
}

void Phase::enter()
{
  ROS_INFO("Entered %s Phase", getName().c_str());
  is_transition_needed_ = false;
  next_phase_type_ = getPhaseType();
}

void Phase::exit()
{
  ROS_INFO("Exiting %s Phase", getName().c_str());
  is_transition_needed_ = false;
  next_phase_type_ = getPhaseType();
}

void Phase::sendMoveCommand(float x, float y, float z)
{
  geometry_msgs::PoseStamped msg;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;

  ROS_INFO("Sending command (%f, %f, %f)", x, y, z);

  local_pos_pub_.publish(msg);
}