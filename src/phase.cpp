#include "comm_pipeline/phase.h"

Phase::Phase() : is_transition_needed_(false)
{
  ros::NodeHandle nh;
  ros::NodeHandle param_nh("~");

  local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  get_target_client_ = nh.serviceClient<comm_pipeline::GetTarget>("/marker/get_target");

  float move_cmd_interval;
  param_nh.param("move_cmd_interval", move_cmd_interval, 2.0f);
  move_timer_.setDuration(move_cmd_interval);
  move_timer_.start();
}

void Phase::enter()
{
  ROS_INFO("Entered %s Phase", getName().c_str());
  is_transition_needed_ = false;
  next_phase_type_ = getPhaseType();

  _enter();
}

void Phase::exit()
{
  ROS_INFO("Exiting %s Phase", getName().c_str());
  is_transition_needed_ = false;
  next_phase_type_ = getPhaseType();

  _exit();
}

void Phase::sendMoveCommand(float x, float y, float z)
{
  geometry_msgs::PoseStamped msg;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;

  ROS_INFO("Sending command (%f, %f, %f)", x, y, z);

  local_pos_pub_.publish(msg);

  move_timer_.restart();
}

void Phase::sendThrottledMoveCommand(float x, float y, float z)
{
  // Limit the rate at which we can send move commands
  if (move_timer_.isFinished())
  {
      sendMoveCommand(x, y, z);
  }
  else {
    ROS_INFO("Skipping command due to throttling = (%f, %f, %f)", x, y, z);
  }
}