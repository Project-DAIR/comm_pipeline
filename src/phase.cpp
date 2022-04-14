#include "comm_pipeline/phase.h"

Phase::Phase() : is_transition_needed_(false), prev_cmd_time_(ros::Time::now())
{
  ros::NodeHandle nh;
  ros::NodeHandle param_nh("~");

  local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  get_target_client_ = nh.serviceClient<comm_pipeline::GetTarget>("/marker/get_target");

  param_nh.param("cmd_time_interval", cmd_time_interval_, 2.0f);
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

  prev_cmd_time_ = ros::Time::now();
}

void Phase::sendThrottledMoveCommand(float x, float y, float z)
{
  // Limit the rate at which we can send move commands
  if (ros::Time::now().sec - prev_cmd_time_.sec >= cmd_time_interval_)
  {
      sendMoveCommand(x, y, z);
  }
  else {
    ROS_INFO("Skipping command due to throttling = (%f, %f, %f)", x, y, z);
  }
}