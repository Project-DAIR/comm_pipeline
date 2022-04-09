#include "planner.h"

Planner::Planner(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeClients();
    // ROS_INFO("Comm Constructor");

    is_moving_ = false;
}

void Planner::initializeSubscribers(){
    state_sub_ = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &Planner::state_callback, this);
    nav_output_sub_ = nh.subscribe<mavros_msgs::NavControllerOutput> ("mavros/nav_controller_output", 10, &Planner::nav_output_callback, this);
}

void Planner::initializePublishers(){
    local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
}

void Planner::initializeClients(){
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  
}

void Planner:;state_callback(const mavros_msgs::State::ConstPtr& msg) {}
  current_state_ = *msg;
  // ROS_INFO_STREAM("Current Mode: " << current_state_.mode);
  if(!current_state_.connected){
    return;
  }

  if(!checkAlt(target_alt, 0.5)){
    return;
  }
  
  if(!checkAlt(target_alt, 0.5) && current_state_.mode != "GUIDED"){
    
    ROS_INFO("Guided mode enabled...");
  }

}

void Planner::nav_output_callback(const mavros_msgs::NavControllerOutput::ConstPtr& msg) {
    return;
}

void Planner::marker_callback(const geometry_msgs::Point::ConstPtr& msg){
  // ROS_INFO_STREAM("Current Mode: " << current_state_.mode);
  marker_pose_ = *msg;
}

void Planner::mission_callback(const mavros_msgs::WaypointReached::ConstPtr& msg) {
    return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  Planner planner(&nh);

  ROS_INFO("Communication Initialized... ");
  ros::spin();

  return 0;
}
