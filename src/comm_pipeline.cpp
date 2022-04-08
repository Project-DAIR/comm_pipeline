#include "comm_pipeline.h"

// ==============================================================================================================================================================
// ==============================================================================================================================================================

CommPipeline::CommPipeline(ros::NodeHandle* nodehandle):nh(*nodehandle){
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeClients();

    target_alt.x = 0;
    target_alt.y = 0;
    target_alt.z = 12;

    marker_pose.x = 0;
    marker_pose.y = 0;
    marker_pose.z = 0;

    is_moving = false;
    prev_changed_to_guided = false;
    is_visual_servoing = false;
    // ROS_INFO("Comm Constructor");
}

void CommPipeline::initializeSubscribers(){
    state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &CommPipeline::state_callback, this);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose", 10, &CommPipeline::pose_callback, this);  
    nav_output_sub = nh.subscribe<mavros_msgs::NavControllerOutput> ("/mavros/nav_controller_output", 10, &CommPipeline::nav_controller_callback, this);
    marker_sub = nh.subscribe<geometry_msgs::Point> ("/marker", 10, &CommPipeline::marker_callback, this);
// minimal_subscriber_ = nh_.subscribe("exampleMinimalSubTopic", 1, &ExampleRosClass::subscriberCallback,this);  
    
}
void CommPipeline::initializePublishers(){
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
}
void CommPipeline::initializeClients(){
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  
}

// ==============================================================================================================================================================
// ==============================================================================================================================================================


// set position to fly to in the gym frame
void CommPipeline::setDestination(float x, float y, float z){
  offset.pose.position.x = x;
  offset.pose.position.y = y;
  offset.pose.position.z = z;
  ROS_INFO("Destination set to x: %f y: %f z %f", x, y, z);
}

// check if x,y,z offset is within tolerance
bool CommPipeline::isClose(float n1, float n2, float tol){
    return (abs(n1 - n2) <= tol);
}

// check if vtol has reached the set target/offset
bool CommPipeline::reachedTarget(geometry_msgs::Point t, float tol){
  if(CommPipeline::isClose(current_pose.pose.position.x, t.x, tol=tol) && 
      CommPipeline::isClose(current_pose.pose.position.y, t.y, tol=tol) && 
      CommPipeline::isClose(current_pose.pose.position.z, t.z, tol=tol)){
    return true;
  }
  else
    return false;
}

// check if vtol is at a given altitude
bool CommPipeline::checkAlt(geometry_msgs::Point t, float tol){
  if(isClose(current_pose.pose.position.z, t.z, tol=tol)){
    return true;
  }
  else
    return false;
}

// ==============================================================================================================================================================
// ==============================================================================================================================================================


// callback functions
void CommPipeline::state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
  // ROS_INFO_STREAM("Current Mode: " << current_state.mode);
  if(!current_state.connected){
    return;
  }

  // if(!checkAlt(target_alt, 0.5)){
  //   return;
  // }
  
  /* 
  Change to guided only when:
    1. Not previously changed into GUIDED
    2. Target altitude is reached
    3. Not already in GUIDED
  */
  if(!prev_changed_to_guided && checkAlt(target_alt, 0.5) && current_state.mode != "GUIDED"){
    offb_set_mode.request.custom_mode = "GUIDED";
    set_mode_client.call(offb_set_mode);
    ROS_INFO("Guided mode enabled...");
    prev_changed_to_guided = true;
  }
}

void CommPipeline::nav_controller_callback(const mavros_msgs::NavControllerOutput::ConstPtr& msg){
  // ROS_INFO("Current Pose x: %f y: %f z %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

  if(!current_state.connected || current_state.mode != "GUIDED"){
    return;
  }

  if (!prev_changed_to_guided) {
    return;
  }

  if(!is_visual_servoing) {
    return;
  }
  
  if(!is_moving){

    // If we are too close then dont bother doing anything
    if (abs(marker_pose.x) < 0.1 && abs(marker_pose.y) < 0.1) {
      return;
    }

    setDestination(marker_pose.x, marker_pose.y, marker_pose.z);
    ROS_INFO("Published Pose x: %f y: %f z %f", offset.pose.position.x, offset.pose.position.y, offset.pose.position.z);
    local_pos_pub.publish(offset);
    is_moving = true;
  }
  else if(msg->wp_dist < 0.2){
    is_moving = false;
    is_visual_servoing = false;
  }
}

void CommPipeline::marker_callback(const geometry_msgs::Point::ConstPtr& msg){
  // ROS_INFO_STREAM("Current Mode: " << current_state.mode);

  // Marker comes in as NED frame but ROS expects ENU frame
  marker_pose.x = msg->x;
  marker_pose.y = msg->y;
  marker_pose.z = 0;

  is_visual_servoing = true;
  // marker_pose.x = msg->y; // East
  // marker_pose.y = msg->x; // North
  // marker_pose.z = -msg->z; // Up
}

void CommPipeline::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_pose = *msg;
}

// ==============================================================================================================================================================
// ==============================================================================================================================================================


// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  CommPipeline comm_pipeline(&nh);
  ROS_INFO("Communication Initialized... ");
  ros::spin();

  return 0;
}