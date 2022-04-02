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

    move = false;
    // ROS_INFO("Comm Constructor");
}

void CommPipeline::initializeSubscribers(){
    state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, &CommPipeline::state_callback, this);  
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, &CommPipeline::pose_callback, this);
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
  float deg2rad = (M_PI/180);
  float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
  float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
  float Z = z;
  offset.pose.position.x = X;
  offset.pose.position.y = Y;
  offset.pose.position.z = Z;
  ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

// calculate target pose wrt to origin(takeoff pose)
void CommPipeline::getTargetPose(){
  target_pose.pose.position.x = offset.pose.position.x + current_pose.pose.position.x;
  target_pose.pose.position.y = offset.pose.position.y + current_pose.pose.position.y;
  target_pose.pose.position.z = offset.pose.position.z + current_pose.pose.position.z;
  ROS_INFO("Setting Target, Current Position x: %f y: %f z %f",  current_pose.pose.position.x,  current_pose.pose.position.y,  current_pose.pose.position.z);
  ROS_INFO("Target set to x: %f y: %f z %f",  target_pose.pose.position.x,  target_pose.pose.position.y,  target_pose.pose.position.z);
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
  if(CommPipeline::isClose(current_pose.pose.position.z, t.z, tol=tol)){
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

  if(!checkAlt(target_alt, 0.5)){
    return;
  }
  
  if(!checkAlt(target_alt, 0.5) && current_state.mode != "GUIDED"){
    offb_set_mode.request.custom_mode = "GUIDED";
    ROS_INFO("Guided mode enabled...");
  }

}

void CommPipeline::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
  // ROS_INFO("Current Pose x: %f y: %f z %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  if(!current_state.connected){
    return;
  }

  if(!checkAlt(target_alt, 0.5)){
    return;
  }

  if(current_state.mode != "GUIDED"){
    return;
  }

  if(move){
    // setDestination(0, 2, 0);
    setDestination(marker_pose.x, marker_pose.y, marker_pose.z);
    getTargetPose();
    local_pos_pub.publish(offset);
    move = false;
  }

  if(!reachedTarget(target_pose.pose.position, 0.5)){
    return;
  }

}

void CommPipeline::marker_callback(const geometry_msgs::Point::ConstPtr& msg){
  // ROS_INFO_STREAM("Current Mode: " << current_state.mode);
  if(!current_state.connected){
    return;
  }
  if(current_state.mode != "GUIDED"){
    return;
  }
  marker_pose = *msg;
  move = true;
}

// ==============================================================================================================================================================
// ==============================================================================================================================================================


// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  CommPipeline comm_pipeline(&nh);

  //the setpoint publishing rate MUST be faster than 2Hz
  // ros::Rate rate(20.0);
  ROS_INFO("Communication Initialized... ");
  ros::spin();

  return 0;
}