#include <ros/ros.h>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "std_msgs/Float64.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
// ==============================================================================================================================================================

// variables

mavros_msgs::State current_state;         // current state of vtol, subscribed topic
geometry_msgs::PoseStamped current_pose;  // current position wrt origin, subscribed topic

geometry_msgs::PoseStamped offset;  // move by offset 
geometry_msgs::PoseStamped target_pose;  // target pose wrt origin

mavros_msgs::SetMode offb_set_mode;

float GYM_OFFSET;
// ==============================================================================================================================================================

// callback functions
void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
  // ROS_INFO_STREAM("Current Mode: " << current_state.mode);
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
  // ROS_INFO("Current Pose x: %f y: %f z %f", current_pose.target_pose.position.x, current_pose.target_pose.position.y, current_pose.target_pose.position.z);
}

// ==============================================================================================================================================================

// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
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
void getTargetPose()
{
  target_pose.pose.position.x = offset.pose.position.x + current_pose.pose.position.x;
  target_pose.pose.position.y = offset.pose.position.y + current_pose.pose.position.y;
  target_pose.pose.position.z = offset.pose.position.z + current_pose.pose.position.z;
  ROS_INFO("At Target, Current x: %f y: %f z %f",  current_pose.pose.position.x,  current_pose.pose.position.y,  current_pose.pose.position.z);
  ROS_INFO("Target set to x: %f y: %f z %f",  target_pose.pose.position.x,  target_pose.pose.position.y,  target_pose.pose.position.z);
}

// check if x,y,z offset is within tolerance
bool isClose(float n1, float n2, float tol=0.25){
    return (abs(n1 - n2) <= tol);
}

// check if vtol has reached the set target/offset
bool reachedTarget(geometry_msgs::Point t, float tol=0.25)
{
  if(isClose(current_pose.pose.position.x, t.x, tol=tol) && isClose(current_pose.pose.position.y, t.y, tol=tol) && isClose(current_pose.pose.position.z, t.z, tol=tol)){
    return true;
  }
  else
    return false;
}

// check if vtol is at a given altitude
bool checkAlt(geometry_msgs::Point t, float tol=0.25)
{
  if(isClose(current_pose.pose.position.z, t.z, tol=tol)){
    return true;
  }
  else
    return false;
}

// ==============================================================================================================================================================
// ==============================================================================================================================================================


// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_callback);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, pose_callback);
  
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

// ==============================================================================================================================================================

  geometry_msgs::Point target_alt;
  target_alt.x = 0;
  target_alt.y = 0;
  target_alt.z = 12;

  bool first = false;

  
// ==============================================================================================================================================================

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  
// allow the subscribers to initialize
  ROS_INFO("Initializing...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }


// wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }
    ROS_INFO("Connected...");

// ==============================================================================================================================================================
// ==============================================================================================================================================================

// checks for delivery height - should modify to a gps location check
  while(!checkAlt(target_alt, 0.5)){
          // ROS_INFO("Waiting...");
          // ROS_INFO("Take Off Current Pose x: %f y: %f z %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

          ros::spinOnce();
          ros::Duration(1).sleep();
  }

  ROS_INFO("At delivery location...");

  offb_set_mode.request.custom_mode = "GUIDED";
  while (!offb_set_mode.response.mode_sent)
      {
        ros::Duration(.1).sleep();
        set_mode_client.call(offb_set_mode);
      }
  ROS_INFO("Guided mode enables...");


      sleep(5);

// ==============================================================================================================================================================
// ==============================================================================================================================================================
while (ros::ok() && current_state.connected)
  {

        // ==============================================================================================================================================================
        // =================================================================== Sending Offset ===========================================================================
        // ==============================================================================================================================================================
    if(!first)
    {
          setDestination(0, 2, 0);
          getTargetPose();
          
          local_pos_pub.publish(offset);
          sleep(0.5);
          while(!reachedTarget(target_pose.pose.position, 0.5)){
              // ROS_INFO("Waiting...");
              ROS_INFO("Current Pose x: %f y: %f z %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

              ros::spinOnce();
              ros::Duration(1).sleep();
          }
        first = true;
    }      
        // ==============================================================================================================================================================
        // ==============================================================================================================================================================


    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}