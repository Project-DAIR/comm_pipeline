#include <ros/ros.h>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

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

geometry_msgs::PoseStamped pose;

float GYM_OFFSET;
// ==============================================================================================================================================================

// callback functions
void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
  // ROS_INFO_STREAM("Current Mode: " << current_state.mode);
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
  // ROS_INFO("Current Pose x: %f y: %f z %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
}

// ==============================================================================================================================================================

// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
  float deg2rad = (M_PI/180);
  float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
  float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
  float Z = z;
  pose.pose.position.x = X;
  pose.pose.position.y = Y;
  pose.pose.position.z = Z;
  ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

// ==============================================================================================================================================================
// ==============================================================================================================================================================


// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 100, state_callback);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 100, pose_callback);
  
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

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
  while(current_state.mode != "GUIDED")
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

// wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }
    ROS_INFO("Connected.");

// ==============================================================================================================================================================
// ==============================================================================================================================================================

// arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  
  while (!srv_arm_i.response.success)
  {
    ros::Duration(.1).sleep();
    arming_client_i.call(srv_arm_i);
  }
  ROS_INFO("Arming...");

// request takeoff
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 10;
  while (!srv_takeoff.response.success)
  {
    ros::Duration(.1).sleep();
    takeoff_cl.call(srv_takeoff);
  }
  ROS_INFO("Takeoff initialized");

  geometry_msgs::Point takeoff_alt;
  takeoff_alt.x = 0;
  takeoff_alt.y = 0;
  takeoff_alt.z = 10;
  while(!checkAlt(takeoff_alt, 0.05)){
      // ROS_INFO("Waiting...");
      ROS_INFO("Take Off Current Pose x: %f y: %f z %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

      ros::spinOnce();
      ros::Duration(1).sleep();
  }


      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if(i == 1)
      {
        ROS_INFO("Failed to reach destination. Stepping to next task.");
      }
    }
    ROS_INFO("Done moving foreward.");
  }
  
  //land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}