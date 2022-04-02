#ifndef COMM_PIPELINE_H_
#define COMM_PIPELINE_H_

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

class CommPipeline
{
public:
    CommPipeline(ros::NodeHandle* nodehandle);

private:

    ros::NodeHandle nh;

    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber marker_sub;

    ros::ServiceClient set_mode_client;
    ros::Publisher local_pos_pub;


    mavros_msgs::State current_state;         // current state of vtol, subscribed topic
    geometry_msgs::PoseStamped current_pose;  // current position wrt origin, subscribed topic

    geometry_msgs::PoseStamped offset;  // move by offset 
    geometry_msgs::PoseStamped target_pose;  // target pose wrt origin

    mavros_msgs::SetMode offb_set_mode;

    float GYM_OFFSET;

    geometry_msgs::Point target_alt;
    geometry_msgs::Point marker_pose;
    bool move;


    void initializeSubscribers();
    void initializePublishers();
    void initializeClients();

    void setDestination(float x, float y, float z);
    void getTargetPose();
    bool isClose(float n1, float n2, float tol=0.25);
    bool reachedTarget(geometry_msgs::Point t, float tol=0.25);
    bool checkAlt(geometry_msgs::Point t, float tol=0.25);


    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void marker_callback(const geometry_msgs::Point::ConstPtr& msg);
};

#endif 