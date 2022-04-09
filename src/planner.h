#ifndef PLANNER_H_
#define PLANNER_H_

#include <ros/ros.h>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/NavControllerOutput.h>
#include <mavros_msgs/WaypointReached.h>

#include "std_msgs/Float64.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>

class Planner
{
public:
    Planner(ros::NodeHandle* nodehandle);

private:

    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber marker_sub_;
    ros::Subscriber nav_output_sub_;

    ros::ServiceClient set_mode_client_;
    ros::Publisher local_pos_pub_;

    mavros_msgs::State current_state_;
    geometry_msgs::Point marker_pose_;

    bool is_moving_;

    void initializeSubscribers();
    void initializePublishers();
    void initializeClients();

    // Determines if n1 is within tolerance to n2
    bool inline isClose(float n1, float n2, float tol){
        return (abs(n1 - n2) <= tol);
    }

    bool reachedTarget(geometry_msgs::Point t, float tol=0.25);
    bool checkAlt(geometry_msgs::Point t, float tol=0.25);


    // Callbacks
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void nav_output_callback(const mavros_msgs::NavControllerOutput::ConstPtr& msg);
    void marker_callback(const geometry_msgs::Point::ConstPtr& msg);
    void mission_callback(const mavros_msgs::WaypointReached::ConstPtr& msg);
};

#endif 