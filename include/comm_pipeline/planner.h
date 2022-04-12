#ifndef PLANNER_H_
#define PLANNER_H_

#include <ros/ros.h>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/NavControllerOutput.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/PositionTarget.h>

#include <comm_pipeline/ActivateStag.h>
#include <comm_pipeline/FoundMarker.h>
#include <comm_pipeline/GetTarget.h>

#include "comm_pipeline/phase_manager.h"

class Planner
{
public:
    Planner(ros::NodeHandle* nodehandle);

    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber marker_sub_;
    ros::Subscriber nav_output_sub_;
    ros::Subscriber mission_sub_;

    // Services
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient activate_stag_client_;
    // ros::ServiceClient get_target_client_;
    ros::ServiceServer found_marker_server_;

    // Callbacks
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void navOutputCallback(const mavros_msgs::NavControllerOutput::ConstPtr& msg);
    void markerCallback(const geometry_msgs::Point::ConstPtr& msg);
    void missionCallback(const mavros_msgs::WaypointReached::ConstPtr& msg);
    bool foundMarkerCallback(comm_pipeline::FoundMarker::Request &req, comm_pipeline::FoundMarker::Response &res);

    // Initializers
    void initializeSubscribers();
    // void initializePublishers();
    void initializeServices();

    void activateStag();
    // void sendMoveCommand(float x, float y, float z);

    // State handlers
    void handleDetectedState();
    void handleVisualServoState();
    void handleDeliverState();

    mavros_msgs::State current_state_;
    mavros_msgs::NavControllerOutput nav_output_;
    geometry_msgs::Point marker_pose_;

    bool changed_to_guided_;
    bool delivery_waypoint_reached_;
    
    // Parameters
    int delivery_waypoint_number_;
    float wp_threshold_;
    
    PhaseManager phase_manager_;
};

#endif 