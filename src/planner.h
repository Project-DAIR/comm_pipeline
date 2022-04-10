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
#include <mavros_msgs/PositionTarget.h>

#include <comm_pipeline/ActivateStag.h>
#include <comm_pipeline/FoundMarker.h>
#include <comm_pipeline/GetTarget.h>

class Planner
{
public:
    Planner(ros::NodeHandle* nodehandle);

private:

    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber state_sub_;
    ros::Subscriber marker_sub_;
    ros::Subscriber nav_output_sub_;
    ros::Subscriber mission_sub_;

    // Publishers
    ros::Publisher local_pos_pub_;

    // Services
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient activate_stag_client_;
    ros::ServiceClient get_target_client_;
    ros::ServiceServer found_marker_server_;

    // Callbacks
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void navOutputCallback(const mavros_msgs::NavControllerOutput::ConstPtr& msg);
    void markerCallback(const geometry_msgs::Point::ConstPtr& msg);
    void missionCallback(const mavros_msgs::WaypointReached::ConstPtr& msg);
    bool foundMarkerCallback(comm_pipeline::FoundMarker::Request &req, comm_pipeline::FoundMarker::Response &res);

    // Initializers
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();

    // Determines if n1 is within tolerance to n2
    bool inline isClose(float n1, float n2, float tol){
        return (abs(n1 - n2) <= tol);
    }

    void activateStag();
    void moveByOffset(float x, float y, float z);

    mavros_msgs::State current_state_;
    mavros_msgs::NavControllerOutput nav_output_;
    geometry_msgs::Point marker_pose_;

    bool is_moving_;
    bool changed_to_guided_;
    bool delivery_waypoint_reached_;
    
    int delivery_waypoint_number_;

    enum class DeliveryState {Scan, Detected, Lost, VisualServo, Deliver, Ended};
    DeliveryState delivery_state_ = DeliveryState::Scan;
};

#endif 