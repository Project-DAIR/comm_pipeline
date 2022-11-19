#include "comm_pipeline/phase.h"

PhaseVisualServo::PhaseVisualServo()
{
    ros::NodeHandle param_nh("~");

    param_nh.param("vs_marker_threshold", marker_threshold_, 2.0f);
    param_nh.param("marker_timeout", marker_timeout_, 5.0f);
    param_nh.param("delivery_height", delivery_height_, 5.0f);
    param_nh.param("height_change_step", height_change_step_, 2.0f);
    param_nh.param("required_time_for_stability", required_time_for_stability_, 2.5f);

    is_running_ = false;
    prev_stable_ = false;
    lidar_height_cm_ = 1.0; // 1 is always ignored

    marker_pos_sub_ = ros::NodeHandle().subscribe<geometry_msgs::Point>("marker/smooth_pose", 10, &PhaseVisualServo::markerCallback, this);
    lidar_sub_ = ros::NodeHandle().subscribe<>("/lidar_correction/smooth_lidar", 10, &PhaseVisualServo::lidarCallback, this);
}

void PhaseVisualServo::_enter()
{
    is_running_ = true;
    prev_stable_ = false;

    // Set to current time so we dont just enter handler and immediately go to lost
    marker_last_seen_time_ = ros::Time::now();

    return;
}

void PhaseVisualServo::_exit()
{
    is_running_ = false;
    prev_stable_ = false;
    return;
}

void PhaseVisualServo::handler()
{
    // Check for marker timeout
    if (ros::Time::now() - marker_last_seen_time_ > ros::Duration(marker_timeout_))
    {
        ROS_WARN("Lost marker, transitioning to Lost state");

        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Lost;
        return;
    }
}

void PhaseVisualServo::markerCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    if (!is_running_)
        return;

    marker_last_seen_time_ = ros::Time::now();

    float x_pos = msg->x;
    float y_pos = msg->y;
    float z_pos = 0;

    // If marker is within a threshold then we reduce our height
    if (abs(x_pos) < marker_threshold_ && abs(y_pos) < marker_threshold_)
    {
        // Use lidar if it is not giving us a value of 1, otherwise use the markers z
        float estimated_height = lidar_height_cm_ != 1 ? lidar_height_cm_ / 100.0 : msg->z;
        float height_change = -(estimated_height - delivery_height_);

        // Clamp so we dont change height too quickly
        z_pos = std::min(std::max(height_change, -height_change_step_), height_change_step_);

        // If height is also within the threshold then transition to delivery
        if (abs(x_pos) < 0.1 * marker_threshold_ && abs(y_pos) < 0.1 * marker_threshold_ && abs(z_pos) < 0.25 * marker_threshold_)
        {
            ROS_INFO("Marker within threshold. Waiting for stability...");

            // Dont send any height changes - prevents oscilatting up and down
	    z_pos = 0;

            if (!prev_stable_)
            {
                stable_start_time_ = ros::Time::now();
                prev_stable_ = true;
            }

            if (ros::Time::now() - stable_start_time_ > ros::Duration(required_time_for_stability_))
            {
                ROS_INFO("Stability achieved. Transitioning to Deliver");
                is_transition_needed_ = true;
                next_phase_type_ = PhaseType::Deliver;
                is_running_ = false;
            }
        }
        else
        {
            prev_stable_ = false;
        }
    }

    sendMoveCommand(x_pos, y_pos, z_pos);
}

void PhaseVisualServo::lidarCallback(const std_msgs::Int16::ConstPtr &msg)
{
    if (msg->data < 0)
    {
        lidar_height_cm_ = 1.0;
    }
    else
    {
        lidar_height_cm_ = msg->data;
    }
}
