#include "comm_pipeline/phase.h"

PhaseDeliver::PhaseDeliver() : in_position_(false), delivery_finished_(false), delivery_started_(false)
{
    ros::NodeHandle param_nh("~");
    param_nh.param("delivery_marker_threshold", marker_threshold_, 0.5f);
    param_nh.param("delivery_height", delivery_height_, 3.0f);
    param_nh.param("marker_offset", marker_offset_, 0.5f);
    param_nh.param("marker_timeout", marker_timeout_, 5.0f);

    // TODO: Remove when delivery subsystem is integrated
    float delivery_hold_time;
    param_nh.param("delivery_hold_time", delivery_hold_time, 60.0f);

    delivery_timer_.setDuration(delivery_hold_time);
    lidar_height_cm_ = 1; // 1 is always ignored
    is_running_ = false;

    start_delivery_pub_ = ros::NodeHandle().advertise<std_msgs::Bool>("/start_delivery", 1000);
    end_delivery_sub_ = ros::NodeHandle().subscribe<std_msgs::Bool>("/done_delivery", 10, &PhaseDeliver::endDeliveryCallback, this);
    marker_pos_sub_ = ros::NodeHandle().subscribe<geometry_msgs::Point>("marker/smooth_pose", 10, &PhaseDeliver::markerCallback, this);
    lidar_sub_ = ros::NodeHandle().subscribe<>("/lidar_correction/smooth_lidar", 10, &PhaseDeliver::lidarCallback, this);
}

void PhaseDeliver::_enter()
{
    // Reset incase we reenter delivery
    in_position_ = false;
    delivery_finished_ = false;
    delivery_started_ = false;
    is_running_ = true;

    // Set to current time so we dont just enter handler and immediately go to lost
    marker_last_seen_time_ = ros::Time::now();

    return;
}

void PhaseDeliver::_exit()
{
    is_running_ = false;
    return;
}

void PhaseDeliver::handler()
{
    // Check for marker timeout
    if (ros::Time::now() - marker_last_seen_time_ > ros::Duration(marker_timeout_))
    {
        ROS_WARN("Lost marker, transitioning to Lost state");

        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Lost;
        return;
    }

    runDeliverySubsystem();
}

void PhaseDeliver::runDeliverySubsystem()
{
    // TODO: Remove once delivery subsystem is integrated
    if (delivery_timer_.isFinished() || delivery_finished_)
    {
        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Ended;
        ROS_INFO("Finished delivery. Transitioning to Ended");

        if (delivery_finished_)
        {
            ROS_INFO("Finished delivery due to received command");
        }
        else if (delivery_timer_.isFinished())
        {
            ROS_INFO("Finished delivery due to timeout");
        }
        else
        {
            ROS_ERROR("This should not have happened");
        }
    }
}

void PhaseDeliver::endDeliveryCallback(const std_msgs::Bool::ConstPtr &msg)
{
    delivery_finished_ = msg->data;
}

void PhaseDeliver::markerCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    if (!is_running_)
        return;

    marker_last_seen_time_ = ros::Time::now();

    float x_pos = msg->x;
    float y_pos = msg->y - marker_offset_;

    // Use lidar if it is not giving us a value of 1, otherwise use the markers z
    float estimated_height = lidar_height_cm_ != 1 ? lidar_height_cm_ / 100 : msg->z;
    float z_pos = -(estimated_height - delivery_height_);

    bool within_threshold = abs(x_pos) < marker_threshold_ &&
                            abs(y_pos) < marker_threshold_ &&
                            abs(z_pos) < marker_threshold_;

    // If in position then run delivery subsystem
    if (within_threshold)
    {
        // Only publish start delivery once
        if (!delivery_started_)
        {
            delivery_started_ = true;
            std_msgs::Bool msg;
            msg.data = true;
            start_delivery_pub_.publish(msg);
            delivery_timer_.start();
            ROS_INFO("In position: Running delivery subsystem");
        }
    }
    else
    {
        ROS_WARN("Out of threshold after delivery started. Pos = (%f, %f, %f) | Thresh = %f", x_pos, y_pos, z_pos, marker_threshold_);
    }
    
    sendMoveCommand(x_pos, y_pos, z_pos);
}

void PhaseDeliver::lidarCallback(const std_msgs::Int16::ConstPtr &msg)
{
    if (msg->data < 0)
    {
        lidar_height_cm_ = 1;
    }
    else
    {
        lidar_height_cm_ = msg->data;
    }
}