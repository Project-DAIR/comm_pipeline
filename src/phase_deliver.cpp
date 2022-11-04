#include "comm_pipeline/phase.h"

PhaseDeliver::PhaseDeliver() : in_position_(false), delivery_finished_(false)
{
    ros::NodeHandle param_nh("~");
    param_nh.param("delivery_marker_threshold", marker_threshold_, 0.5f);
    param_nh.param("delivery_height", delivery_height_, 3.0f);

    // TODO: Remove when delivery subsystem is integrated
    float delivery_hold_time;
    param_nh.param("delivery_hold_time", delivery_hold_time, 60.0f);

    delivery_timer_.setDuration(delivery_hold_time);

    start_delivery_pub_ = ros::NodeHandle().advertise<std_msgs::Bool>("/start_delivery", 1000);
    end_delivery_sub_ = ros::NodeHandle().subscribe<std_msgs::Bool>("/done_delivery", 10, &PhaseDeliver::endDeliveryCallback, this);
}

void PhaseDeliver::_enter()
{
    // Reset incase we reenter delivery
    in_position_ = false;
    return;
}

void PhaseDeliver::_exit()
{
    return;
}

void PhaseDeliver::handler()
{
    refinePosition();

    // If in position then run delivery subsystem
    if (in_position_)
    {
        std_msgs::Bool msg;
        msg.data = true;
        start_delivery_pub_.publish(msg);
        delivery_timer_.start();
        ROS_INFO("In position: Running delivery subsystem");
        runDeliverySubsystem();
    }
}

void PhaseDeliver::runDeliverySubsystem()
{
    // TODO: Remove once delivery subsystem is integrated
    if (delivery_timer_.isFinished() || delivery_finished_)
    {
        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Ended;
        ROS_INFO("Finished delivery. Transitioning to Ended");

        if (delivery_finished_) {
            ROS_INFO("Finished delivery due to received command");
        }
        else if (delivery_timer_.isFinished())
        {
            ROS_INFO("Finished delivery due to timeout");
        }
        else {
            ROS_ERROR("This should not have happened");
        }
    }
}

void PhaseDeliver::refinePosition()
{

    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        // If marker not tracked go to Lost state
        if (!get_target.response.isTracked && in_position_)
        {
            ROS_WARN("Lost marker while delivering. Holding position");
            return;
        }
        else if (!get_target.response.isTracked) 
        {
            ROS_WARN("Lost marker, transitioning to Lost state");
            is_transition_needed_ = true;
            next_phase_type_ = PhaseType::Lost;
            return;
        }

        float x_pos = get_target.response.position.x;
        float y_pos = get_target.response.position.y;
        float z_pos = -(get_target.response.position.z - delivery_height_);


        bool within_threshold = abs(x_pos) < marker_threshold_ && 
                                abs(y_pos) < marker_threshold_ && 
                                abs(z_pos) < marker_threshold_;

        if (within_threshold) {
            in_position_ = true;
        }
        else {
            ROS_INFO("Refining position = (%f, %f, %f)", x_pos, y_pos, z_pos);
            sendThrottledMoveCommand(x_pos, y_pos, z_pos);
        }
    }
}

void PhaseDeliver::endDeliveryCallback(const std_msgs::Bool::ConstPtr& msg) {
    delivery_finished_ = msg->data;
}