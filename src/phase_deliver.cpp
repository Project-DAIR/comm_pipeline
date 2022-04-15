#include "comm_pipeline/phase.h"

PhaseDeliver::PhaseDeliver() : in_position_(false), delivery_started_(false)
{
    ros::NodeHandle param_nh("~");
    param_nh.param("delivery_marker_threshold", marker_threshold_, 0.5f);
    param_nh.param("delivery_height", delivery_height_, 3.0f);

    // TODO: Remove when delivery subsystem is integrated
    param_nh.param("delivery_hold_time", delivery_hold_time_, 30);
}

void PhaseDeliver::_enter()
{
    // Reset incase we reenter delivery
    in_position_ = false;
    delivery_started_ = false;
    return;
}

void PhaseDeliver::_exit()
{
    return;
}

void PhaseDeliver::handler()
{
    // If in position then run delivery subsystem
    if (in_position_)
    {
        ROS_INFO("In position: Running delivery subsystem");
        runDeliverySubsystem();
    }
    else {
        ROS_INFO("Improving position accuracy");
        refinePosition();
    }
}

void PhaseDeliver::runDeliverySubsystem()
{
    // For now we stay in this state for 30 seconds to simulate delivery
    if (!delivery_started_)
    {
        delivery_start_time_ = ros::Time::now();
        delivery_started_ = true;
        return;
    }

    // TODO: Remove once delivery subsystem is integrated
    if (ros::Time::now() - delivery_start_time_ > ros::Duration(delivery_hold_time_))
    {
        delivery_started_ = false;

        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Ended;
        ROS_INFO("Finished delivery. Transitioning to Ended");
    }
}

void PhaseDeliver::refinePosition() {

    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        // If marker not tracked go to Lost state
        if (!get_target.response.isTracked)
        {
            ROS_WARN("Lost marker, transitioning to Lost state");

            is_transition_needed_ = true;
            next_phase_type_ = PhaseType::Lost;
            return;
        }

        float x_pos = get_target.response.position.x;
        float y_pos = get_target.response.position.y;
        float z_pos = -(get_target.response.position.z - delivery_height_);

        ROS_WARN("%f, %f, %f, %f", x_pos, y_pos, z_pos, marker_threshold_);

        // If marker is within a threshold then we are ready to deliver
        if (abs(x_pos) < marker_threshold_ &&
            abs(y_pos) < marker_threshold_ &&
            abs(z_pos) < marker_threshold_)
        {
            in_position_ = true;
        }

        sendThrottledMoveCommand(get_target.response.position.x, get_target.response.position.y, 0);
    }
}