#include "comm_pipeline/phase.h"

PhaseDeliver::PhaseDeliver() : in_position_(false)
{
    ros::NodeHandle param_nh("~");
    param_nh.param("delivery_marker_threshold", marker_threshold_, 0.5f);
    param_nh.param("delivery_height", delivery_height_, 3.0f);

    // TODO: Remove when delivery subsystem is integrated
    float delivery_hold_time;
    param_nh.param("delivery_hold_time", delivery_hold_time, 60.0f);

    delivery_timer_.setDuration(delivery_hold_time);
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
        delivery_timer_.start();
        ROS_INFO("In position: Running delivery subsystem");
        runDeliverySubsystem();
    }
}

void PhaseDeliver::runDeliverySubsystem()
{
    // TODO: Remove once delivery subsystem is integrated
    if (delivery_timer_.isFinished())
    {
        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Ended;
        ROS_INFO("Finished delivery. Transitioning to Ended");
    }
}

void PhaseDeliver::refinePosition()
{

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


        bool within_threshold = abs(x_pos) < marker_threshold_ && 
                                abs(y_pos) < marker_threshold_ && 
                                abs(z_pos) < marker_threshold_;

        if (within_threshold) {
            in_position_ = true;
        }
        else {
            ROS_INFO("Refining position = (%f, %f, %f)", x_pos, y_pos, z_pos);
            sendThrottledMoveCommand(get_target.response.position.x, get_target.response.position.y, 0);
        }
    }
}