#include "comm_pipeline/phase.h"

PhaseDeliver::PhaseDeliver() : delivery_started_(false) {
    ros::NodeHandle param_nh("~");
    // TODO: Remove when delivery subsystem is integrated
    param_nh.param("delivery_hold_time", delivery_hold_time_, 30);
}

void PhaseDeliver::_enter()
{
    return;
}

void PhaseDeliver::_exit()
{
    return;
}

void PhaseDeliver::handler()
{
    // For now we stay in this state for 30 seconds to simulate delivery
    if (!delivery_started_)
    {
        delivery_start_time_ = ros::Time::now();
        delivery_started_ = true;
        return;
    }

    if (ros::Time::now().sec - delivery_start_time_.sec > 30)
    {
        delivery_started_ = false;

        is_transition_needed_ = true;
        next_phase_type_ =  PhaseType::Ended;
        ROS_INFO("Finished delivery. Transitioning to Ended");
    }

    return;
}