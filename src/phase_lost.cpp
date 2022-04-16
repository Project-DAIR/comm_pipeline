#include "comm_pipeline/phase.h"

PhaseLost::PhaseLost() : moved_to_altitude_(false),
                         moved_to_last_known_(false)
{
    float behaviour_wait_time;
    
    ros::NodeHandle param_nh("~");
    param_nh.param("altitude_increase", altitude_increase_, 2.0f);
    param_nh.param("behaviour_wait_time", behaviour_wait_time, 2.0f);

    behaviour_timer_.setDuration(behaviour_wait_time);
    current_behaviour_ = Behavior::LastKnownLocation;
}

void PhaseLost::_enter()
{
    current_behaviour_ = Behavior::LastKnownLocation;
    moved_to_altitude_ = false;
    moved_to_last_known_ = false;
}

void PhaseLost::_exit()
{
    return;
}

void PhaseLost::handler()
{
    switch (current_behaviour_)
    {
    case Behavior::LastKnownLocation:
        moveToLastKnownLocation();
        break;

    case Behavior::AltitudeIncrease:
        moveToHigherAltitude();
        break;

    case Behavior::RestartScan:
        moveToStartOfScan();
        break;
    }
}

void PhaseLost::foundMarker()
{
    // If the marker is found then go to Visual Servo state
    ROS_INFO("Refound marker, transitioning to Visual Servo state");
    is_transition_needed_ = true;
    next_phase_type_ = PhaseType::VisualServo;
}

void PhaseLost::moveToLastKnownLocation()
{
    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        if (get_target.response.isTracked)
        {
            foundMarker();
            return;
        }

        // If we have moved to the last known location then 
        if (moved_to_last_known_)
        {
            // Doesnt do anything if timer has already started
            behaviour_timer_.start();

            // Dont change behaviour too quickly, give camera time to see marker
            if (behaviour_timer_.isFinished())
            {
                ROS_INFO("Marker still lost. Attempting next behaviour...");
                current_behaviour_ = Behavior::AltitudeIncrease;
                behaviour_timer_.stop();
            }

            return;
        }

        ROS_INFO("Moving to last known marker location");

        float x_pos = get_target.response.position.x;
        float y_pos = get_target.response.position.y;
        float z_pos = 0;

        sendMoveCommand(x_pos, y_pos, z_pos);
        moved_to_last_known_ = true;
    }
}

void PhaseLost::moveToHigherAltitude()
{
    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        if (get_target.response.isTracked)
        {
            foundMarker();
            return;
        }

        if (moved_to_altitude_)
        {
            behaviour_timer_.start();

            // Dont change behaviour too quickly, give camera time to see marker
            if (behaviour_timer_.isFinished())
            {
                ROS_INFO("Marker still lost. Attempting next behaviour...");
                current_behaviour_ = Behavior::RestartScan;
                behaviour_timer_.stop();
            }

            return;
        }

        ROS_INFO("Increasing altitude by %f", altitude_increase_);
        sendMoveCommand(0, 0, altitude_increase_);
        moved_to_altitude_ = true;
    }
}

void PhaseLost::moveToStartOfScan()
{
    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        if (get_target.response.isTracked)
        {
            foundMarker();
            return;
        }

        ROS_INFO("Going back to Scan.");
        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Scan;
    }
}