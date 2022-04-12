#include "comm_pipeline/phase.h"

void PhaseDetected::_enter()
{
    return;
}

void PhaseDetected::_exit()
{
    return;
}

void PhaseDetected::handler()
{
    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        if (get_target.response.isTracked)
        {
            ROS_INFO("Marker found. Transitioning to Visual Servo");
            is_transition_needed_ = true;
            next_phase_type_ = PhaseType::VisualServo;

            // TODO: Do we want to move in detected state?
            sendMoveCommand(get_target.response.position.x, get_target.response.position.y, get_target.response.position.z);
        }
        else
        {
            is_transition_needed_ = true;
            next_phase_type_ = PhaseType::Scan;
            ROS_WARN("Can't find marker. Transitioning back to Scan");
        }
    }
    else
    {
        ROS_ERROR("Call to GetTarget service failed");
    }
}