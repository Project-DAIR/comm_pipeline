#include "comm_pipeline/phase.h"

PhaseVisualServo::PhaseVisualServo()
{
    ros::NodeHandle param_nh("~");

    param_nh.param("marker_threshold_", marker_threshold_, 0.25f);
    param_nh.param("delivery_height", delivery_height_, 5.0f);
    param_nh.param("height_change_step", height_change_step_, 2.0f);
}

void PhaseVisualServo::_enter()
{
    return;
}

void PhaseVisualServo::_exit()
{
    return;
}

void PhaseVisualServo::handler()
{
    comm_pipeline::GetTarget get_target;
    if (get_target_client_.call(get_target))
    {
        // If marker not tracked go to Lost state
        if (!get_target.response.isTracked)
        {
            ROS_WARN("Lost marker, transitioning to Lost state");
            
            is_transition_needed_ = true;
            next_phase_type_ =  PhaseType::Lost;
            return;
        }

        float x_pos = get_target.response.position.x;
        float y_pos = get_target.response.position.y;
        float z_pos = 0;

        // If marker is within a threshold then we reduce our height
        if (abs(get_target.response.position.x) < marker_threshold_ &&
            abs(get_target.response.position.y) < marker_threshold_)
        {
            float estimated_height = get_target.response.position.z;
            float height_change = -(estimated_height - delivery_height_);

            // Clamp so we dont change height too quickly
            z_pos = std::min(std::max(height_change, -height_change_step_), height_change_step_);

            // If height is also within the threshold then transition to delivery
            // TODO: Is this robust enough? We should reach this condition several times before we
            // can say we are stable enough to deliver
            if (abs(z_pos) < marker_threshold_)
            {
                ROS_INFO("Marker within threshold. Transitioning to Deliver");
                is_transition_needed_ = true;
                next_phase_type_ =  PhaseType::Deliver;
            }
        }
        
        sendThrottledMoveCommand(x_pos, y_pos, z_pos);
    }
    else
    {
        ROS_ERROR("Call to GetTarget service failed");
    }
}