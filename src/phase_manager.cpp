#include "comm_pipeline/phase_manager.h"

PhaseManager::PhaseManager()
{
    ros::service::waitForService("marker/get_target");
    ROS_DEBUG("GetTarget service is reachable");
}

PhaseType PhaseManager::getCurrentPhaseType()
{
    return current_phase_->getPhaseType();
}

void PhaseManager::runCurrentPhase()
{
    current_phase_->handler();

    if (current_phase_->isTransitionNeeded())
    {
        changePhase(current_phase_->getNextPhaseType());
    }
}

void PhaseManager::changePhase(PhaseType next_phase_type)
{
    // Dont transition if we are trying to go to same phase
    if (next_phase_type == current_phase_->getPhaseType())
    {
        ROS_WARN("Attempted to transition into same phase (%s)", current_phase_->getName().c_str());
        return;
    }

    Phase *next_phase = nullptr;

    switch (next_phase_type)
    {
    case PhaseType::Scan:
        next_phase = &scan_phase_;
        break;

    case PhaseType::Detected:
        next_phase = &detected_phase_;
        break;

    case PhaseType::VisualServo:
        next_phase = &visual_servo_phase_;
        break;

    case PhaseType::Deliver:
        next_phase = &deliver_phase_;
        break;

    case PhaseType::Lost:
        next_phase = &lost_phase_;
        break;

    case PhaseType::Ended:
        next_phase = &ended_phase_;
        break;

    case PhaseType::Invalid:
        ROS_ERROR("Invalid State. This should not happen");
        return;
    }

    current_phase_->exit();
    next_phase->enter();

    current_phase_ = next_phase;
}

void PhaseManager::setDetectedPosition(float x, float y, float z) {
    detected_phase_.detected_pos_x_ = x;
    detected_phase_.detected_pos_y_ = y;
    detected_phase_.detected_pos_z_ = z;
}

void PhaseManager::initialize() {
    current_phase_ = &scan_phase_;
    current_phase_->enter();
}