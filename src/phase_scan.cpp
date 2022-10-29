#include "comm_pipeline/phase.h"
#include "comm_pipeline/scanner.h"

#include <geometry_msgs/Point.h>

void PhaseScan::_enter()
{
    move_accepted_ = true;
    scan_generator_.calcSquareWPs();
    return;
}

void PhaseScan::_exit()
{
    return;
}

void PhaseScan::handler()
{
    if (!move_accepted_) {
        ROS_INFO("Resending previous scan wp = (%f, %f, %f)", prev_wp_.x, prev_wp_.y, prev_wp_.z);
        move_accepted_ = sendThrottledMoveCommand(prev_wp_.x, prev_wp_.y, prev_wp_.z);
        return;
    }

    geometry_msgs::Point next_wp = scan_generator_.nextWP();

    // If scanning has finished transition to ended state
    if (scan_generator_.isFinished())
    {
        ROS_INFO("Scan path is finished");
        is_transition_needed_ = true;
        next_phase_type_ = PhaseType::Ended;
        return;
    }

    ROS_INFO("Moving to scanner wp = (%f, %f, %f)", next_wp.x, next_wp.y, next_wp.z);
    move_accepted_ = sendThrottledMoveCommand(next_wp.x, next_wp.y, next_wp.z);

    prev_wp_ = next_wp;

    return;
}