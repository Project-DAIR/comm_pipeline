#include "comm_pipeline/phase.h"
#include "comm_pipeline/scanner.h"

#include <geometry_msgs/Point.h>

void PhaseScan::_enter() {
    scan_generator.calcSquareWPs();
    return;
}

void PhaseScan::_exit() {
    return;
}

void PhaseScan::handler() {

    // If scanning has finished transition to ended state
    if (scan_generator.isFinished()) {
        ROS_INFO("Scan path is finished");
        is_transition_needed_ = true;
        next_phase_type_ =  PhaseType::Ended;
        return;
    }

    geometry_msgs::Point next_wp = scan_generator.nextWP();
    ROS_INFO("Moving to scanner wp = (%f, %f, %f)", next_wp.x, next_wp.y, next_wp.z);
    sendMoveCommand(next_wp.x, next_wp.y, next_wp.z);

    return;
}