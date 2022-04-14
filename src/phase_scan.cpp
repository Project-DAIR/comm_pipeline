#include "comm_pipeline/phase.h"
#include "comm_pipeline/scanner.h"

#include <geometry_msgs/Point.h>

// PhaseScan::PhaseScan(){
    // Scanner scan;
// }

void PhaseScan::_enter() {
    scan_generator.calcSquareWPs();
    return;
}

void PhaseScan::_exit() {
    return;
}

void PhaseScan::handler() {
    geometry_msgs::Point next_wp, last_wp;
    next_wp = scan_generator.nextWP();
    if(next_wp == last_wp){
        return;
    }
    sendThrottledMoveCommand(next_wp.x, next_wp.y, next_wp.z);
    last_wp = next_wp;

    return;
}