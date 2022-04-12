#ifndef PHASE_MANAGER_H_
#define PHASE_MANAGER_H_

#include <map>

#include "phase.h"

class PhaseManager
{

public:
    PhaseManager();

    PhaseType getCurrentPhaseType();
    void runCurrentPhase();
    void changePhase(PhaseType next_phase);

private:
    // Phases
    PhaseScan scan_phase_;
    PhaseDetected detected_phase_;
    PhaseVisualServo visual_servo_phase_;
    PhaseDeliver deliver_phase_;
    PhaseLost lost_phase_;
    PhaseEnded ended_phase_;

    Phase *current_phase_ = &scan_phase_;
};

#endif