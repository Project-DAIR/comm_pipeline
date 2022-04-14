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
    void setDetectedPosition(float x, float y, float z);
    void initialize();

private:
    // Phases
    PhaseScan scan_phase_;
    PhaseDetected detected_phase_;
    PhaseVisualServo visual_servo_phase_;
    PhaseDeliver deliver_phase_;
    PhaseLost lost_phase_;
    PhaseEnded ended_phase_;

    Phase *current_phase_;
};

#endif