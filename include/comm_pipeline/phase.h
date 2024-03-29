#ifndef PHASE_H_
#define PHASE_H_

#include <string>
#include <ros/ros.h>
#include <comm_pipeline/GetTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include "comm_pipeline/scanner.h"
#include "comm_pipeline/timer.h"

enum class PhaseType
{
    Scan,
    Detected,
    VisualServo,
    Deliver,
    Lost,
    Ended,
    Invalid // Only used for default base phase
};

class Phase
{
public:
    Phase();

    void enter();
    void exit();

    virtual void _enter(){};
    virtual void _exit(){};
    virtual void handler(){};

    virtual std::string getName() {return "InvalidPhase"; };
    virtual PhaseType getPhaseType() {return PhaseType::Invalid; };

    void sendMoveCommand(float x, float y, float z);
    bool sendThrottledMoveCommand(float x, float y, float z);

    bool isTransitionNeeded() { return is_transition_needed_; };
    virtual PhaseType getNextPhaseType() { return next_phase_type_; };

protected:
    bool is_transition_needed_;
    PhaseType next_phase_type_;
    float cmd_time_interval_;

    Timer move_timer_;

    ros::Publisher local_pos_pub_;
    ros::ServiceClient get_target_client_;
};

class PhaseScan : public Phase
{
public:
    std::string getName() { return "Scan"; };
    PhaseType getPhaseType() { return PhaseType::Scan; };
    void handler() override;

private:
    void _enter() override;
    void _exit() override;
    Scanner scan_generator_;
    bool move_accepted_;
    geometry_msgs::Point prev_wp_;
};

class PhaseDetected : public Phase
{
public:
    std::string getName() { return "Detected"; };
    PhaseType getPhaseType() { return PhaseType::Detected; };
    void handler() override;

    float detected_pos_x_;
    float detected_pos_y_;
    float detected_pos_z_;
private:
    void _enter() override;
    void _exit() override;

};

class PhaseVisualServo : public Phase
{
public:
    PhaseVisualServo();

    std::string getName() { return "VisualServo"; };
    PhaseType getPhaseType() { return PhaseType::VisualServo; };
    void handler() override;

    void markerCallback(const geometry_msgs::Point::ConstPtr& msg);
    void lidarCallback(const std_msgs::Int16::ConstPtr& msg);

private:
    ros::Subscriber marker_pos_sub_;
    ros::Subscriber lidar_sub_;

    // Params
    float marker_threshold_;
    float marker_timeout_;
    float delivery_height_;
    float height_change_step_;
    float required_time_for_stability_;

    bool is_running_;
    bool prev_stable_;

    float lidar_height_cm_;

    ros::Time marker_last_seen_time_;
    ros::Time stable_start_time_;

    void _enter() override;
    void _exit() override;
};

class PhaseDeliver : public Phase
{
public:
    PhaseDeliver();

    std::string getName() { return "Deliver"; };
    PhaseType getPhaseType() { return PhaseType::Deliver; };
    void handler() override;

    void runDeliverySubsystem();
    void refinePosition();

    void endDeliveryCallback(const std_msgs::Bool::ConstPtr& msg);

private:
    ros::Publisher start_delivery_pub_;
    ros::Subscriber end_delivery_sub_;

    float marker_threshold_;
    float delivery_height_;
    bool in_position_;
    bool delivery_finished_;
    bool delivery_started_;

    Timer delivery_timer_;

    void _enter() override;
    void _exit() override;
};

class PhaseLost : public Phase
{
public:
    PhaseLost();

    std::string getName() { return "Lost"; };
    PhaseType getPhaseType() { return PhaseType::Lost; };
    void handler() override;

private:
    void _enter() override;
    void _exit() override;

    void moveToLastKnownLocation();
    void moveToHigherAltitude();
    void moveToStartOfScan();
    void foundMarker();
    
    enum class Behavior {
        LastKnownLocation,
        AltitudeIncrease,
        RestartScan
    };

    Behavior current_behaviour_;
    float altitude_increase_;

    Timer behaviour_timer_;
    
    bool moved_to_last_known_;
    bool moved_to_altitude_;
};

class PhaseEnded : public Phase
{
public:
    PhaseEnded();
    std::string getName() { return "Ended"; };
    PhaseType getPhaseType() { return PhaseType::Ended; };
    void handler() override;

private:
    void _enter() override;
    void _exit() override;

    ros::ServiceClient set_mode_client_;
};

#endif
