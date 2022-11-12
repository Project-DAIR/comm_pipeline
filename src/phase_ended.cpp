#include "comm_pipeline/phase.h"
#include "mavros_msgs/SetMode.h"

PhaseEnded::PhaseEnded() {
    ros::NodeHandle nh;
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void PhaseEnded::_enter() {
    mavros_msgs::SetMode mode_change;
    mode_change.request.custom_mode = "AUTO";

    if (set_mode_client_.call(mode_change))
    {
      // Make sure mode change was sent to the FCU
      if (!mode_change.response.mode_sent)
      {
        ROS_ERROR("AUTO mode was not set by MAVROS");
        return;
      }
    }
    else
    {
      ROS_ERROR("Call to SetMode service failed");
    }
    return;
}

void PhaseEnded::_exit() {
    return;
}

void PhaseEnded::handler() {
    return;
}