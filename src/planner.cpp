#include "comm_pipeline/planner.h"

Planner::Planner(ros::NodeHandle *nodehandle) : nh_(*nodehandle),
                                                changed_to_guided_(false),
                                                delivery_waypoint_reached_(false)
{
  initializeSubscribers();
  initializeServices();

  ros::NodeHandle param_nh("~");

  param_nh.param("delivery_waypoint_number", delivery_waypoint_number_, 2);
  param_nh.param("wp_threshold", wp_threshold_, 0.25f);
}

void Planner::initializeSubscribers()
{
  state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Planner::stateCallback, this);
  nav_output_sub_ = nh_.subscribe<mavros_msgs::NavControllerOutput>("mavros/nav_controller_output", 10, &Planner::navOutputCallback, this);
  mission_sub_ = nh_.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 10, &Planner::missionCallback, this);
}

void Planner::initializeServices()
{
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  activate_stag_client_ = nh_.serviceClient<comm_pipeline::ActivateStag>("marker/activate_stag");

  ros::service::waitForService("mavros/set_mode");
  ros::service::waitForService("marker/activate_stag");
  ROS_DEBUG("All service clients connceted succesfully");

  found_marker_server_ = nh_.advertiseService("planner/found_marker",
                                              &Planner::foundMarkerCallback, this);
}

void Planner::activateStag()
{
  comm_pipeline::ActivateStag srv;

  if (activate_stag_client_.call(srv))
  {
    if (srv.response.activated)
    {
      ROS_INFO("Marker detector activated succesfully");

      // TODO: Run scan pattern
    }
    else
    {
      ROS_WARN("Marker detector did not activate");
    }
  }
  else
  {
    ROS_ERROR("Call to ActivateStag service failed");
  }
}

bool Planner::foundMarkerCallback(comm_pipeline::FoundMarker::Request &req, comm_pipeline::FoundMarker::Response &res)
{
  ROS_INFO("Found marker service called");

  if (phase_manager_.getCurrentPhaseType() == PhaseType::Scan)
  {
    phase_manager_.setDetectedPosition(req.position.point.x, req.position.point.y, req.position.point.z);
    phase_manager_.changePhase(PhaseType::Detected);
    res.success = true;
  }
  else
  {
    ROS_WARN("Found Marker called whilst not in Scan");
    res.success = false;
  }

  return true;
}

void Planner::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;

  if (!current_state_.connected)
  {
    return;
  }

  /*
  Change to guided only when:
    1. Not previously changed into GUIDED
    2. Delivery Waypoint is reached
    3. Not already in GUIDED
  */
  if (!changed_to_guided_ && delivery_waypoint_reached_ && current_state_.mode != "GUIDED")
  {
    mavros_msgs::SetMode mode_change;
    mode_change.request.custom_mode = "GUIDED";

    if (set_mode_client_.call(mode_change))
    {

      // Make sure GUIDED mode was sent to the FCU
      if (!mode_change.response.mode_sent)
      {
        ROS_ERROR("GUIDED mode was not set by MAVROS");
        return;
      }

      ROS_INFO("Guided mode enabled...");
      changed_to_guided_ = true;

      // Send STag Activation
      activateStag();
    }
    else
    {
      ROS_ERROR("Call to SetMode service failed");
    }
  }
}

void Planner::navOutputCallback(const mavros_msgs::NavControllerOutput::ConstPtr &msg)
{
  nav_output_ = *msg;

  if (current_state_.mode != "GUIDED")
  {
    return;
  }

  // nav output returns in cm so convert to metres
  float xy_distance_to_wp = nav_output_.wp_dist / 100.0f;
  float total_dist_to_wp = pow(xy_distance_to_wp, 2) + pow(nav_output_.alt_error, 2);

  // If we havent arrived at the waypoint then return
  if (total_dist_to_wp > pow(wp_threshold_, 2))
  {
    return;
  }

  phase_manager_.runCurrentPhase();
}

void Planner::missionCallback(const mavros_msgs::WaypointReached::ConstPtr &msg)
{
  if (msg->wp_seq >= delivery_waypoint_number_)
  {
    delivery_waypoint_reached_ = true;
    ROS_INFO("Delivery Waypoint Reached");
  }
}