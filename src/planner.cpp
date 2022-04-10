#include "planner.h"

Planner::Planner(ros::NodeHandle *nodehandle) : nh_(*nodehandle),
                                                is_moving_(false),
                                                changed_to_guided_(false),
                                                delivery_waypoint_reached_(false)
{
  initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
  initializePublishers();
  initializeServices();

  ros::NodeHandle param_nh("~");

  param_nh.param("delivery_waypoint_number", delivery_waypoint_number_, 2);
}

void Planner::initializeSubscribers()
{
  state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Planner::stateCallback, this);
  nav_output_sub_ = nh_.subscribe<mavros_msgs::NavControllerOutput>("mavros/nav_controller_output", 10, &Planner::navOutputCallback, this);
  mission_sub_ = nh_.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 10, &Planner::missionCallback, this);
}

void Planner::initializePublishers()
{
  local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
}

void Planner::initializeServices()
{
  // Second argument set to true means persistent connection to service (more efficient)
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode", true);
  activate_stag_client_ = nh_.serviceClient<comm_pipeline::ActivateStag>("marker/activate_stag", true);
  get_target_client_ = nh_.serviceClient<comm_pipeline::GetTarget>("marker/get_target", true);

  ros::service::waitForService("mavros/set_mode");
  ros::service::waitForService("marker/activate_stag");
  ros::service::waitForService("marker/get_target");
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
}

void moveByOffset(float x, float y, float z) {
  geometry_msgs::PoseStamped msg;

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;

  local_pos_pub.publish(msg);
}

bool Planner::foundMarkerCallback(comm_pipeline::FoundMarker::Request &req, comm_pipeline::FoundMarker::Response &res)
{
  // TODO
  if (delivery_state_ == DeliveryState::Scan) {
    delivery_state_ = DeliveryState::Detected;

    moveByOffset(res.position.point.x, res.position.point.y, res.position.point.z);
  }

  return false;
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
  }
}

void Planner::navOutputCallback(const mavros_msgs::NavControllerOutput::ConstPtr &msg)
{
  nav_output_ = *msg;
}

void Planner::markerCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  marker_pose_ = *msg;
}

void Planner::missionCallback(const mavros_msgs::WaypointReached::ConstPtr &msg)
{
  if (msg->wp_seq >= delivery_waypoint_number_)
  {
    delivery_waypoint_reached_ = true;
    ROS_INFO("Delivery Waypoint Reached");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  Planner planner(&nh);

  ROS_INFO("Communication Initialized... ");
  ros::spin();

  return 0;
}
