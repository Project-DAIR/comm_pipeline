#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include "comm_pipeline/scanner.h"


Scanner::Scanner() : square_WPs_(6, geometry_msgs::Point()), is_finished_(false)
{
    ros::NodeHandle param_nh("~");
    param_nh.param("square_side", side_, 2.0f);
    param_nh.param("scan_height", scan_height_, 2.0f);
}

void Scanner::calcSquareWPs()
{
    geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("mavros/local_position/pose");
    std_msgs::Float64::ConstPtr heading = ros::topic::waitForMessage<std_msgs::Float64>("mavros/global_position/compass_hdg");

    arrival_heading_ = heading->data;

    square_WPs_[0].x = 0;
    square_WPs_[0].y = -side_/2;;
    square_WPs_[0].z = -(msg->pose.position.z - scan_height_);

    square_WPs_[1].x = side_/2;;
    square_WPs_[1].y = 0;
    square_WPs_[1].z = 0;
    
    square_WPs_[2].x = 0;
    square_WPs_[2].y = side_;
    square_WPs_[2].z = 0;

    square_WPs_[3].x = -side_;
    square_WPs_[3].y = 0;
    square_WPs_[3].z = 0;

    square_WPs_[4].x = 0;
    square_WPs_[4].y = -side_;
    square_WPs_[4].z = 0;

    square_WPs_[5].x = side_/2;
    square_WPs_[5].y = 0;
    square_WPs_[5].z = 0;

    next_wp_it_ = square_WPs_.begin();

    for (auto a : square_WPs_)
    {
        ROS_INFO("Square coordinates: %f, %f, %f", a.x, a.y, a.z);
    }
}

geometry_msgs::Point Scanner::nextWP()
{
    geometry_msgs::Point next_wp;

    if (next_wp_it_ != square_WPs_.end())
    {
        next_wp = *next_wp_it_;
        next_wp_it_++;

        float cur_heading = ros::topic::waitForMessage<std_msgs::Float64>("mavros/global_position/compass_hdg")->data;
        float heading_diff = cur_heading - arrival_heading_;

        float rot_x = next_wp.x * cos(M_PI / 180 * heading_diff) - next_wp.y * sin(M_PI / 180 * heading_diff);
        float rot_y = next_wp.x * sin(M_PI / 180 * heading_diff) + next_wp.y * cos(M_PI / 180 * heading_diff);

        next_wp.x = rot_x;
        next_wp.y = rot_y;
    }
    else
    {
        is_finished_ = true;
    }

    return next_wp;
}

bool Scanner::isFinished()
{
    return is_finished_;
}
