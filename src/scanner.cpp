#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

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

    square_WPs_[0].x = 0;
    square_WPs_[0].y = 0;
    square_WPs_[0].z = -(msg->pose.position.z - scan_height_);

    square_WPs_[1].x = 0;
    square_WPs_[1].y = -side_/2;
    square_WPs_[1].z = 0;
    
    square_WPs_[2].x = side_/2;
    square_WPs_[2].y = 0;
    square_WPs_[2].z = 0;

    square_WPs_[3].x = 0;
    square_WPs_[3].y = side_;
    square_WPs_[3].z = 0;

    square_WPs_[4].x = -side_;
    square_WPs_[4].y = 0;
    square_WPs_[4].z = 0;

    square_WPs_[5].x = 0;
    square_WPs_[5].y = -side_;
    square_WPs_[5].z = 0;

    // square_WPs_[5].x = side_;
    // square_WPs_[5].y = 0;
    // square_WPs_[5].z = 0;

    next_wp_it_ = square_WPs_.begin();

    for (auto a : square_WPs_)
    {
        ROS_INFO("Square coordinates: %f, %f, %f", a.x, a.y, a.z);
    }
}

geometry_msgs::Point Scanner::nextWP()
{
    if (next_wp_it_ != square_WPs_.end())
    {
        next_wp_ = *next_wp_it_;
        next_wp_it_++;
    }
    else
    {
        is_finished_ = true;
    }

    return next_wp_;
}

bool Scanner::isFinished()
{
    return is_finished_;
}
