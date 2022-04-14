#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>

#include "comm_pipeline/scanner.h"

#define PI 3.14159265

Scanner::Scanner() : square_WPs_(5, geometry_msgs::Point()), is_finished_(false)
{
    ros::NodeHandle param_nh("~");
    param_nh.param("scan_diagonal", scan_diagonal_, 2.0f);

    internal_angle_ = 45.0 * PI / 180.0;

    scan_diagonal_ = 2;
    side_ = 2 * cos(internal_angle_) * scan_diagonal_;
}

void Scanner::calcSquareWPs()
{

    square_WPs_[0].x = -scan_diagonal_ * cos(internal_angle_);
    square_WPs_[0].y = -scan_diagonal_ * sin(internal_angle_);
    square_WPs_[0].z = 0;
    // square_WPs_[0] = sq_pose_;

    square_WPs_[1].x = side_;
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

    next_wp_it_ = square_WPs_.begin();

    for (auto a : square_WPs_)
    {
        ROS_INFO("%f, %f, %f", a.x, a.y, a.z);
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
