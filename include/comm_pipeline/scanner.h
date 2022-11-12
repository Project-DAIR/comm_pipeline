#ifndef SCANNER_H_
#define SCANNER_H_

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Point.h>

class Scanner
{
public:
    Scanner();
    void calcSquareWPs(void);
    geometry_msgs::Point nextWP();
    bool isFinished();
    float scan_height_;

private:
    float side_;
    bool is_finished_;

    float arrival_heading_;

    std::vector<geometry_msgs::Point> square_WPs_;
    std::vector<geometry_msgs::Point>::iterator next_wp_it_;
};

#endif