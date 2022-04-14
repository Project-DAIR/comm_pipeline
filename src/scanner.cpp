#include <ros/ros.h>
#include <math.h> 
#include <vector>
#include <geometry_msgs/Point.h>

#include "comm_pipeline/scanner.h"

#define PI 3.14159265

Scanner::Scanner() : square_WPs_(4) {
    internal_angle_ = 45.0;

    setDiag();
    calcSide();
} 

void Scanner::calcSquareWPs(){
    square_WPs_[0].x = diag_*cos(internal_angle_ * PI/180.0);
    square_WPs_[0].y = diag_*sin(internal_angle_ * PI/180.0);
    square_WPs_[0].z = 0;
    // square_WPs_[0] = sq_pose_;

    square_WPs_[1].x = side_;
    square_WPs_[1].x = 0;
    square_WPs_[1].x = 0;

    square_WPs_[2].x = 0;
    square_WPs_[2].y = side_;
    square_WPs_[2].z = 0;

    square_WPs_[2].x = -side_;
    square_WPs_[2].y = 0;
    square_WPs_[2].z = 0;

    next_wp_it_ = square_WPs_.begin();

}
geometry_msgs::Point Scanner::nextWP(){
    if(next_wp_it_ != square_WPs_.end()){
        next_wp_ = *next_wp_it_;
        next_wp_it_++;
    }
    return next_wp_;
}

void Scanner::setDiag(){
    diag_ = 2;
}
void Scanner::calcSide(){
    side_ = 2 * cos(internal_angle_ * PI/180.0) * diag_;
}
