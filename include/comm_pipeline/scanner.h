#ifndef SCANNER_H_
#define SCANNER_H_

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Point.h>


class Scanner{
    public:
        Scanner();
        void calcSquareWPs(void);
        geometry_msgs::Point nextWP();

    private:
        
        float diag_;
        float side_;
        float internal_angle_;
        
        geometry_msgs::Point next_wp_;
        // std::vector<geometry_msgs::Point> square_WPs_(4, geometry_msgs::Point());
        std::vector<geometry_msgs::Point> square_WPs_;
        std::vector<geometry_msgs::Point>::iterator next_wp_it_;

        void setDiag(void);
        void calcSide(void);

};

#endif