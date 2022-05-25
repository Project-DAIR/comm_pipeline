#ifndef TIMER_H_
#define TIMER_H_

#include <ros/ros.h>

class Timer
{
public:
    Timer() : is_started_(false), duration_(0)
    {
    }

    void setDuration(float duration) {
        duration_ = ros::Duration(duration);
    }

    void start()
    {
        if (!is_started_)
        {
            is_started_ = true;
            start_time_ = ros::Time::now();
        }
    }

    void stop()
    {
        is_started_ = false;
    }

    void restart()
    {
        stop();
        start();
    }

    bool isFinished()
    {
        if (!is_started_)
        {
            return true;
        }

        if (ros::Time::now() - start_time_ > duration_)
        {
            return true;
        }

        return false;
    }

private:
    bool is_started_;
    ros::Duration duration_;
    ros::Time start_time_;
};

#endif