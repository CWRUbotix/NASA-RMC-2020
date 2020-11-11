#pragma once

#include <ros/ros.h>

class MotorThread {
public:
    MotorThread(ros::NodeHandle nh) : m_nh(nh) {}

    void operator()() {}

private:
    ros::NodeHandle m_nh;
};