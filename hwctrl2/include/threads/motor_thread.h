#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <string>
#include <vector>

class MotorThread {
public:
    MotorThread(ros::NodeHandle nh) : m_nh(nh), m_loop_rate(1000) {}

    void operator()() {}

private:
    ros::NodeHandle m_nh;
    ros::Rate       m_loop_rate;
    ros::CallbackQueue m_cb_queue;
};