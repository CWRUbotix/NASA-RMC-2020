#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <vector>

#include "hardware/sensor.h"

class SensorThread {
public:
    SensorThread(ros::NodeHandle nh);
    ~SensorThread() = default;

    void setup_sensors();
    void update_sensors();

    void shutdown();
    void sleep();

    void operator()();
private:
    ros::NodeHandle    m_nh;
    ros::Rate          m_loop_rate;
    ros::CallbackQueue m_cb_queue;

    std::vector<Sensor> m_sensors;
};

void sensor_thread(SensorThread* sensor_thread);