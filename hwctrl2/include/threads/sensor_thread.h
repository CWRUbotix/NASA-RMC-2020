#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <vector>

#include "hardware/sensor.h"
#include "hardware/quad_encoder.h"
#include "hardware/uwb.h"

class SensorThread {
public:
    SensorThread(ros::NodeHandle nh);
    ~SensorThread() = default;

    void setup_sensors();

    void shutdown();
    void sleep();

    void operator()();
private:
    ros::NodeHandle    m_nh;
    ros::Rate          m_loop_rate;
    ros::CallbackQueue m_cb_queue;

    std::vector<SensorImpl>  m_sensors;
    std::vector<UwbNode> m_uwb_nodes;
    uint32_t             m_uwb_ind;
};
