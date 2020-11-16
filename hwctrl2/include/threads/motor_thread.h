#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <hwctrl2/SetMotorMsg.h>
#include <hwctrl2/SensorData.h>
#include <hwctrl2/LimitSwState.h>
#include <hwctrl2/MotorData.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/move/unique_ptr.hpp>
#include <boost/move/make_unique.hpp>

#include <vector>

#include "hardware/motor.h"

class MotorThread {
public:
    MotorThread(ros::NodeHandle nh);
    ~MotorThread() = default;

    void read_from_server();

    void setup_motors();
    void update_motors();

    void sleep();
    void shutdown();

    void operator()();

private: 
    void set_motor_callback(boost::shared_ptr<hwctrl2::SetMotorMsg> msg);
    void limit_switch_callback(boost::shared_ptr<hwctrl2::LimitSwState> msg);
    void sensor_data_callback(boost::shared_ptr<hwctrl2::SensorData> msg);

private:
    // general ros objects
    ros::NodeHandle    m_nh;
    ros::Rate          m_loop_rate;
    ros::CallbackQueue m_cb_queue;

    // subscribers
    ros::Subscriber  m_motor_set_sub;
    ros::Subscriber  m_limit_sw_sub;
    ros::Subscriber  m_sensor_data_sub;

    // publishers
    ros::Publisher   m_motor_data_pub;

    // motors
    using MotorVec = std::vector<boost::shared_ptr<Motor>>;
    using MotorIter = MotorVec::iterator;

    MotorVec  m_motors;
    MotorIter m_motor_iter;
};