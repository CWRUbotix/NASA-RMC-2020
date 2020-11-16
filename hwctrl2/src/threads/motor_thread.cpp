#include "threads/motor_thread.h"

#include <ros/spinner.h>

#include <string>

#include "hwctrl.h"

MotorThread::MotorThread(ros::NodeHandle nh) : m_nh(nh), m_loop_rate(1000) {
    m_motors.reserve(16);
    // m_cb_queue = ros::CallbackQueue();

    m_nh.setCallbackQueue(&m_cb_queue);

    m_motor_data_pub = m_nh.advertise<hwctrl2::MotorData>("motor_data", 128);

    m_motor_set_sub   = m_nh.subscribe("motor_setpoints", 128, &MotorThread::set_motor_callback, this);
    m_sensor_data_sub = m_nh.subscribe("sensor_data", 128, &MotorThread::sensor_data_callback, this);
    // m_limit_sw_sub  = m_nh.subscribe("");

    read_from_server();
}

void MotorThread::read_from_server() {

}

void MotorThread::set_motor_callback(boost::shared_ptr<hwctrl2::SetMotorMsg> msg) {

}

void MotorThread::sensor_data_callback(boost::shared_ptr<hwctrl2::SensorData> msg) {

}

void MotorThread::setup_motors() {
    for(auto motor : m_motors) {
        motor->setup();
    }
}

void MotorThread::update_motors() {
    auto time = ros::Time::now();
    for(auto motor : m_motors) {
        if(motor->ready_to_update()) {
            motor->update(time);
            // m_motor_data_pub.publish(data);
        }
    }
}

void MotorThread::shutdown() {
    // stop motors maybe?
}

void MotorThread::sleep() {
    m_loop_rate.sleep();
}

void MotorThread::operator()() {
    setup_motors();
    while(ros::ok()) {
        update_motors();
        sleep();
    }
    shutdown();
}