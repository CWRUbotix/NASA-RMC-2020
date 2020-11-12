#include "threads/sensor_thread.h"

#include <ros/spinner.h>


SensorThread::SensorThread(ros::NodeHandle nh) : m_loop_rate(100) {

}

void SensorThread::sleep() {
    m_loop_rate.sleep();
}

void SensorThread::shutdown() {

}

void SensorThread::setup_sensors() {
    for(Sensor sensor : m_sensors) {
        sensor.setup();
    }
}

// run thread
void SensorThread::operator()() {
    ROS_INFO("Starting sensor_thread");
    ros::AsyncSpinner spinner(1, NULL);

    setup_sensors();

    while(ros::ok()) {
        sleep();   
    }
    shutdown();
}