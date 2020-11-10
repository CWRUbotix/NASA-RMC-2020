#include <ros/ros.h>

#include <thread>

#include "threads/sensor_thread.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hwctrl2");
    ros::NodeHandle nh;

    SensorThread st(nh);


    std::thread sensor_thread_obj(std::ref(st));

    ros::waitForShutdown();
    return 0;
}