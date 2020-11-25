#include "hwctrl.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include <thread>
#include <utility>
#include <string>
#include <iostream>

#include "threads/canbus_thread.h"
#include "threads/sensor_cal_thread.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "hwctrl_cal");
    ROS_INFO("Hardware Controller Calibration Node");
	ros::NodeHandle n;

	CanbusThread    canbus_thread(n);
    SensorCalThread sensor_cal_thread(n);

	ROS_INFO("Node init success");

	std::thread canbus_thread_obj(std::ref(canbus_thread));
    std::thread sensor_cal_thread_obj(std::ref(sensor_cal_thread));

	canbus_thread_obj.detach();
    sensor_cal_thread_obj.detach();

	ros::waitForShutdown();
	return 0;
}