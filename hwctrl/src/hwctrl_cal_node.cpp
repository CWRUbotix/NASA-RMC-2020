#include "pch.h"

#include "hwctrl.h"

#include "threads/sensor_cal_thread.h"


int main(int argc, char** argv){
	ros::init(argc, argv, "hwctrl_cal");
    ROS_INFO("Hardware Controller Calibration Node");
	ros::NodeHandle n;

    SensorCalThread sensor_cal_thread(n);

	ROS_INFO("Node init success");

    std::thread sensor_cal_thread_obj(std::ref(sensor_cal_thread));

    sensor_cal_thread_obj.detach();

	ros::waitForShutdown();
	return 0;
}