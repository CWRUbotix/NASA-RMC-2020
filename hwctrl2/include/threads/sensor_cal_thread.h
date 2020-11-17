#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/utility/string_view.hpp>

#include "threads/sensor_thread.h"

class SensorCalThread : SensorThread {
public:
    SensorCalThread(ros::NodeHandle);
    virtual ~SensorCalThread() = default;

    virtual void operator()() override final;
private:
    void calibrate_sensor_with_name(boost::string_view name, std::vector<Calibration>* cals);
};