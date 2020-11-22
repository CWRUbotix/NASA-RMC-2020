#pragma once

#include "threads/sensor_thread.h"

#include <ros/spinner.h>

#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include "hwctrl.h"
#include "hardware/sensor.h"

class SensorCalThread : SensorThread {
public:
    SensorCalThread(ros::NodeHandle);
    virtual ~SensorCalThread() = default;

    virtual void operator()() override final;
private:
    void calibrate_sensor_with_name(boost::string_view name, std::vector<Calibration>& cals);
};