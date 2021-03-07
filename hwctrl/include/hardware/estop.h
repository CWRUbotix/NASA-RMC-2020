#pragma once

#include <ros/time.h>

#include <std_msgs/Bool.h>

#include <vector>

#include "sensor.h"

// This is more or less a duplicate of generic gpio sensor
// I just dont think we need it to send sensor_data messages.
// boolean will be fine. Plus we might want to extend this class later
class EStop : public GpioSensor<std_msgs::Bool> {
public:
  EStop(ros::NodeHandle nh, std::unique_ptr<Gpio> gpio, SensorConfig const& config);
  ~EStop() = default;

  virtual void setup() override final { m_is_setup = true; }
  virtual void update() override final;
  virtual void calibrate(std::vector<Calibration>& cals) override final {}
private:
  bool m_last_state = true;
};

