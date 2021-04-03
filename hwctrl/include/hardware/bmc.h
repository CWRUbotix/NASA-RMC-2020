#pragma once

#include "motor.h"

#include <ros/ros.h>

class BMCMotor : public CanMotor {
 public:
  BMCMotor(ros::NodeHandle nh, MotorConfig const& config);
  virtual ~BMCMotor() = default;

  virtual void setup() override final;
  virtual void update(ros::Time time) override final;
  virtual void stop() override final;

  virtual void can_rx_callback(FramePtr frame) override final;

 private:
};
