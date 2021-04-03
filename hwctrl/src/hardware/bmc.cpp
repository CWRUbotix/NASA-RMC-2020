#include "hardware/bmc.h"

BMCMotor::BMCMotor(ros::NodeHandle nh, MotorConfig const& config)
    : CanMotor(nh, config) {}

void BMCMotor::setup() { m_is_setup = true; }

void BMCMotor::update(ros::Time time) {
  if (!m_online) {
    return;
  }

  m_update = false;
}

void BMCMotor::stop() {
  // TODO: Make this actually work
  set_setpoint(ros::Time::now(), 0.0);
}

void BMCMotor::can_rx_callback(FramePtr frame) {}
