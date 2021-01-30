#include "hardware/bmc.h"

BMCMotor::BMCMotor(ros::NodeHandle nh, const std::string& name, uint32_t id,
                   uint32_t can_id, ros::Duration update_pd,
                   float accel_setpoint, float max_accel, float max_rpm,
                   float gear_reduc, ros::Duration timeout)
    : CanMotor(nh, name, id, can_id, MotorType::BMC, ControlType::Position,
               update_pd, accel_setpoint, max_accel, max_rpm, gear_reduc,
               timeout) {}

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
