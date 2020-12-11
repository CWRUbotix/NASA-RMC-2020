#include "hardware/motor.h"

#include <cmath>

MotorType get_motor_type(boost::string_view type_str) {
  if (type_str.compare("vesc") == 0) {
    return MotorType::Vesc;
  } else if (type_str.compare("bmc") == 0) {
    return MotorType::BMC;
  } else if (type_str.compare("sbrth") == 0) {
    return MotorType::Sabertooth;
  }
  return MotorType::None;
}

Motor::Motor(ros::NodeHandle nh, const std::string& name, uint32_t id,
             MotorType m_type, ControlType c_type, ros::Duration update_pd,
             float accel_setpoint, float max_accel, float max_rpm,
             float gear_reduc, ros::Duration timeout)
    : m_nh(nh),
      m_name(name),
      m_id(id),
      m_type(m_type),
      m_control_type(c_type),
      m_update_pd(update_pd),
      m_timeout(timeout),
      m_accel_setpoint(accel_setpoint),
      m_max_rpm(max_rpm),
      m_max_accel(max_accel),
      m_rpm_coef(gear_reduc) {
  m_motor_data_pub = m_nh.advertise<hwctrl::MotorData>("motor_data", 128);
  m_update_timer = m_nh.createTimer(m_update_pd, &Motor::set_update_flag, this);
}

void Motor::set_setpoint(ros::Time time, float setpoint, float acceleration) {
  m_setpoint = setpoint;
  m_accel_setpoint = acceleration;
  m_last_set_time = time;
}

CanMotor::CanMotor(ros::NodeHandle nh, const std::string& name, uint32_t id,
                   uint32_t can_id, MotorType m_type, ControlType c_type,
                   ros::Duration update_pd, float accel_setpoint,
                   float max_accel, float max_rpm, float gear_reduc,
                   ros::Duration timeout)
    : Motor(nh, name, id, m_type, c_type, update_pd, accel_setpoint, max_accel,
            max_rpm, gear_reduc, timeout),
      m_can_id(can_id) {
  m_can_tx_pub = m_nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
  m_can_rx_sub =
      m_nh.subscribe("can_frames_rx", 128, &CanMotor::can_rx_callback, this);
}
