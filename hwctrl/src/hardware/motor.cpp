#include "hardware/motor.h"

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

Motor::Motor(ros::NodeHandle nh,MotorConfig const& config)
    : m_nh(nh), m_config(config)
{
  m_motor_data_pub = m_nh.advertise<hwctrl::MotorData>("motor_data", 128);
  m_motor_cmd_sub = m_nh.subscribe(m_config.cmd_topic, 64, &Motor::setpoint_callback, this);
  m_update_timer = m_nh.createTimer(m_config.update_pd, &Motor::set_update_flag, this);
  m_update_timer.start();
}

void Motor::set_setpoint(ros::Time time, float setpoint, float acceleration) {
  m_setpoint = setpoint;
  m_accel_setpoint = acceleration;
  m_last_set_time = time;
}

CanMotor::CanMotor(ros::NodeHandle nh, MotorConfig const& config)
    : Motor(nh, config), m_can_id(config.id) {
  m_can_tx_pub = m_nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
  m_can_rx_sub =
      m_nh.subscribe("can_frames_rx", 128, &CanMotor::can_rx_callback, this);
}
