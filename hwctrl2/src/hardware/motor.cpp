#include "hardware/motor.h"

#include <cmath>

Motor::Motor(ros::NodeHandle nh, const std::string& name, uint32_t id, MotorType type, ControlType c_type, ros::Duration update_pd, ros::Duration timeout)
: m_nh(nh), m_name(name), m_id(id), m_type(type), m_control_type(c_type), m_update_pd(update_pd), m_timeout(timeout)
{   
    m_motor_data_pub = m_nh.advertise<hwctrl2::MotorData>("motor_data", 128);
    m_update_timer = m_nh.createTimer(m_update_pd, &Motor::set_update_flag, this);
}

void Motor::set_setpoint(ros::Time time, float setpoint) {
    // m_last_setpoint = m_setpoint;
    m_setpoint = setpoint;
    m_last_set_time = time;
}

CanMotor::CanMotor(ros::NodeHandle nh, const std::string& name, uint32_t id, MotorType type, ControlType c_type, ros::Duration update_pd, ros::Duration timeout, uint32_t can_id)
: Motor(nh, name, id, type, c_type, update_pd, timeout), m_can_id(can_id)
{
    m_can_tx_pub = m_nh.advertise<hwctrl2::CanFrame>("can_frames_tx", 128);
    m_can_rx_sub = m_nh.subscribe("can_frames_rx", 128, &CanMotor::can_rx_callback, this);
}