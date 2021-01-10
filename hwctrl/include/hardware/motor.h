#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <hwctrl/CanFrame.h>
#include <hwctrl/MotorData.h>

#include <boost/utility/string_view.hpp>

#include <string>
#include <vector>

#include "sensor.h"

#define DEFAULT_MAX_ACCEL 			30.0
#define DEFAULT_MAX_RPM 			50.0
#define MOTOR_LOOP_PERIOD 			0.005

enum class ControlType {
    None, RPM, Position
};

enum class MotorType {
    None, Vesc, Sabertooth, BMC
};

MotorType get_motor_type(boost::string_view);

class Motor {
public:
    Motor(
        ros::NodeHandle nh, const std::string& name, uint32_t id, MotorType m_type,
        ControlType c_type, ros::Duration update_pd = ros::Duration(MOTOR_LOOP_PERIOD), float accel_setpoint = DEFAULT_MAX_ACCEL, float max_accel = DEFAULT_MAX_ACCEL, 
        float max_rpm = DEFAULT_MAX_RPM, float gear_reduc = 1.0f, ros::Duration timeout = ros::Duration(2.0)
    );
    virtual ~Motor() = default;

    // allow moves but not copies
    Motor(Motor const&) = delete;
    void operator=(Motor const&) = delete;

    virtual void setup()                = 0;
    virtual void update(ros::Time time) = 0; // send the setpoint and return sensor data
    virtual void stop()                 = 0;
    virtual void limit_stop(LimitSwitch::Direction direction) {};

    void set_setpoint(ros::Time time, float setpoint = 0.0f, float acceleration = 0.0f);

    inline const std::string& get_name() const { return m_name; }
    inline uint32_t           get_id()   const { return m_id; }
    inline MotorType          get_motor_type() const { return m_type; }
    inline ControlType        get_control_type() const { return m_control_type; }

    inline bool is_online()       const { return m_online; }
    inline bool ready_to_update() const { return m_update; }

private:
    void set_update_flag(const ros::TimerEvent&) {
        m_update = true;
    }

protected:
    ros::NodeHandle m_nh;
    ros::Publisher  m_motor_data_pub;
    ros::Subscriber m_motor_cmd_sub;

    std::string   m_name;
    uint32_t      m_id;
    MotorType     m_type;
    ControlType   m_control_type;

    float         m_setpoint      = 0.0;
    float         m_last_setpoint = 0.0;
    float         m_accel_setpoint;
    float         m_max_rpm;
    float         m_max_accel;

    float         m_rpm_coef;
    
    ros::Time     m_last_set_time;
    ros::Time     m_last_update_time;
    ros::Duration m_timeout;

    ros::Duration m_update_pd;
    ros::Timer    m_update_timer;

    bool          m_update   = false;
    bool          m_online   = false;
    bool          m_is_setup = false;
};

class CanMotor : public Motor {
public:
    CanMotor(
        ros::NodeHandle nh, const std::string& name, uint32_t id,  uint32_t can_id, MotorType m_type,
        ControlType c_type, ros::Duration update_pd = ros::Duration(MOTOR_LOOP_PERIOD), float accel_setpoint = DEFAULT_MAX_ACCEL, float max_accel = DEFAULT_MAX_ACCEL, 
        float max_rpm = DEFAULT_MAX_RPM, float gear_reduc = 1.0f, ros::Duration timeout = ros::Duration(2.0)
    );
    virtual ~CanMotor() = default;
    
protected:
    using FramePtr = boost::shared_ptr<hwctrl::CanFrame>;
    virtual void can_rx_callback(FramePtr frame) = 0;
protected:
    uint32_t m_can_id;

    ros::Publisher  m_can_tx_pub;
    ros::Subscriber m_can_rx_sub;
};

class SerialMotor : public Motor {
    
};
