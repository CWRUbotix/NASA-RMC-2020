#pragma once

#include "motor.h"

#include <ros/ros.h>

class BMCMotor : CanMotor {
public:
    BMCMotor(
        ros::NodeHandle nh, const std::string& name, uint32_t id,  uint32_t can_id,
        ros::Duration update_pd = ros::Duration(MOTOR_LOOP_PERIOD), float accel_setpoint = DEFAULT_MAX_ACCEL, float max_accel = DEFAULT_MAX_ACCEL, 
        float max_rpm = DEFAULT_MAX_RPM, float gear_reduc = 1.0f, ros::Duration timeout = ros::Duration(2.0)
    );
    virtual ~BMCMotor() = default;

    virtual void setup()  override final;
    virtual void update(ros::Time time) override final;

    virtual void can_rx_callback(FramePtr frame) override final;
private:

};