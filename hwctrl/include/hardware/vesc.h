#pragma once

#include "motor.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#include "types.h"
#include "util.h"

class VescMotor : public CanMotor {
public:
    VescMotor(
        ros::NodeHandle nh, const std::string& name, uint32_t id,  uint32_t can_id,
        ros::Duration update_pd = ros::Duration(MOTOR_LOOP_PERIOD), float accel_setpoint = DEFAULT_MAX_ACCEL, float max_accel = DEFAULT_MAX_ACCEL, 
        float max_rpm = DEFAULT_MAX_RPM, float gear_reduc = 1.0f, ros::Duration timeout = ros::Duration(2.0)
    );
    virtual ~VescMotor() = default;

    virtual void setup() override final;
    virtual void update(ros::Time time) override final;

    virtual void can_rx_callback(FramePtr frame) override final;
private:
    void send_rpm_frame(ros::Time time, float rpm);
    void send_values_frame(ros::Time time);
    
private:
    bool m_vesc_pending_update = true;
};