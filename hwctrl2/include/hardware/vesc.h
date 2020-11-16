#pragma once

#include "motor.h"

class Vesc : public CanMotor {
public:
    Vesc(
        ros::NodeHandle nh, const std::string& name, uint32_t id,
        ControlType c_type, ros::Duration update_pd, ros::Duration timeout = ros::Duration(2.0), uint32_t can_id = 0
    );
    virtual ~Vesc() = default;

    virtual void setup() override final;
    virtual void update(ros::Time time) override final;

    virtual void can_rx_callback(FramePtr frame) override final;
private:
    void send_rpm_frame(ros::Time time, float rpm);
    void send_values_frame(ros::Time time);
    
private:
    bool m_vesc_pending_update = true;
};