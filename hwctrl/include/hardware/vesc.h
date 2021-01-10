#pragma once

#include "motor.h"

class VescMotor : public CanMotor {
 public:
  VescMotor(ros::NodeHandle nh, const std::string& name, uint32_t id,
            uint32_t can_id,
            ros::Duration update_pd = ros::Duration(MOTOR_LOOP_PERIOD),
            float accel_setpoint = DEFAULT_MAX_ACCEL,
            float max_accel = DEFAULT_MAX_ACCEL,
            float max_rpm = DEFAULT_MAX_RPM, float gear_reduc = 1.0f,
            ros::Duration timeout = ros::Duration(2.0));
  virtual ~VescMotor() = default;

  virtual void setup() override final;
  virtual void update(ros::Time time) override final;
  virtual void stop() override final;

  virtual void can_rx_callback(FramePtr frame) override final;
  virtual void update_values_from_buffer(uint8_t* buffer);

 protected:
  float m_temp_mos1;
  float m_temp_mos2;
  float m_current_motor;
  float m_current_in;
  float m_avg_id;
  float m_avg_iq;
  float m_duty_now;
  float m_rpm;
  float m_v_in;
  float m_amp_hours;
  float m_amp_hours_charged;
  float m_watt_hours;
  float m_watt_hours_charged;
  int32_t m_tachometer;
  int32_t m_tachometer_abs;
  int8_t m_fault_code;

 private:
  void send_rpm_frame(ros::Time time, float rpm);
  void send_values_frame(ros::Time time);

 private:
  bool m_vesc_pending_update = true;
};
