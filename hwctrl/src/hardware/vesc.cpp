#include "hardware/vesc.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#include "types.h"
#include "util.h"

VescMotor::VescMotor(ros::NodeHandle nh, const std::string& name, uint32_t id,
                     uint32_t can_id, ros::Duration update_pd,
                     float accel_setpoint, float max_accel, float max_rpm,
                     float gear_reduc, ros::Duration timeout)
    : CanMotor(nh, name, id, can_id, MotorType::Vesc, ControlType::RPM,
               update_pd, accel_setpoint, max_accel, max_rpm, gear_reduc,
               timeout) {}

void VescMotor::setup() {
  // something maybe
  m_is_setup = true;
}

void VescMotor::can_rx_callback(FramePtr frame) {
  const uint32_t rx_id = (uint32_t)frame->can_id;
  const uint8_t can_id = (uint8_t)(rx_id & 0xFF);  // extract only the id

  if((rx_id & ~0xFF) == 0) {
    return;
  }

  const uint8_t cmd = (uint8_t)(rx_id >> 8);
  uint8_t* frame_data = frame->data.data();

  switch (cmd) {
    case CanPacketId::CAN_PACKET_STATUS: {
      if(m_id != can_id) {
        break;
      }
      
      m_online = true;

      float rpm;
      float current_in;
      float duty_now;
      {
        size_t idx = 0;
        rpm = buffer::get_floating<float, int32_t>(frame_data, 1.0, idx);
        
        // use these eventually?
        current_in = buffer::get_floating<float, int16_t>(frame_data, 10.0f, idx);
        duty_now = buffer::get_floating<float, int16_t>(frame_data, 1000.0, idx);
      }

      // publish rpm data
      auto msg = boost::make_shared<hwctrl::MotorData>();
      msg->data_type = msg->RPM;
      msg->id = m_id;
      msg->value = rpm / m_rpm_coef;
      msg->timestamp = ros::Time::now();
      m_motor_data_pub.publish(msg);

      break;
    }
    case CanPacketId::CAN_PACKET_FILL_RX_BUFFER: {
      if (can_id != 0) {
        break;
      }
      std::memcpy(&m_rx_buf + frame_data[0], frame_data + 1, frame->can_dlc - 1);
      break;
    }
    case CanPacketId::CAN_PACKET_PROCESS_RX_BUFFER: {
        if(can_id != 0) {
          break; 
        }
        int idx = 0;
        uint16_t packet_len;
        uint16_t crc;

        int vesc_id = frame_data[idx++];

        if(vesc_id != m_id) {
          std::memset(&m_rx_buf, 0, 1024);
          break;
        }

        int n_commands = frame_data[idx++];
        packet_len = (frame_data[idx++] << 8);
        packet_len |= frame_data[idx++];
        crc = (frame_data[idx++] << 8);
        crc |= frame_data[idx++];

        const auto chk_crc = crc::crc16(m_rx_buf.data(), packet_len);

        if(crc != chk_crc) {
          ROS_WARN("Error: Vesc checksum doesnt match");
          break;
        }

        idx = 0;
        int comm_cmd = m_rx_buf[idx++];
        switch(comm_cmd) {
          case COMM_GET_VALUES: {
            auto msg = boost::make_shared<hwctrl::MotorData>();
            msg->data_type = msg->RPM;
            msg->id = m_id;
            
            // eventually we may want to actually do something with this data
            // for now, we only use rpm
            float temp_mos1     = buffer::get_floating<float, int16_t>(m_rx_buf.data(), 10.0, idx);
            float temp_mos2     = buffer::get_floating<float, int16_t>(m_rx_buf.data(), 10.0, idx);
            float current_motor = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 100.0, idx);
            float current_in    = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 100.0, idx);
            float avg_id        = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 100.0, idx);
            float avg_iq        = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 100.0, idx);
            float duty_now      = buffer::get_floating<float, int16_t>(m_rx_buf.data(), 1000.0, idx);
            float rpm           = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 1.0, idx);
            float v_in          = buffer::get_floating<float, int16_t>(m_rx_buf.data(), 10.0, idx);
            float amp_hours          = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 10000.0, idx);
            float amp_hours_charged  = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 10000.0, idx);
            float watt_hours         = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 10000.0, idx);
            float watt_hours_charged = buffer::get_floating<float, int32_t>(m_rx_buf.data(), 10000.0, idx);
            int32_t tachometer       = buffer::get<int32_t>(m_rx_buf.data(), idx);
            int32_t tachometer_abs   = buffer::get<int32_t>(m_rx_buf.data(), idx);
            int8_t fault_code        = (int8_t) m_rx_buf[idx++];

            msg->timestamp = ros::Time::now();
            msg->value = rpm / m_rpm_coef;
            m_motor_data_pub.publish(msg);
            break;
          }
        }
      break;
    }
    default:
      // These are depreciated
      ROS_DEBUG("Vesc (id: %d) recieved depreciated frames", m_id);
      break;
  }
}

void VescMotor::update(ros::Time time) {
  if (!m_online) {
    return;
  }

  float delta = m_setpoint - m_last_setpoint;
  float dt = (float)(time.toSec() - m_last_update_time.toSec());
  float accel = std::fabs(delta / dt);

  if (accel > std::fabs(m_accel_setpoint)) {
    if (delta > 0) {
      delta = std::fabs(m_accel_setpoint) * dt;  // new delta (is positive)
    } else {
      delta = -1.0 * std::fabs(m_accel_setpoint) *
              dt;  // new delta (needs to be negative)
    }
  }
  m_last_setpoint += delta;

  if ((time.toSec() - m_last_set_time.toSec()) > m_timeout.toSec()) {
    // shut down the motor
    m_last_setpoint = 0;
  }

  float eRPM = m_rpm_coef * m_last_setpoint;

  m_last_update_time = time;
  send_rpm_frame(time, eRPM);

  // send update frame
  if (!m_vesc_pending_update) {
    send_values_frame(time);
    m_vesc_pending_update = true;
  }
  m_update = false;
}

void VescMotor::stop() {
  // TODO: Make this actually work
  set_setpoint(ros::Time::now(), 0.0);
}

void VescMotor::send_rpm_frame(ros::Time time, float rpm) {
  auto frame = boost::make_shared<hwctrl::CanFrame>();

  int n_rpm = (int)rpm;
  frame->can_id =
      (CanPacketId::CAN_PACKET_SET_RPM << 8) | m_can_id | CAN_EFF_FLAG;

  // memcpy(frame.data, &n_rpm, sizeof(n_rpm));
  frame->data[0] = (uint8_t)(n_rpm >> 24);
  frame->data[1] = (uint8_t)(n_rpm >> 16);
  frame->data[2] = (uint8_t)(n_rpm >> 8);
  frame->data[3] = (uint8_t)(n_rpm & 0xFF);
  frame->can_dlc = sizeof(n_rpm);

  m_can_tx_pub.publish(frame);
  m_last_update_time = time;
}

void VescMotor::send_values_frame(ros::Time time) {
  auto frame = boost::make_shared<hwctrl::CanFrame>();

  frame->can_id = m_can_id |
                  (CanPacketId::CAN_PACKET_PROCESS_SHORT_BUFFER << 8) |
                  CAN_EFF_FLAG;
  uint8_t ind = 0;
  frame->data[ind++] = 0;  // our id
  frame->data[ind++] = 0x00;
  frame->data[ind++] = CommPacketId::COMM_GET_VALUES;
  frame->can_dlc = ind;
  m_can_tx_pub.publish(frame);
}
