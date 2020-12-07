#include "hardware/quad_encoder.h"

#include <linux/can.h>
#include <linux/can/raw.h>

QuadEncoder::QuadEncoder(CanSensorArgs, bool inverted)
    : CanSensorArgsPass(PubData), m_inverted(inverted) {}

void QuadEncoder::setup() {
  // send inversion frame
  auto frame = boost::make_shared<hwctrl::CanFrame>();
  frame->can_id = m_can_id | (0x02 << 8);  // set id and command;
  frame->can_dlc = 1;  // 1 inversion byte, command in id buffer

  // copy polarity into frame
  frame->data[0] = (uint8_t)m_inverted;

  // publish frame to configure quad encoder
  m_pub.publish(frame);

  m_is_setup = true;
}

void QuadEncoder::update() {
  // sent rtr frame to device
  // requests a tick frame
  auto frame = boost::make_shared<hwctrl::CanFrame>();
  frame->can_id = m_can_id | CAN_RTR_FLAG;
  frame->can_dlc = 8;
  m_pub.publish(frame);

  m_update = false;
}

void QuadEncoder::can_rx_callback(FramePtr frame) {
  const uint32_t rx_id = (uint32_t)frame->can_id;
  const uint8_t can_id =
      (uint8_t)(rx_id & 0xFF);  // extract only the id (first 8 bits)

  if (m_can_id != can_id) {
    // not for us
    return;
  }

  // get command stored in second bit of rx_id
  // (stored in highest 3 bits of rx_id)
  const uint8_t cmd = (uint8_t)(rx_id >> 8) & 0b111;

  // this is a frame for us. First bit will be frame type
  switch (cmd) {
    case 0: {
      // error frame
      if (frame->can_dlc < 1) {
        ROS_WARN("Received bad can_frame from %s (id: %d)", m_name.c_str(),
                 m_id);
        break;
      }
      ROS_WARN("Recieved error frame from %s (id: %d) - error id: %d",
               m_name.c_str(), m_id, frame->data[0]);
      break;
    }
    case 1: {
      if (frame->can_dlc < 4) {
        ROS_WARN("Received bad can_frame from %s (id: %d)", m_name.c_str(),
                 m_id);
        break;
      }
      int32_t temp;
      memcpy(&temp, &frame->data[0], 4);

      auto msg = boost::make_shared<hwctrl::SensorData>();
      msg->name = m_name;
      msg->sensor_id = m_id;
      msg->value = (float)temp;
      m_pub.publish(msg);

      break;
    }
    default:
      ROS_WARN("Unhandled encoder frame. can_id: %d", m_can_id);
      break;
  }
}