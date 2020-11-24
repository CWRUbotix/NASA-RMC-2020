#include "hardware/vesc.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#include "types.h"
#include "util.h"

VescMotor::VescMotor(
    ros::NodeHandle nh, const std::string& name, uint32_t id,  uint32_t can_id,
    ros::Duration update_pd, float accel_setpoint, float max_accel, 
    float max_rpm, float gear_reduc, ros::Duration timeout
) : CanMotor(nh, name, id, can_id, MotorType::Vesc, ControlType::RPM, update_pd, accel_setpoint, max_accel, max_rpm, gear_reduc, timeout) {

}

void VescMotor::setup() {
    // something maybe
    m_is_setup = true;
}

void VescMotor::can_rx_callback(FramePtr frame) {
    uint32_t rx_id = (uint32_t) frame->can_id;
    int8_t can_id = (int8_t)(rx_id & 0xFF); // extract only the id

    if(m_can_id != can_id) {
        // not for us
        return;
    }

    uint8_t cmd = (uint8_t)(rx_id >> 8);

    switch(cmd) {
        case CanPacketId::CAN_PACKET_STATUS: {
            m_online = true;

            float rpm;
            {
                int idx = 0;
                rpm = buffer::get_float16(frame->data.data(), 1.0f, &idx);
                // maybe grab some other data too?
            }

            // publish rpm data
            hwctrl::MotorData msg;
            msg.data_type = msg.RPM;
            msg.id = m_id;
            msg.value = rpm / m_rpm_coef;
            msg.timestamp = ros::Time::now();
            // ROS_INFO("Publish to motor_data");
            m_motor_data_pub.publish(msg);

            break;
        }
        case CanPacketId::CAN_PACKET_FILL_RX_BUFFER:
        case CanPacketId::CAN_PACKET_PROCESS_RX_BUFFER:
        default:
            // These are depreciated
            ROS_DEBUG("Vesc (id: %d) recieved depreciated frames", m_id);
            break;
    }

}

void VescMotor::update(ros::Time time) {
    if(!m_online) {
        return;
    }

    float delta = m_setpoint - m_last_setpoint;
    float dt = (float) (time.toSec() - m_last_update_time.toSec());
    float accel = std::fabs(delta / dt);

    if(accel > std::fabs(m_accel_setpoint)) {
        if (delta > 0) {
            delta = std::fabs(m_accel_setpoint) * dt; // new delta (is positive)
        } else {
            delta = -1.0 * std::fabs(m_accel_setpoint) * dt; // new delta (needs to be negative)
        }
    }
    m_last_setpoint += delta;

    if((time.toSec() - m_last_set_time.toSec()) > m_timeout.toSec()) {
        // shut down the motor
        m_last_setpoint = 0;
    }

    float eRPM = m_rpm_coef * m_last_setpoint;

    m_last_update_time = time;
    send_rpm_frame(time, eRPM);
    
    // send update frame
    if(!m_vesc_pending_update) {
        send_values_frame(time);
        m_vesc_pending_update = true;
    }
    m_update = false;
}

void VescMotor::send_rpm_frame(ros::Time time, float rpm) {
    auto frame = boost::make_shared<hwctrl::CanFrame>();

    int n_rpm = (int)rpm;
	frame->can_id = (CanPacketId::CAN_PACKET_SET_RPM << 8) | m_can_id | CAN_EFF_FLAG;

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

    frame->can_id = m_can_id | (CanPacketId::CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | CAN_EFF_FLAG;
    uint8_t ind = 0;
    frame->data[ind++] = 0; // our id
    frame->data[ind++] = 0x00;
    frame->data[ind++] = CommPacketId::COMM_GET_VALUES;
    frame->can_dlc = ind;
    m_can_tx_pub.publish(frame);
}