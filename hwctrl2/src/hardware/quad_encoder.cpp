#include "hardware/quad_encoder.h"

QuadEncoder::QuadEncoder(CanSensorArgs, bool inverted)
: CanSensorArgsPass(PubData), m_inverted(inverted)
{}

void QuadEncoder::setup() {
    auto frame = boost::make_shared<hwctrl2::CanFrame>();
    frame -> can_id = m_id;
    // send inversion frame
    frame -> can_dlc = 2; // 1 byte for frame ident, 1 byte for inversion bit

    // copy polarity command id into frame
   frame -> data[0] = (uint8_t)0x02; // TODO: CHECK THIS ID PLEASE!!!!!!
   frame -> data[1] = (uint8_t)m_inverted;

    // publish frame to configure quad encoder
    m_pub.publish(frame);

    m_is_setup = true;
}

void QuadEncoder::update() {
    // sent rtr frame to id;
    auto frame = boost::make_shared<hwctrl2::CanFrame>();
    frame -> can_id = m_id;
    m_pub.publish(frame);

    m_update = false;
}

void QuadEncoder::can_rx_callback(boost::shared_ptr<hwctrl2::CanFrame> frame) {
    if(frame->can_id != m_id) {
        // not for us
        return;
    }

    // this is a frame for us. First bit will be frame type
    auto frame_type = frame->data[0];
    switch(frame_type) {
        case 0: {
            // error frame
            if(frame->can_dlc < 2) {
                ROS_WARN("Received bad can_frame from %s (id: %d)", m_name.c_str(), m_id);
                break;
            }
            ROS_WARN("Recieved error frame from %s (id: %d) - error id: %d", m_name.c_str(), m_id, frame->data[1]);
            break;
        }
        case 1: {
            if(frame->can_dlc < 5) {
                ROS_WARN("Received bad can_frame from %s (id: %d)", m_name.c_str(), m_id);
                break;
            }
            int32_t temp;
            memcpy(&temp, &frame->data[1], 4);

            auto msg = boost::make_shared<hwctrl2::SensorData>();
            msg -> name = m_name;
            msg -> sensor_id = m_id;
            msg -> value = (float) temp;
            m_pub.publish(msg);

            break;
        }
        default:
            break;
    }


}