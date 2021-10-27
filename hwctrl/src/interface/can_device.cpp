//
// Created by bscholar on 10/23/21.
//

#include "can_device.h"

constexpr uint32_t queue_size = 16;

CanDevice::CanDevice(const ros::NodeHandle& nh, uint32_t id) :
m_can_id(id)
{
    m_can_rx_sub = nh.subscribe("can_frames_rx",  queue_size, can_rx_callback_internal, this);
    m_can_tx_pub = nh.advertise<hwctrl::CanFrame>("can_frames_tx", queue_size);
}

void CanDevice::can_rx_callback_internal(hwctrl::CanFramePtr frame) {
    if(frame->can_id != m_can_id) {
        return;
    }
    can_rx_callback(frame);
}