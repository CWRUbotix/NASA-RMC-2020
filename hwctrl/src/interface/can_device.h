//
// Created by bscholar on 10/23/21.
//
#pragma once

#include <cinttypes>

#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <boost/shared_ptr.hpp>

#include <hwctrl/CanFrame.h>

class CanDevice {
public:
    CanDevice(const ros::NodeHandle& nh, uint32_t id);
    ~CanDevice();

    virtual void can_rx_callback(hwctrl::CanFramePtr frame) = 0;

private:
    void can_rx_callback_internal(hwctrl::CanFramePtr  frame);

private:
    uint32_t m_can_id;
    ros::Subscriber m_can_rx_sub;
    ros::Publisher  m_can_tx_pub;
};

