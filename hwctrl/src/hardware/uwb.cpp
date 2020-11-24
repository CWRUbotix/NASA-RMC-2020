#include "pch.h"
#include "hardware/uwb.h"

#include <linux/can.h>
#include <linux/can/raw.h>

UwbNode::UwbNode(CanSensorArgs)
: CanSensorArgsPass(PubData), m_anchors()
{
    // reserve space for 5 anchors
    m_anchors.reserve(5);
};

void UwbNode::setup() {

    m_is_setup = true;
}

void UwbNode::ping() {
    auto msg = boost::make_shared<hwctrl::CanFrame>();
    msg->can_id = m_can_id | CAN_RTR_FLAG;
    msg->can_dlc = 8;
    m_can_tx_pub.publish(msg);
}

void UwbNode::update() {
    // dont actually query sensors here, we want this to be done in a certain way

    // publish if the data is ready. Otherwise dont
    m_update = false;
}

void UwbNode::can_rx_callback(FramePtr frame) {
    uint32_t rx_id = (uint32_t)frame->can_id;
	uint32_t can_id = (rx_id & 0xFF);
	if(m_can_id != can_id){
        // not for us
        return;
    }

    ROS_DEBUG("UWB frame (node id %d)", can_id);

    hwctrl::UwbData msg;
    msg.anchor_id = frame->data[1];
    float temp = 0.0;
    memcpy(&temp, frame->data.data() + 2, 4);
    msg.distance = temp;
    msg.node_id = can_id;
    m_pub.publish(msg);
}

Anchor const* UwbNode::get_anchor_by_id(uint32_t id) {
    for(AnchorIter it = m_anchors.begin(); it != m_anchors.end(); ++it) {
        if(it->id == id) {
            return &(*it);
        }
    }

    m_anchors.emplace_back<Anchor>(id);
    return &(*m_anchors.end());
}

