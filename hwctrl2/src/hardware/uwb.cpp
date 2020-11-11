#include "hardware/uwb.h"


UwbNode::UwbNode(CanSensorArgs)
: CanSensorArgsPass, m_anchors()
{
    // reserve space for 5 anchors
    m_anchors.reserve(5);
};

void UwbNode::setup() {

}

void UwbNode::update(const ros::TimerEvent&) {

}

void UwbNode::can_rx_callback(boost::shared_ptr<hwctrl2::CanFrame> frame) {

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

