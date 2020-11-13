#include "hardware/uwb.h"


UwbNode::UwbNode(CanSensorArgs)
: CanSensorArgsPass(PubData), m_anchors()
{
    // reserve space for 5 anchors
    m_anchors.reserve(5);
};

void UwbNode::setup() {

    m_is_setup = true;
}

void UwbNode::update() {

    m_update = false;
}

void UwbNode::can_rx_callback(FramePtr frame) {

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

