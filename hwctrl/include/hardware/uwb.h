#pragma once

#include "sensor.h"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <vector>


#define UWB_CAN_HDDR_SIZE 2

enum UwbCanData {
  UWB_TIMESTAMP = 0,
  UWB_X,
  UWB_Y,
  UWB_Z,
  UWB_DISTANCE,
  UWB_RX_POWER,
  UWB_FP_POWER,
  UWB_FP_SNR,
  UWB_RANGING_DONE,
  UWB_LEGACY
};

struct Anchor {
    Anchor(uint32_t id) : id(id) {};

    uint32_t id;
    uint32_t timestamp;
    float x;
    float y;
    float z;
    float distance;
    float rx_power;
    float fp_power;
    float fp_snr;
    bool data_ready;

    std::string to_string();
};


class UwbNode : public CanSensor<hwctrl::UwbData> {
public:
    UwbNode(CanSensorArgs);
    virtual ~UwbNode() = default;

    virtual void setup()  override final;
    virtual void update() override final;

    void ping();

    virtual void can_rx_callback(FramePtr frame) override final;

    inline uint32_t get_num_anchors() const { return m_anchors.size(); }

    Anchor const* get_anchor_by_id(uint32_t id);

private:
    std::vector<Anchor> m_anchors;

    using AnchorIter = std::vector<Anchor>::iterator;
};