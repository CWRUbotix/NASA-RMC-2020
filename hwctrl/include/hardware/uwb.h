#pragma once

#include "sensor.h"

#include <vector>

#define UWB_CAN_HDDR_SIZE 2

enum class UwbCanData {
  TIMESTAMP = 0,
  X,
  Y,
  Z,
  DISTANCE,
  RX_POWER,
  FP_POWER,
  FP_SNR,
  RANGING_DONE,
  LEGACY
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