#pragma once

#include "sensor.h"

class QuadEncoder : public CanSensor<hwctrl::SensorData> {
public:
    QuadEncoder(CanSensorArgs, bool inverted = false);
    virtual ~QuadEncoder() = default;

    void setup()  override final;
    void update() override final;

    void can_rx_callback(FramePtr frame) override final;
private:
    bool m_inverted;

public:
    enum IncomingFrameIdent {

    };
    enum OutgoingFrameIdent {

    };
};