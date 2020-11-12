#pragma once

#include "sensor.h"

class QuadEncoder : public CanSensor<hwctrl2::SensorData> {
public:
    QuadEncoder(CanSensorArgs, bool inverted);
    virtual ~QuadEncoder() = default;

    void setup() override final;
    void update(const ros::TimerEvent&) override final;

    void can_rx_callback(boost::shared_ptr<hwctrl2::CanFrame> frame) override final;
private:
    bool m_inverted;

public:
    enum IncomingFrameIdent {

    };
    enum OutgoingFrameIdent {

    };
};
