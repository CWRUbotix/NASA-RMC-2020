#pragma once

#include "sensor.h"

class QuadEncoder : public CanSensor<hwctrl::SensorData> {
 public:
  QuadEncoder(SensorConfig const& config, bool inverted = false);
  virtual ~QuadEncoder() = default;

  virtual void setup() override final;
  virtual void update() override final;
  virtual void calibrate(std::vector<Calibration>&) override final{};

  virtual void can_rx_callback(FramePtr frame) override final;

 private:
  bool m_inverted;

 public:
  enum class IncomingFrameIdent : uint8_t {

  };
  enum class OutgoingFrameIdent : uint8_t {

  };
};
