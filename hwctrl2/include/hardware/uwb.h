#pragma once

#include "sensor.h"

struct AnchorData{
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


class UwbNode : public CanSensor {



}