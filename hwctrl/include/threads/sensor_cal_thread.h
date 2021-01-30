#pragma once

#include <ros/node_handle.h>

#include <boost/utility/string_view.hpp>

#include <string>
#include <vector>

#include "hwctrl.h"
#include "threads/sensor_thread.h"

class SensorCalThread : public SensorThread {
 public:
  SensorCalThread(ros::NodeHandle);
  virtual ~SensorCalThread() = default;
  
  virtual void setup() override final;
  virtual void update(ros::Time) override final;
  virtual void shutdown() override final;

 private:
  void calibrate_sensor_with_name(boost::string_view name);
 private:
  std::string m_line;
  std::string m_cal_path;
  std::vector<Calibration> m_cals;
  bool m_done = false;

};
