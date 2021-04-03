#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/timer.h>

#include <boost/utility/string_view.hpp>

#include <vector>
#include <memory>

#include "hwctrl.h"

#include "hwctrl_thread.h"

#include "hardware/sensor.h"
#include "hardware/uwb.h"


class SensorThread : public HwctrlThread {
 public:
  SensorThread(ros::NodeHandle nh, std::string name = "sensor_thread");
  virtual ~SensorThread() = default;

  void configure_from_server(boost::shared_ptr<Spi> spi);
  
  virtual void setup() override;
  virtual void update(ros::Time) override;
  virtual void shutdown() override;

 private:
  boost::shared_ptr<Sensor> create_sensor_from_values(
      ros::NodeHandle nh, std::string name, std::string topic, SensorType type,
      uint32_t id, ros::Duration period, std::unique_ptr<Gpio> gpio,
      boost::shared_ptr<Spi> spi);

  void uwb_ping_callback(const ros::TimerEvent&);

 protected:
  using SensorVec = std::vector<boost::shared_ptr<Sensor>>;
  using SensorVecIter = SensorVec::iterator;

  SensorVec m_sensors;

  std::vector<boost::shared_ptr<UwbNode>> m_uwb_nodes;
  uint32_t m_uwb_idx;
  ros::Duration m_uwb_update_pd;
  ros::Timer m_uwb_update_timer;
};
