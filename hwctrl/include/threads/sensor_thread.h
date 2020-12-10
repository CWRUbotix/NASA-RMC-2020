#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/timer.h>

#include <boost/move/make_unique.hpp>
#include <boost/move/unique_ptr.hpp>
#include <boost/utility/string_view.hpp>

#include <vector>

#include "hwctrl.h"

#include "hwctrl_thread.h"

#include "hardware/sensor.h"
#include "hardware/uwb.h"

using boost::movelib::make_unique;
using boost::movelib::unique_ptr;

class SensorThread : HwctrlThread {
 public:
  SensorThread(ros::NodeHandle nh);
  virtual ~SensorThread() = default;

  void configure_from_server(boost::shared_ptr<Spi> spi);

  // Dont think we need this anymore
  // void configure_from_csv();

  void setup_sensors();
  void update_sensors();

  void shutdown();
  void sleep();

  virtual void operator()();

 private:
  boost::shared_ptr<Sensor> create_sensor_from_values(
      ros::NodeHandle nh, std::string name, std::string topic, SensorType type,
      uint32_t id, ros::Duration period, unique_ptr<Gpio> gpio,
      boost::shared_ptr<Spi> spi);

  void uwb_ping_callback(const ros::TimerEvent&);

 protected:
  ros::NodeHandle m_nh;
  ros::Rate m_loop_rate;
  ros::CallbackQueue m_cb_queue;

  using SensorVec = std::vector<boost::shared_ptr<Sensor>>;
  using SensorVecIter = SensorVec::iterator;

  SensorVec m_sensors;

  std::vector<boost::shared_ptr<UwbNode>> m_uwb_nodes;
  uint32_t m_uwb_idx;
  ros::Duration m_uwb_update_pd;
  ros::Timer m_uwb_update_timer;
};
