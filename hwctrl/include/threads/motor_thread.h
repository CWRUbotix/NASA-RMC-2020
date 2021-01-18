#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <hwctrl/LimitSwState.h>
#include <hwctrl/MotorCmd.h>
#include <hwctrl/MotorData.h>
#include <hwctrl/SensorData.h>
#include <hwctrl/CanFrame.h>

#include <boost/move/make_unique.hpp>
#include <boost/move/unique_ptr.hpp>

#include <vector>
#include <map>

#include "hardware/motor.h"
#include "hwctrl_thread.h"

class MotorThread : public HwctrlThread {
 public:
  MotorThread(ros::NodeHandle nh);
  ~MotorThread() = default;
   
  void read_from_server();
  
  void setup_motors();
  void stop_motors();
  
  virtual void setup() override final;
  virtual void update(ros::Time time) override final;
  virtual void shutdown() override final;

 private:
  std::vector<std::string> get_limit_switch_topics();
  
  void set_motor_callback(boost::shared_ptr<hwctrl::MotorCmd> msg);
  void limit_switch_callback(boost::shared_ptr<hwctrl::LimitSwState> msg);
  void estop_callback(boost::shared_ptr<std_msgs::Bool> msg);

 protected:
  using FramePtr = boost::shared_ptr<hwctrl::CanFrame>;
  virtual void can_rx_callback(FramePtr frame);

 private:

  // subscribers
  ros::Subscriber m_motor_set_sub;
  ros::Subscriber m_estop_sub;
  ros::Subscriber m_can_rx_sub;
  std::vector<ros::Subscriber> m_ls_subs;

  // motors
  std::map<uint32_t, boost::shared_ptr<Motor>> m_motors;

  std::vector<uint8_t> m_vesc_buffer;

  bool m_sys_power_on = false;
};
