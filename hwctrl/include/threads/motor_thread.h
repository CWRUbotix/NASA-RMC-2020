#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <hwctrl/LimitSwState.h>
#include <hwctrl/MotorCmd.h>
#include <hwctrl/MotorData.h>
#include <hwctrl/SensorData.h>

#include <boost/move/make_unique.hpp>
#include <boost/move/unique_ptr.hpp>

#include <vector>
#include <map>

#include "hardware/motor.h"
#include "hwctrl_thread.h"

class MotorThread : HwctrlThread {
 public:
  MotorThread(ros::NodeHandle nh);
  ~MotorThread() = default;
   
  void read_from_server();
  
  void setup_motors();
  void update_motors();

  void sleep();
  void shutdown();

  void operator()();

 private:
  std::vector<std::string> get_limit_switch_topics();
  
  void set_motor_callback(boost::shared_ptr<hwctrl::MotorCmd> msg);
  void limit_switch_callback(boost::shared_ptr<hwctrl::LimitSwState> msg);
  void estop_callback(boost::shared_ptr<std_msgs::Bool> msg);
 private:
  // general ros objects
  ros::NodeHandle m_nh;
  ros::Rate m_loop_rate;
  ros::CallbackQueue m_cb_queue;

  // subscribers
  ros::Subscriber m_motor_set_sub;
  ros::Subscriber m_estop_sub;
  std::vector<ros::Subscriber> m_ls_subs;

  // motors
  std::map<uint32_t, boost::shared_ptr<Motor>> m_motors;

  bool m_sys_power_on = false;
};
