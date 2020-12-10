#pragma once

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <ros/time.h>

#include <string>

class HwctrlThread {
 public:
   HwctrlThread(std::string name, ros::NodeHandle nh, ros::Rate update_pd)
     : m_name(name), m_nh(nh), m_update_pd(update_pd) {}
  ~HwctrlThread() = default;
    
  // delete copy constructors
  HwctrlThread(HwctrlThread const&) = delete;
  void operator=(HwctrlThread const&) = delete;

  // delete move constructors
  HwctrlThread(HwctrlThread&&) = delete;
  void operator=(HwctrlThread&&) = delete;
  
  virtual void setup() = 0;
  virtual void update(ros::Time time) = 0;
  virtual void shutdown() = 0;

  inline void sleep() { m_update_pd.sleep(); }

  void operator()() {
    ROS_INFO("Starting %s", m_name.c_str());
    m_nh.setCallbackQueue(&m_cb_queue);
    ros::AsyncSpinner spinner(1, &m_cb_queue);
    spinner.start(); 
    while(m_nh.ok()) {
      update(ros::Time::now());
      sleep();
    }
    ROS_INFO("Shutting down %s", m_name.c_str());
    shutdown();
  }

 protected:
  ros::NodeHandle m_nh;
  ros::CallbackQueue m_cb_queue;
  ros::Rate m_update_pd;

  std::string m_name;
};
