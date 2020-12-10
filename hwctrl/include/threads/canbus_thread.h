#pragma once

#include <string>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <hwctrl/CanFrame.h>

#include "hwctrl_thread.h"

static ros::Subscriber can_rx_sub;
static ros::Publisher can_tx_pub;

class CanbusThread : public HwctrlThread {
 public:
  CanbusThread(ros::NodeHandle nh, std::string iface = "can1");
  ~CanbusThread() = default;
  
  virtual void setup() override final;
  virtual void update(ros::Time) override final;
  virtual void shutdown() override final;

 private:
  ros::Publisher m_can_rx_pub;
  ros::Subscriber m_can_tx_sub;

  std::string m_iface;
  int m_sock;
  bool m_sock_ready;

  // frames from node to bus
  int read_can_frames();

  // frames from bus to node
  void can_tx_callback(boost::shared_ptr<hwctrl::CanFrame> frame);
};
