#pragma once

#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <hwctrl2/CanFrame.h>

static ros::Subscriber can_rx_sub;
static ros::Publisher  can_tx_pub;

class CanbusThread {
public:
    CanbusThread(ros::NodeHandle nh, std::string iface = "can1");
    ~CanbusThread() = default;

    int  init();
    void sleep();
    void shutdown();

    void operator()();

private:
    ros::NodeHandle    m_nh;
    ros::Rate          m_loop_rate;
    ros::Publisher     m_can_rx_pub;
    ros::Subscriber    m_can_tx_sub;
    ros::CallbackQueue m_cb_queue;

    std::string m_iface;
    int         m_sock;
    bool        m_sock_ready; 

    // frames from node to bus
    int read_can_frames();

    // frames from bus to node
    void can_tx_callback(boost::shared_ptr<hwctrl2::CanFrame> frame);
};