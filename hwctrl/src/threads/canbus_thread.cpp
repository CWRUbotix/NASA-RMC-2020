#include "threads/canbus_thread.h"

#include <ros/spinner.h>

#include <hwctrl/CanFrame.h>

#include <inttypes.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

CanbusThread::CanbusThread(ros::NodeHandle nh, std::string iface)
 : HwctrlThread("canbus_thread", nh, 1000), m_iface(iface) {

  // PUBLISHERS
  m_can_rx_pub = m_nh.advertise<hwctrl::CanFrame>("can_frames_rx", 128);

  // SUBSCRIBERS
  m_can_tx_sub = m_nh.subscribe("can_frames_tx", 128,
                                &CanbusThread::can_tx_callback, this);
}

void CanbusThread::setup() {
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 10000;  // 10ms CAN bus read timeout

  // initialize the CAN socket
  if ((m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    ROS_ERROR("Error while opening CAN socket");
    m_sock_ready = false;
    return;
  }

  strcpy(ifr.ifr_name, m_iface.c_str());
  ioctl(m_sock, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  ROS_DEBUG("%s at index %d", m_iface.c_str(), ifr.ifr_ifindex);

  setsockopt(m_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));

  // socket bind
  if (bind(m_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ROS_ERROR("Error in CAN socket bind");
    m_sock_ready = false;
    return;
  }
  m_sock_ready = true;
  ROS_DEBUG("Canbus init success");
}

void CanbusThread::can_tx_callback(boost::shared_ptr<hwctrl::CanFrame> frame) {
  // ROS_INFO("Writing frame to CAN bus");
  struct can_frame f;
  f.can_id = frame->can_id;
  f.can_dlc = frame->can_dlc;
  for (int i = 0; i < frame->can_dlc; i++) {
    f.data[i] = frame->data[i];
  }
  // WRITE CAN FRAME TO SOCKET IMMEDIATELY
  if (m_sock_ready) {
    int n_bytes = write(m_sock, &f, sizeof(f));
  }
}

void CanbusThread::update(ros::Time) {
  if (!m_sock_ready)
    return;
  int retval = 0, nbytes = 0;
  struct can_frame rx_frame;
  while ((nbytes = read(m_sock, &rx_frame, sizeof(struct can_frame))) > 0) {
    auto frame_msg = boost::make_shared<hwctrl::CanFrame>();
    frame_msg->can_id = rx_frame.can_id;
    frame_msg->can_dlc = rx_frame.can_dlc;
    for (int i = 0; i < rx_frame.can_dlc; i++) {
      frame_msg->data[i] = rx_frame.data[i];
    }
    ROS_INFO("Publishing to can_frames_rx");
    m_can_rx_pub.publish(frame_msg);  // publish message immediately
    retval++;
  }
  
  if(retval > 0)
    ROS_INFO("Sent %d can frames.", retval);
}

void CanbusThread::shutdown() { close(m_sock); }

