//
// Created by bscholar on 10/23/21.
//

#include <string>
#include <cinttypes>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

// vars
std::string iface;
bool socket_ready;
int socket;

void can_tx_callback(boost::shared_ptr<hwctrl::CanFrame> frame) {
    struct can_frame f;
    f.can_id = frame->can_id;
    f.can_dlc =  frame->can_dlc;
    for(int i = 0; i < frame->can_dlc; ++i) {
        f.data[i] = frame->data[i];
    }
    if(socket_ready) {
        int n_bytes = write(socket, &f, sizeof(f));
    } else {
        // this is bad
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "can_bus_node");

    ros::NodeHandle nh;
    ros::Rate rate(1000);

    can_rx_pub = nh.advertise<hwctrl::CanFrame>("can_frames_rx", 128);
    can_tx_sub = nh.subscribe("can_frames_tx", 128, &can_tx_callback);

    if(nh.hasParam("/hwctrl/can_if")) {
        nh.getParam("/hwctrl/can_if", iface);
        ROS_INFO("Using can interface: %s", iface);
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    struct timeval tv;

    tv.tv_sec = 0;
    tv.tv_usec = 10000;

    if((socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("Error while opening CAN socket");
        socket_ready = false;
        return -1;
    }

    strcpy(ifr.ifr_name, iface.c_str());
    ioctl(socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    if(bind(socket, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
        ROS_ERROR("Error in CAN socket bind");
        socket_ready = false;
        return -1;
    }

    socket_ready = true;
    ROS_DEBUG("Canbus init successfull");

    int retval = 0, nbytes = 0;
    struct can_frame rx_frame;

    while(nh.ok()) {

        if(!socket_ready) {
            ROS_ERROR("Socket not ready");
            return -1;
        }

        while((nbytes = read(socket, &tx_frame,sizeof(struct can_frame))) > 0) {
            auto frame_msg = boost::make_shared<hwctrl::CanFrame>();
            frame_msg->can_id = rx_frame.can_id;
            frame_msg->can_dlc = rx_frame.can_dlc;

            for(int i = 0; i < rx_frame.can_dlc;++i) {
                frame_msg->data[i] = rx_frame.data[i];
            }
            ROS_INFO("Publishing to can_frames_tx");
            can_rx_pub.publish(frame_msg);
            retval++;
        }

        if(retval > 0)
            ROS_INFO("Sent %d can frames", retval);

        ros::spinOnce();
        rate.sleep();
    }

    close(socket);
    return 0;
}