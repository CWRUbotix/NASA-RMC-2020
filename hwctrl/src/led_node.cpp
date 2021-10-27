//
// Created by bscholar on 10/23/21.
//

#include <string>
#include <cinttypes>

#include <ros/ros.h>

// data
uint32_t can_id;
uint8_t r, g, b;
bool new_data;

bool color_handler(hwctrl::LED::Request& req, hwctrl::LED::Response* res) {
    new_data = true;
    r = req.r;
    g = req.g;
    b = req.b;
    ROS_INFO("Request: r=%d, g=%d, b=%d", r, g, b);
    // for now just say it succeeded
    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "led_node");

    ros::NodeHandler nh;
    ros::Rate rate(10);

    ros::Publisher can_tx_pub = nh.advertise<hwctrl::CanFrame>("can_frame_tx", 32);
    ros::ServiceServer color_srv = nh.advertiseService("led_node_service", color_handler);

    if(nh.hasParam("/hwctrl/led_node_id")) {
        int temp;
        nh.getParam("/hwctrl/led_node_id", temp);
        can_id = (uint32_t) temp;
    } else {
        ROS_ERROR("Could not fetch can id for the LED node");
        exit(-1);
    }

    ROS_INFO("LED Node Ready");

    r = 0;
    g = 0;
    b = 0;
    new_data = false;

    while(nh.ok()) {

        if(new_data) {
            hwctrl::CanFrame  msg;
            msg.can_id = can_id;
            msg.can_dlc =3;
            msg.data[0] = r;
            msg.data[1] = g;
            msg.data[2] = b;

            can_tx_pub.publish(msg);

            new_data = false;
        }

        ros::spinOnce();
        rate.sleep();
    }


}