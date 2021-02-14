#include "ros/ros.h"
#include "hwctrl/CanFrame.h"
#include "hwctrl/LED.h"

int r, g, b;

bool receiveColors(hwctrl::LED::Request &req , hwctrl::LED::Response &res )
{
    ROS_INFO("reqest: r=%ld, g = %ld, b = %ld",(long int) req.r, (long int) req.g, (long int) req.b);
    r = req.r;
    g = req.g;
    b = req.b;
    return true;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "led_node");
     ros::NodeHandle n;
     ros::Publisher m_can_rx_pub = n.advertise<hwctrl::CanFrame>("can_frames_tx", 2);
     ros::Rate loop_rate(10);
     ros::ServiceServer service = n.advertiseService("led_node_service", receiveColors);
     ROS_INFO("Ready to recieve led colors");

    while(ros::ok())
    {
        hwctrl::CanFrame msg;
        msg.can_id = 10;
        msg.can_dlc= 3;

        msg.data[0] = r;
        msg.data[1] = g;
        msg.data[2] = b;
        m_can_rx_pub.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;

 }


