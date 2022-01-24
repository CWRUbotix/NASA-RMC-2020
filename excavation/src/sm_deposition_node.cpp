#include <sm_deposition/sm_deposition.h>

//#include "ros/ros.h"

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_deposition");
    ros::NodeHandle nh;

    smacc::run<sm_depostion::SmDepostion>();
}