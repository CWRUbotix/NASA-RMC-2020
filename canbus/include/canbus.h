#ifndef CANBUS_H_
#define CANBUS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <canbus/UWB_data.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <inttypes.h>

typedef canbus::UWB_data UWB_msg;

using std::string;

typedef struct {
	uint8_t type;
	uint8_t anchor_id;
	float distance;
	uint16_t confidence;
} DistanceFrame;

#endif
