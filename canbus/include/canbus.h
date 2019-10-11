#ifndef CANBUS_H_
#define CANBUS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <canbus/UWB_data.h>
#include <canbus/motor_data.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <vesc.h>

#define MAX_NUM_NODES 4
#define MAX_NUM_ANCHORS 3
#define MAX_CAN_TRIES 3
using namespace std;

using std::string;

const string id_header 				= "id";
const string type_header 			= "type";
const string x_header 				= "x";
const string y_header 				= "y";
const string anchor_str 			= "anchor";
const string node_str 				= "node";
const string node_config_fname 		= "/home/ros/ros_catkin_ws/src/canbus/include/node_config.csv";
const string can_config_fname 		= "/home/ros/ros_catkin_ws/src/canbus/include/can_config.csv";
const std::string device_type_hddr 	= "device_type";
const std::string can_id_hddr 		= "can_id";
int nNodes = 0;
int nAnchors = 0;

typedef canbus::UWB_data UWB_msg;
typedef canbus::motor_data motor_data_msg;

typedef struct {
	int id;
	string type;
	float x;
	float y;
} UwbNode;

typedef struct {
	uint8_t type;
	uint8_t anchor_id;
	float distance;
	uint16_t confidence;
} DistanceFrame;

typedef struct {
	string type;
	uint8_t can_id;
}CanDevice;

int get_nodes_from_file(string fname, string sType);

int read_can_config(string fname, std::vector<CanDevice> &devices);

#endif
