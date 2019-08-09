#ifndef CANBUS_H_
#define CANBUS_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <canbus/UWB_data.h>
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
#include <string>

#define MAX_NUM_NODES 4
#define MAX_NUM_ANCHORS 3

using namespace std;

using std::string;

static const string id_header 		= "id";
static const string type_header 	= "type";
static const string x_header 		= "x";
static const string y_header 		= "y";
static const string anchor_str 		= "anchor";
static const string node_str 		= "node";
static const string node_config_fname = "/home/ros/ros_catkin_ws/src/canbus/include/node_config.csv";
int nNodes = 0;
int nAnchors = 0;

typedef canbus::UWB_data UWB_msg;

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

UwbNode* get_nodes_from_file(string fname, string sType, int* len, int max_len);

#endif
