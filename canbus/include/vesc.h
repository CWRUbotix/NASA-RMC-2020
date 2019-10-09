#ifndef VESC_H_
#define VESC_H_

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
#include <datatypes.h>
#include <crc.h>
#include <buffer.h>

/**
 * takes a vesc communication buffer and fills a vector with valid can frames ready to be sent
 */
void generate_can_frames(int target_id, uint8_t* buf, int len, std::vector<struct can_frame> &tx_frame_queue);

int send_packet(int sock, uint8_t* packet, int len);

int set_rpm(int sock, int target_id, float rpm);

int get_values(int sock, int target_id);

#endif