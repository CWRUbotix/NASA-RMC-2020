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
void generate_can_frames(int target_id, int self_id, uint8_t* data, int data_len, std::vector<struct can_frame> &tx_frame_queue);

/**
 * send a short buffer to VESC with target ID with specified command and no data
 */
int send_short_buf(int sock, int target_id, int self_id, int command);

/**
 * send a short buffer to VESC with target ID, specified command, and data
 * data is assumed to be 4 bytes
 */
int send_short_buf(int sock, int target_id, int self_id, int command, uint8_t* data);

/**
 * Send an arbitrary VESC comm packet
 */
int send_packet(int sock, int target_id, int self_id, uint8_t* packet, int len);

/**
 * Sets the RPM of VESC with target_id using short buffer command
 */
int set_rpm(int sock, int target_id, int self_id, float rpm);

/**
 * Sends COMM_GET_VALUES command to VESC with target_id
 */
int get_values(int sock, int target_id, int self_id);


void parse_motor_frames(std::vector<struct can_frame> &frames, std::vector<canbus::motor_data> &motor_msgs);

void fill_msg_from_buffer(uint8_t* vesc_rx_buf, canbus::motor_data &motor_msg);

void fill_msg_from_status_packet(uint8_t* frame_buf, canbus::motor_data &motor_msg);

#endif
