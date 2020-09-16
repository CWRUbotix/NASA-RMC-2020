#ifndef SABERTOOTH_H_
#define SABERTOOTH_H_

#include <ros/ros.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <termios.h>

#define SBTH_MOTOR_1_FORE 	0x00
#define SBTH_MOTOR_1_BACK 	0x01
#define SBTH_MIN_VOLTAGE 	0x02
#define SBTH_MAX_VOLTAGE 	0x03
#define SBTH_MOTOR_2_FORE 	0x04
#define SBTH_MOTOR_2_BACK 	0x05
#define SBTH_MOTOR_1_7_BIT 	0x06
#define SBTH_MOTOR_2_7_BIT 	0x07

#define SBTH_SERIAL_TIMEOUT 0x0E
#define SBTH_BAUD_RATE 		0x0F
#define SBTH_RAMPING 		0x10
#define SBTH_DEADBAND 		0x11

int sbth_send_packet(std::string name, uint8_t * packet);

uint8_t sbth_chksum(uint8_t * packet);

#endif