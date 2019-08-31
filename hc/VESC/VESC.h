#ifndef _VESC_H_
#define _VESC_H_

#include <inttypes.h>
#include <datatypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define BUFFER_ALLOC 	1024
#define VESC_BAUD 		115200


class VESC{
private:
	int32_t send_ind;
	int32_t recv_ind;
	uint8_t send_buf[BUFFER_ALLOC] = {};
	uint8_t recv_buf[BUFFER_ALLOC] = {};
	void send_packet(uint8_t* data, int32_t len);
	bool read_packet(uint8_t* data, int32_t* len);
	HardwareSerial* serial;
	mc_values motor_values;

public:
	VESC(HardwareSerial* ser);
	void begin();
	bool data_ready();
	void request_mc_values();
	void update_mc_values();
	void set_rpm(int rpm);
	int  get_rpm();
	float get_current_in();
	float get_temperature_1();
	float get_temperature_2();
};

#endif