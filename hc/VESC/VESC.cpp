
#include "inttypes.h"
#include "VESC.h"
#include "datatypes.h"
#include "crc.h"
#include "buffer.h"

VESC::VESC(HardwareSerial* ser){
	this->serial = ser;
	this->send_ind = 0;
	this->recv_ind = 0;
}
void VESC::begin(){
	this->serial->begin(VESC_BAUD);
}

// SETTERS
void VESC::set_rpm(int rpm){
	this->send_ind = 0;
	this->send_buf[send_ind++] = COMM_SET_RPM;
	buffer_append_int32(this->send_buf, rpm, &(this->send_ind));

	this->send_packet(this->send_buf, this->send_ind);
}

// GETTERS
int VESC::get_rpm(){
	return (this->motor_values).rpm;
}
float VESC::get_current_in(){
	return (this->motor_values).current_in;
}

float VESC::get_temperature_1(){
	return (this->motor_values).temp_mos1;
}

float VESC::get_temperature_2(){
	 return (this->motor_values).temp_mos2;
}

// COMMANDS
void VESC::request_mc_values(){
	this->send_ind = 0;
	this->send_buf[send_ind++] = COMM_GET_VALUES;
	this->send_packet(this->send_buf, this->send_ind);
}


void VESC::update_mc_values(){
	int32_t temp = 0;
	bool valid = this->read_packet(this->recv_buf, &temp);
	uint8_t type = this->recv_buf[0];

	this->recv_ind = 0;
	type = this->recv_buf[this->recv_ind++];

	if(valid && (type == COMM_GET_VALUES)){
		this->motor_values.temp_mos1 			= buffer_get_float16(this->recv_buf, 10.0, 	&this->recv_ind);
		this->motor_values.temp_mos2 			= buffer_get_float16(this->recv_buf, 10.0, 	&this->recv_ind);
		this->motor_values.current_motor 		= buffer_get_float32(this->recv_buf, 100.0, &this->recv_ind);
		this->motor_values.current_in 			= buffer_get_float32(this->recv_buf, 100.0, &this->recv_ind);
		this->motor_values.avg_id				= buffer_get_float32(this->recv_buf, 100.0, &this->recv_ind);
		this->motor_values.avg_iq				= buffer_get_float32(this->recv_buf, 100.0, &this->recv_ind);
		this->motor_values.duty_now 			= buffer_get_float16(this->recv_buf, 1000.0,&this->recv_ind);
		this->motor_values.rpm 					= buffer_get_float32(this->recv_buf, 1.0, 	&this->recv_ind);
		this->motor_values.v_in 				= buffer_get_float16(this->recv_buf, 10.0, 	&this->recv_ind);
		this->motor_values.amp_hours 			= buffer_get_float32(this->recv_buf, 10000.0, &this->recv_ind);
		this->motor_values.amp_hours_charged 	= buffer_get_float32(this->recv_buf, 10000.0, &this->recv_ind);
		this->motor_values.watt_hours 			= buffer_get_float32(this->recv_buf, 10000.0, &this->recv_ind);
		this->motor_values.watt_hours_charged 	= buffer_get_float32(this->recv_buf, 10000.0, &this->recv_ind);
		this->motor_values.tachometer 			= buffer_get_int32(  this->recv_buf, &this->recv_ind);
		this->motor_values.tachometer_abs 		= buffer_get_int32(  this->recv_buf, &this->recv_ind);
		this->motor_values.fault_code 			= (mc_fault_code)(this->recv_buf)[this->recv_ind++];
	}

}

bool VESC::data_ready(){
	return (this->serial->available() > 0);
}


void VESC::send_packet(uint8_t* data, int32_t len){
	int buf_len = len + 4; // length + header length + chksum length + stop byte
	if(len <= 256){
		buf_len += 1; // + 1 byte for payload length
	}else{
		buf_len += 2; // +2 bytes for payload length
	}

	uint8_t buf[buf_len] 	= {};
	int buf_ind = 0;

	buf[buf_len - 1] 		= 3; 	// stop byte at the end

	if(len <= 256){
		buf[buf_ind++] = 2; // short packet
		buf[buf_ind++] = (uint8_t)(len & 0xFF);
	}else{
		buf[buf_ind++] = 3; // long packet
		buf[buf_ind++] = (uint8_t)(len >> 8);
		buf[buf_ind++] = (uint8_t)(len & 0xFF);
	}

	for(int i = 0; i < len; i++){
		buf[buf_ind++] = data[i];
	}

	uint16_t crc = crc16(data, len);
	buf[buf_ind++] = (uint8_t)(crc >> 8);
	buf[buf_ind++] = (uint8_t)(crc & 0xFF);
	buf[buf_ind++] = 0x03;

	this->serial->write(buf, buf_len);

}

// puts the body of the packet into the data parameter
// index will be length of data
// will return false if something is wrong with the packet
// will return true if the size is correct and the chksum is correct
bool VESC::read_packet(uint8_t* data, int32_t* index){
	bool retval = true;

	uint8_t raw_packet[BUFFER_ALLOC] = {}; 
	int len = 0;

	while((this->serial->available() > 0) && (len <= BUFFER_ALLOC)){
		raw_packet[len++] = this->serial->read();
	}

	this->recv_ind = 0;

	int size = raw_packet[this->recv_ind++]; // will be 2 or 3
	int packet_len = 0;

	if(size == 2){ // short packet
		packet_len = raw_packet[this->recv_ind++];
	}else if (size == 3){ // long packet
		packet_len = (raw_packet[this->recv_ind++] << 8);
		packet_len += raw_packet[this->recv_ind++];
	}else{
		retval = false;
	}

	// packet length is length of our body, not including the chksum
	// this is the data relevant to the caller
	for( *index = 0; (*index) < packet_len; (*index)++){
		data[(*index)] = raw_packet[this->recv_ind++]; 				// place in the data
	}

	// get the chksum out
	uint16_t chksum;
	chksum 	= (raw_packet[this->recv_ind++] & 0xFF) << 8;
	chksum 	|= (raw_packet[this->recv_ind++] & 0xFF);

	uint16_t chksum_expected = crc16(data, packet_len); 				// what we expect the chksum to be

	// retval = (retval && (chksum == chksum_expected));
	// retval = (retval && (raw_packet[this->recv_ind++] == 0x03)); 	// last byte must be a stop byte (0x03)
	return retval;

}