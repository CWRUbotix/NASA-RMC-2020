#ifndef CLIENT_COMMS_H_
#define CLIENT_COMMS_H_


#include "values_and_types.h"
#include <string.h>
#include <inttypes.h>

FAULT_T read_from_client(void);
void reply_to_client(FAULT_T fault);
uint16_t checksum(uint8_t* data, int length);

uint8_t cmd[CMD_MAX_LEN] = {};
uint8_t rpy[RPY_MAX_LEN] = {};

uint8_t* cmd_body;
int cmd_type 			= 0;
int cmd_body_len 		= 0;

uint8_t* rpy_body;
int rpy_type 			= 0;
int rpy_body_len 		= 0;
uint8_t test_data[4] 	= {}; 		// to hold the test bytes sent in a test exchange

FAULT_T read_from_client(){
	int index = 0;
	while(CLIENT.available() > 0){
		cmd[index++] = CLIENT.read();
	}
	// index now equals body_length + header_length

	cmd_body 		= cmd + HEADER_LEN;
	cmd_type 		= cmd[0];
	cmd_body_len 	= (cmd[1] << 8) + cmd[2];
	uint16_t chksum = (cmd[3] << 8) + cmd[4];

	if((cmd_type != CMD_SET_OUTPUTS) && 
		(cmd_type != CMD_READ_VALUES) && 
		(cmd_type != CMD_TEST) &&
		(cmd_type != CMD_T_SYNC))
	{
		return INVALID_CMD;
	}else if((index - HEADER_LEN) != cmd_body_len){
		return LEN_MISMATCH;
	}else if(chksum != checksum(cmd_body, index - HEADER_LEN)){
		return CHKSUM_MISMATCH;
	}else{
		//return NO_FAULT;
	}

	switch(cmd_type){
		case CMD_SET_OUTPUTS:{
			int id = 0;
			float value = 0.0;
			MotorInfo* motor = NULL;
			MotorType mtr_type;
			for(int i = 0; i < cmd_body_len; i+=CMD_BLOCK_LEN){
				value = 0.0;
				id = cmd_body[i];
				memcpy(&value, cmd_body + i+1, 4); 	// copy bytes from cmd_body to value
				debug("MOTOR: " + String(value, 3));
				motor = &motor_infos[id];
				motor->last_setpt = motor->setpt;
				motor->setpt = value;
				if(id == EXC_ROT_PORT){
					motor_infos[EXC_ROT_STBD].setpt = motor->setpt;
				}else if(id == EXC_ROT_STBD){
					motor->setpt = motor_infos[EXC_ROT_PORT].setpt;
				}
			}
			break;}
		case CMD_READ_VALUES:{
			// really nothing to do here
			break;}
		case CMD_TEST:{
			test_data[0] = cmd_body[0];
			test_data[1] = cmd_body[1];
			test_data[2] = cmd_body[2];
			test_data[3] = cmd_body[3];
			break;}
		case CMD_T_SYNC:{
			int value = (cmd_body[0] << 24) | (cmd_body[1] << 16) | (cmd_body[2] << 8) | (cmd_body[3]);
			t_offset = micros() - value;
			break;}
	}
	return NO_FAULT;
}

void reply_to_client(FAULT_T fault){
	switch(fault){
		case NO_FAULT: 			rpy_type = cmd_type; break;
		case INVALID_CMD: 		rpy_type = RPY_INVALID_CMD; break;
		case LEN_MISMATCH: 		rpy_type = RPY_LEN_MISMATCH; break;
		case CHKSUM_MISMATCH: 	rpy_type = RPY_CHKSUM_MISMATCH; break;
	}

	rpy[0] 		= rpy_type & 0xFF;
	rpy_body  	= rpy + HEADER_LEN;

	switch (rpy_type){
		case RPY_SET_OUTPUTS:{
			rpy_body_len = cmd_body_len;
			int id = 0;
			float value = 0.0;
			MotorInfo* motor;
			for(int i = 0; i < cmd_body_len; i+=RPY_BLOCK_LEN){
				id = cmd_body[i];
				motor = &motor_infos[id];
				value = motor->setpt;
				rpy_body[i] 	= id;
				memcpy(rpy_body + i+1, &value, 4);
			}
			uint16_t chksum = checksum(rpy_body, rpy_body_len);

			rpy[1] = (rpy_body_len >> 8) & 0xFF;
			rpy[2] = (rpy_body_len) & 0xFF;
			rpy[3] = (chksum >> 8) & 0xFF;
			rpy[4] = (chksum) & 0xFF;
			break;}
		case RPY_READ_VALUES:{
			rpy_body_len = cmd_body_len * RPY_BLOCK_LEN_L; 	// each 1 byte now needs 9 bytes
			int id = 0;
			int rpy_ind = 0;
			float value = 0.0;
			int t_stamp = 0;
			SensorInfo* sensor;
			for(int i = 0; i < cmd_body_len; i++){
				rpy_ind = i*RPY_BLOCK_LEN_L;
				id = cmd_body[i];
				sensor = &sensor_infos[id];
				value = sensor->value;
				t_stamp = sensor->t_stamp;
				rpy_body[rpy_ind++] = id;
				memcpy(rpy_body + rpy_ind, &value, 4);
				rpy_ind += 4;
				memcpy(rpy_body + rpy_ind, &t_stamp, 4);
			}
			uint16_t chksum = checksum(rpy_body, rpy_body_len);

			rpy[1] = (rpy_body_len >> 8) & 0xFF;
			rpy[2] = (rpy_body_len) & 0xFF;
			rpy[3] = (chksum >> 8) & 0xFF;
			rpy[4] = (chksum) & 0xFF;
			break;}
		case RPY_T_SYNC:{
			rpy_body_len = 4;
			int value = TIME_STAMP;
			rpy_body[0] = value << 24;
			rpy_body[1] = value << 16;
			rpy_body[2] = value << 8;
			rpy_body[3] = value;
			uint16_t chksum = checksum(rpy_body, rpy_body_len);

			rpy[1] = (rpy_body_len >> 8) & 0xFF;
			rpy[2] = (rpy_body_len) & 0xFF;
			rpy[3] = (chksum >> 8) & 0xFF;
			rpy[4] = (chksum) & 0xFF;
			break;}
		case RPY_TEST:{
			rpy_body_len = 4;

			rpy_body[0] = cmd_body[0];
			rpy_body[1] = cmd_body[1];
			rpy_body[2] = cmd_body[2];
			rpy_body[3] = cmd_body[3];
			uint16_t chksum = checksum(rpy_body, rpy_body_len);

			rpy[1] = (rpy_body_len >> 8) & 0xFF;
			rpy[2] = (rpy_body_len) & 0xFF;
			rpy[3] = (chksum >> 8) & 0xFF;
			rpy[4] = (chksum) & 0xFF;

			break;}
		case RPY_INVALID_CMD:{
			rpy[1] = 0x00;
			rpy[2] = 0x00;
			rpy[3] = 0x00;
			rpy[4] = 0x00;
			break;}
		case RPY_LEN_MISMATCH:{
			rpy[1] = 0x00;
			rpy[2] = 0x00;
			rpy[3] = 0x00;
			rpy[4] = 0x00;
			break;}
		case RPY_CHKSUM_MISMATCH:{
			rpy[1] = 0x00;
			rpy[2] = 0x00;
			rpy[3] = 0x00;
			rpy[4] = 0x00;
			break;}
	}
	
	CLIENT.write(rpy, rpy_body_len + HEADER_LEN);

}

uint16_t checksum(uint8_t* data, int length){
	uint16_t chksum = 0;
	for(int i = 0; i<length ; i++){
		chksum += data[i]; 
	}
	return (int16_t)chksum;
}



#endif