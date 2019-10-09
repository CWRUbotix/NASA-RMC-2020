#include <vesc.h>

/**
 * takes a vesc comm packet and fills a vector with can frames
 */
void generate_can_frames(int target_id, int self_id, uint8_t* data, int data_len, std::vector<struct can_frame> &tx_frame_queue){
	// MAKE THE CAN FRAMES
	int ind			= 0;
	int frame_ind 	= 0;
	int copy_len 	= 0;
	while(ind < data_len){
		struct can_frame frame;
		frame.can_id = target_id | (CAN_PACKET_FILL_RX_BUFFER << 8);
		frame.data[0] = ind;
		if((data_len - ind) >= 7){
			copy_len = 7;
		}else{
			copy_len = data_len - ind;
		}
		memcpy(frame.data+1, data+ind, copy_len);
		ind += copy_len;
		frame.can_dlc = copy_len + 1; // copy_len + buffer_position
		tx_frame_queue.push_back(frame);
	}
	uint16_t crc = crc16(data, data_len); 	// get our crc

	// now add a command to tell the vesc to process the buffer
	struct can_frame frame;
	int i = 0;
	frame.can_id = target_id | (CAN_PACKET_PROCESS_RX_BUFFER << 8);
	frame.data[i++] = self_id;
	frame.data[i++] = 0x01; // number of commands sent (we just sent 1)
	frame.data[i++] = (data_len >> 8) & 0xFF;
	frame.data[i++] = (data_len) & 0xFF;
	frame.data[i++] = (uint8_t)(crc >> 8);
	frame.data[i++] = (uint8_t)(crc & 0xFF);
	frame.can_dlc = i;
	tx_frame_queue.push_back(frame);
}

int send_packet(int sock, int self_id, uint8_t* packet, int len){
	std::vector<struct can_frame> frames;
	generate_can_frames(target_id, self_id, buffer, 5, frames);
	int nbytes = 0;
	for(auto frame = frames.begin(); i != frames.end(); ++i){
		nbytes = write(sock, frame, sizeof(struct can_frame));
	}
	if(nbytes > 0){
		return 0;
	}else{
		return 1;
	}
}

int send_short_buf(int sock, int target_id, int self_id, int command){
	struct can_frame frame;
	frame.can_id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | target_id;
	int ind = 0;
	frame.data[ind++] = self_id;
	frame.data[ind++] = 0x00;
	frame.data[ind++] = command;
	frame.can_dlc = ind;
	int nbytes = write(sock, frame, sizeof(struct can_frame));
	if(nbytes > 0){
		return 0;
	}else{
		return 1;
	}
}

int send_short_buf(int sock, int target_id, int self_id, int command, uint8_t* data){
	struct can_frame frame;
	frame.can_id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | target_id;
	int ind = 0;
	frame.data[ind++] = self_id;
	frame.data[ind++] = 0x00;
	frame.data[ind++] = command;
	memcpy(frame.data+ind, data, 4);
	ind += 4;
	frame.can_dlc = ind;
	int nbytes = write(sock, frame, sizeof(struct can_frame));
	if(nbytes > 0){
		return 0;
	}else{
		return 1;
	}
}


int set_rpm(int sock, int target_id, int self_id, float rpm){
	uint8_t data[4] = {};
	int n_rpm = (int)rpm;
	memcpy(data, &n_rpm, 4);
	return send_short_buf(sock, target_id, self_id, COMM_SET_RPM, data);
}

int get_values(int sock, int target_id, int self_id){
	return send_short_buf(sock, target_id, self_id, COMM_GET_VALUES);
}

void handle_vesc_frame(struct can_frame frame, std::vector<canbus::motor_data> &motor_msgs){
	
}

void parse_motor_frames(std::vector<struct can_frame> &frames, std::vector<canbus::motor_data> &motor_msgs){
	uint8_t rx_buffer[1024];
	for(auto frame = frames.begin(); frame != frames.end(); ++frame){
		
	}
	// motor_msg.timestamp = ros::Time::now();
	// int8_t vesc_id = (rx_id & 0xFF);
	// int vesc_cmd = (rx_id >> 8);
	// motor_msg.motor_type 	= "VESC";
	// motor_msg.can_id 		= vesc_id;
	// switch(vesc_cmd){
	// 	case(CAN_PACKET_STATUS):{
	// 		// routine status message
	// 		int32_t rpm;
	// 		int16_t current;
	// 		int16_t duty_cycle;
	// 		memcpy(&rpm, rx_frame.data, 4);
	// 		memcpy(&current, rx_frame.data + 4, 2);
	// 		memcpy(&duty_cycle, rx_frame.data + 6, 2);
	// 		motor_msg.raw_rpm = (float)rpm;
	// 		motor_msg.current = (float)current;
	// 		motor_msg.duty_cycle = (float)duty_cycle;
	// 		break;}
	// }
	// motor_data.publish(motor_msg);
}