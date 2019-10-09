#include <vesc.h>

/**
 * takes a vesc comm packet and fills a vector with can frames
 */
void generate_can_frames(int target_id, uint8_t* data, int data_len, std::vector<struct can_frame> &tx_frame_queue){
	// MAKE THE PACKET
	int packet_len = data_len + 4; // length + header_length + chksum_len + stop_byte
	if(len <= 256){
		packet_len += 1; // plus 1 for data_length
	}else{
		packet_len += 2; // plus 2 for data_length 
	}

	uint8_t packet[packet_len] = {};

	int ind = 0;

	if(len <= 256){
		packet[ind++] = 2; // short packet
		packet[ind++] = (uint8_t)(len & 0xff);
	}else{
		packet[ind++] = 3; // long packet
		packet[ind++] = (uint8_t)(len >> 8);
		packet[ind++] = (uint8_t)(len & 0xFF);
	}

	for(int i = 0; i < data_len; i++){
		packet[ind++] = data[i];
	}

	uint16_t crc = crc16(data, data_len);
	packet[ind++] = (uint8_t)(crc >> 8);
	packet[ind++] = (uint8_t)(crc & 0xFF);
	packet[ind++] = 0x03; // stop byte

	// MAKE THE CAN FRAMES
	int ind			= 0;
	int frame_ind 	= 0;
	int copy_len 	= 0;
	while(ind < packet_len){
		struct can_frame frame;
		frame.can_id = target_id | (CAN_PACKET_FILL_RX_BUFFER << 8);
		frame.data[0] = ind;
		if((packet_len - ind) >= 7){
			copy_len = 7;
		}else{
			copy_len = packet_len - ind;
		}
		memcpy(frame.data+1, buf+ind, copy_len);
		ind += copy_len;
		frame.can_dlc = copy_len + 1; // copy_len + buffer_position
		tx_frame_queue.push_back(frame);
	}
	// now add a command to tell the vesc to process the buffer
	struct can_frame frame;
	frame.can_id = target_id | (CAN_PACKET_PROCESS_RX_BUFFER << 8);
	frame.can_dlc = 0;
	tx_frame_queue.push_back(frame);
}

int send_packet(int sock, uint8_t* packet, int len){
	std::vector<struct can_frame> frames;
	generate_can_frames(target_id, buffer, 5, frames);
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

int set_rpm(int sock, int target_id, float rpm){
	uint8_t buffer[5] = {};
	buffer[0] = COMM_SET_RPM;
	int n_rpm = (int)rpm;
	memcpy(buffer+1, &n_rpm, 4);
	int status = send_packet(sock, buffer, 5);
	return status;
}

void get_values(int sock, int target_id){
	uint8_t buffer[1] = {COMM_GET_VALUES};
	return send_packet(sock, buffer, 1);
}