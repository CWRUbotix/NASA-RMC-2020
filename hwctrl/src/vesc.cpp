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
	frame.can_id = target_id | (CAN_PACKET_PROCESS_RX_BUFFER << 8) | CAN_EFF_FLAG;
	frame.data[i++] = self_id;
	frame.data[i++] = 0x01; // number of commands sent (we just sent 1)
	frame.data[i++] = (data_len >> 8) & 0xFF;
	frame.data[i++] = (data_len) & 0xFF;
	frame.data[i++] = (uint8_t)(crc >> 8);
	frame.data[i++] = (uint8_t)(crc & 0xFF);
	frame.can_dlc = i;
	tx_frame_queue.push_back(frame);
}

int send_packet(int sock, int target_id, int self_id, uint8_t* packet, int len){
	std::vector<struct can_frame> frames;
	generate_can_frames(target_id, self_id, packet, 5, frames);
	int nbytes = 0;
	for(auto frame = frames.begin(); frame != frames.end(); ++frame){
		nbytes = write(sock, &frame, sizeof(struct can_frame));
	}
	if(nbytes > 0){
		return 0;
	}else{
		return 1;
	}
}

int send_short_buf(int sock, int target_id, int self_id, int command){
	struct can_frame frame;
	frame.can_id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | target_id | CAN_EFF_FLAG;
	int ind = 0;
	frame.data[ind++] = self_id;
	frame.data[ind++] = 0x00;
	frame.data[ind++] = command;
	frame.can_dlc = ind;
	int nbytes = write(sock, &frame, sizeof(struct can_frame));
	if(nbytes > 0){
		return 0;
	}else{
		return 1;
	}
}

int send_short_buf(int sock, int target_id, int self_id, int command, uint8_t* data){
	struct can_frame frame;
	frame.can_id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | target_id | CAN_EFF_FLAG;
	int ind = 0;
	frame.data[ind++] = self_id;
	frame.data[ind++] = 0x00;
	frame.data[ind++] = command;
	frame.data[ind++] = data[0];
	frame.data[ind++] = data[1];
	frame.data[ind++] = data[2];
	frame.data[ind++] = data[3];

	frame.can_dlc = ind;
	int nbytes = write(sock, &frame, sizeof(struct can_frame));
	if(nbytes > 0){
		return 0;
	}else{
		return 1;
	}
}

int set_rpm_can(int sock, int target_id, int self_id, float rpm){
	int n_rpm = (int)rpm;
	struct can_frame frame;
	frame.can_id = (CAN_PACKET_SET_RPM << 8) | target_id | CAN_EFF_FLAG;

	// memcpy(frame.data, &n_rpm, sizeof(n_rpm));
	frame.data[0] = (uint8_t)(n_rpm >> 24);
	frame.data[1] = (uint8_t)(n_rpm >> 16);
	frame.data[2] = (uint8_t)(n_rpm >> 8);
	frame.data[3] = (uint8_t)(n_rpm & 0xFF);
	frame.can_dlc = sizeof(n_rpm);
	int nbytes = write(sock, &frame, sizeof(struct can_frame));

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

void handle_vesc_frame(struct can_frame frame, boost::shared_ptr<hwctrl::VescData>& motor_msg){

}

void fill_data_from_buffer(uint8_t* rx_buf, VescData* data){
	int ind = 1;
	data->temp_mos1 			= buffer_get_float16(rx_buf, 10.0, 	&ind);
	data->temp_mos2 			= buffer_get_float16(rx_buf, 10.0, 	&ind);
	data->current_motor 		= buffer_get_float32(rx_buf, 100.0, &ind);
	data->current_in 			= buffer_get_float32(rx_buf, 100.0, &ind);
	data->avg_id				= buffer_get_float32(rx_buf, 100.0, &ind);
	data->avg_iq				= buffer_get_float32(rx_buf, 100.0, &ind);
	data->duty_now 				= buffer_get_float16(rx_buf, 1000.0,&ind);
	data->rpm 					= buffer_get_float32(rx_buf, 1.0, 	&ind);
	data->v_in 					= buffer_get_float16(rx_buf, 10.0, 	&ind);
	data->amp_hours 			= buffer_get_float32(rx_buf, 10000.0, &ind);
	data->amp_hours_charged 	= buffer_get_float32(rx_buf, 10000.0, &ind);
	data->watt_hours 			= buffer_get_float32(rx_buf, 10000.0, &ind);
	data->watt_hours_charged 	= buffer_get_float32(rx_buf, 10000.0, &ind);
	data->tachometer 			= buffer_get_int32(  rx_buf, &ind);
	data->tachometer_abs 		= buffer_get_int32(  rx_buf, &ind);
	data->fault_code 			= (int8_t)rx_buf[ind++];
}

void fill_data_from_status_packet(uint8_t* rx_buf, VescData* data){
	int ind = 0;
	data->rpm 			= buffer_get_float32(frame_buf, 1.0, &ind);
	data->current_in 	= buffer_get_float16(frame_buf, 10.0, &ind);
	data->duty_now 		= buffer_get_float16(frame_buf, 1000.0, &ind);

bool VescCan::set_vesc_callback(canbus::SetVescCmd::Request& request, canbus::SetVescCmd::Response& response){
	// ROS_INFO("Setting VESC %d to %f eRPM",request.can_id, request.e_rpm);
	int retval = set_rpm_can(this->can_sock, request.can_id, this->self_can_id, request.e_rpm);
	if(retval == 0){
		response.status = 0;
		response.timestamp = ros::Time::now();
		return true;
	}else{
		response.status = 2;
		return false;
	}
}
VescCan::VescCan(int s, int id){
	can_sock = s;
	self_can_id = id;
}



// void fill_msg_from_buffer(uint8_t* rx_buf, boost::shared_ptr<hwctrl::VescData>& motor_msg){
// 	int ind = 1;
// 	motor_msg->temp_mos1 			= buffer_get_float16(rx_buf, 10.0, 	&ind);
// 	motor_msg->temp_mos2 			= buffer_get_float16(rx_buf, 10.0, 	&ind);
// 	motor_msg->current_motor 		= buffer_get_float32(rx_buf, 100.0, &ind);
// 	motor_msg->current_in 			= buffer_get_float32(rx_buf, 100.0, &ind);
// 	motor_msg->avg_id				= buffer_get_float32(rx_buf, 100.0, &ind);
// 	motor_msg->avg_iq				= buffer_get_float32(rx_buf, 100.0, &ind);
// 	motor_msg->duty_now 			= buffer_get_float16(rx_buf, 1000.0,&ind);
// 	motor_msg->rpm 					= buffer_get_float32(rx_buf, 1.0, 	&ind);
// 	motor_msg->v_in 				= buffer_get_float16(rx_buf, 10.0, 	&ind);
// 	motor_msg->amp_hours 			= buffer_get_float32(rx_buf, 10000.0, &ind);
// 	motor_msg->amp_hours_charged 	= buffer_get_float32(rx_buf, 10000.0, &ind);
// 	motor_msg->watt_hours 			= buffer_get_float32(rx_buf, 10000.0, &ind);
// 	motor_msg->watt_hours_charged 	= buffer_get_float32(rx_buf, 10000.0, &ind);
// 	motor_msg->tachometer 			= buffer_get_int32(  rx_buf, &ind);
// 	motor_msg->tachometer_abs 		= buffer_get_int32(  rx_buf, &ind);
// 	motor_msg->fault_code 			= (int8_t)rx_buf[ind++];
// }

// void fill_msg_from_status_packet(uint8_t* frame_buf, boost::shared_ptr<canbus::VescData>& motor_msg){
// 	int ind = 0;
// 	motor_msg->rpm 			= buffer_get_float32(frame_buf, 1.0, &ind);
// 	motor_msg->current_in 	= buffer_get_float16(frame_buf, 10.0, &ind);
// 	motor_msg->duty_now 	= buffer_get_float16(frame_buf, 1000.0, &ind);
// }
