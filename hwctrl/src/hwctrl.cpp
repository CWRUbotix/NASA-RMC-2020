#include <hwctrl.h>

////////////////////////////////////////////////////////////////////////////////
// MOTOR INTERFACE STUFF
////////////////////////////////////////////////////////////////////////////////
HwMotorIf::HwMotorIf(ros::NodeHandle n)
 : loop_rate(1000) {
	this->nh 				= n; 	// store a copy of the node handle passed by value

	this->get_motors_from_csv(config_file_path);

	// PUBLISHERS
	this->can_tx_pub 		= this->nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
	this->motor_data_pub 	= this->nh.advertise<hwctrl::MotorData>("motor_data", 128);

	// SUBSCRIBERS
	this->can_rx_sub 		= this->nh.subscribe("can_frames_rx", 128, &HwMotorIf::can_rx_callback, this);
	this->sensor_data_sub 	= this->nh.subscribe("sensor_data", 128, &HwMotorIf::sensor_data_callback, this);
	this->limit_sw_sub 		= this->nh.subscribe("limit_switch_states", 128, &HwMotorIf::limit_sw_callback, this);

	// SET MOTOR SERVICE
	this->set_motor_srv 	= this->nh.advertiseService("set_motor", &HwMotorIf::set_motor_callback, this);
}

// ===== SUBSCRIBER CALLBACKS =====
/**
 * any CAN data of interest is from VESC's
 */
void HwMotorIf::can_rx_callback(const boost::shared_ptr<hwctrl::CanFrame>& frame){
	uint32_t rx_id = (uint32_t)frame->can_id;

	if((rx_id & ~0xFF) != 0){
		// we can assume this is a VESC message
		uint8_t cmd = (uint8_t)(rx_id >> 8);
		int8_t id 	= (int8_t)(rx_id & 0xFF);
		uint8_t* frame_data = frame->data.data(); // get the back-buffer of the vector
		ROS_INFO("VESC %d sent msg type %d", id, cmd);
		HwMotor* vesc = this->get_vesc_from_can_id(id);
		switch(cmd){
			case CAN_PACKET_STATUS:{
				if(vesc == NULL){
					break;
				}
				VescData* vesc_data = &(vesc->vesc_data);

				// store a ROS timestamp
				vesc_data->timestamp = ros::Time::now();

				// store data with the correct vesc object
				fill_data_from_status_packet(frame_data, vesc_data);
				break;
			}
			case CAN_PACKET_FILL_RX_BUFFER:{
				if(id != 0x00){
						// this is not intended for us (our canID is 0)
					break;
				}
				// copy the CAN data into the corresponding vesc rx buffer
				memcpy(vesc->vesc_data.vesc_rx_buf + frame_data[0], frame_data + 1, frame->can_dlc - 1);
				break;
			}
			case CAN_PACKET_PROCESS_RX_BUFFER:{
				if(id != 0x00){
					// not intended for us
					break;
				}
				if(vesc == NULL){
					// we know of no VESC with the given CAN ID
					break;
				}
				int ind = 0;
				uint16_t packet_len;
				uint16_t crc;
				int vesc_id = frame_data[ind++]; // which vesc sent the data
				int n_cmds 	= frame_data[ind++]; // how many commands
				packet_len 	= (frame_data[ind++] << 8);
				packet_len 	|= frame_data[ind++];
				crc 		= (frame_data[ind++] << 8);
				crc 		|= frame_data[ind++];

				uint16_t chk_crc = crc16(vesc->vesc_data.vesc_rx_buf, packet_len);
				// ROS_INFO("PROCESSING RX BUFFER\nPacket Len:\t%d\nRcvd CRC:\t%x\nComputed CRC:\t%x", packet_len, crc, chk_crc);

				if(crc != chk_crc){
					ROS_INFO("Error: Checksum doesn't match");
					// error in transmission
					break;
				}

				ind = 0;
				int comm_cmd = vesc->vesc_data.vesc_rx_buf[ind++];
				switch(comm_cmd){
					case COMM_GET_VALUES:{
						vesc->vesc_data.timestamp 	= ros::Time::now();
						vesc->vesc_data.motor_type 	= "vesc";
						vesc->vesc_data.can_id 		  = vesc_id;

						fill_data_from_buffer(vesc->vesc_data.vesc_rx_buf, &(vesc->vesc_data));

						break;}
					}

				break;
			}
				// indicates the end of the cmd buffer
		}
	}else{
		ROS_INFO("Not a vesc message");
	}
}

void HwMotorIf::sensor_data_callback(const hwctrl::SensorData& data){
	// determine if this sensor is useful to us;
}

void HwMotorIf::limit_sw_callback(const boost::shared_ptr<hwctrl::LimitSwState>& state){
	// figure out which motor this corresponds to and stop it!
}

/**
 * @param can_id the can id of the VESC we're looking for
 * @return a pointer to the VESC with this CAN ID (null if not found)
 */
HwMotor* HwMotorIf::get_vesc_from_can_id(int can_id){
	HwMotor* motor_arr = this->motors.data();
	int size = this->motors.size();
	int i = 0;
	bool found = false;
	while(!found && i < size){
		found = (motor_arr[i].motor_type == DEVICE_VESC) && (motor_arr[i].device_id == can_id);
		i++;
	}
	if(found){
		return &(motor_arr[i-1]);
	}else{
		return NULL;
	}
}


void maintain_motors_thread(HwMotorIf* motor_if){
	while(ros::ok()){
		motor_if->maintain_next_motor();
		this->loop_rate.sleep();
	}
}

void HwMotorIf::maintain_next_motor(){
	HwMotor* motors = this->motors.data();
	HwMotor* motor = &(motors[this->motor_ind]);
	// HwMotor* motor = &(this->motors.at(this->motor_ind));
	// ROS_INFO("Maintaining motor %s", motor->name.c_str());
	switch(motor->motor_type){
		case(DEVICE_NONE):break;
		case(DEVICE_VESC):{
			// ROS_INFO("Index: %d, Setpoint: %f", this->motor_ind, motor->setpoint);
			if(motor->online){
				// figure out next velocity setpoint based on acceleration & last sent RPM
				float delta 	=  motor->setpoint - motor->last_setpoint;
				float dt 		= (float)(ros::Time::now().toSec() - motor->update_t.toSec());
				float accel 	= fabs(delta/dt); // proposed acceleration

				// if proposed acceleration is higher than requested accleration, accelerate only at requested acceleration.
				// else the delta we calculated is ok
				if(accel > fabs(motor->accel_setpoint)){
					if(delta > 0){
						delta = fabs(motor->accel_setpoint) * dt; // new delta (is positive)
					}else{
						delta = -1.0 * fabs(motor->accel_setpoint) * dt; // new delta (needs to be negative)
					}
				}
				motor->last_setpoint = motor->last_setpoint + delta; // the new setpoint based on delta
			}else{
				ROS_INFO("Motor %d offline",motor->id);
			}

			float eRPM = motor->rpm_coef * motor->last_setpoint;
			canbus::SetVescCmd cmd;
			cmd.request.can_id 	= motor->device_id; // device_id should be the can_id
			cmd.request.e_rpm 	= eRPM;
			if(this->vesc_client.call(cmd)){
				// presumed success
				// ROS_INFO("VESC Service call successful");
				if(cmd.response.status == 0){
					motor->update_t = cmd.response.timestamp;
					motor->online = true;
				}else{
					ROS_INFO("Motor offline");
				}
			}else{
				ROS_INFO("Set VESC service call failed");
				motor->online = false;
			}
			break;}
		case(DEVICE_SABERTOOTH):{

			break;}
	}

	this->motor_ind ++;
	if(this->motor_ind >= this->motors.size()){
		this->motor_ind = 0;
	}
}


bool HwMotorIf::set_motor_callback(hwctrl::SetMotor::Request& request, hwctrl::SetMotor::Response& response){
	if(request.id >= this->motors.size()){
		return false;
	}
	int id = request.id;
	ROS_INFO("Setting motor %d to %f at %f", id, request.setpoint, request.acceleration);
	HwMotor* motors = this->motors.data(); // pointer to our motor struct

	motors[id].setpoint = request.setpoint;
	if(fabs(request.acceleration) > motors[id].max_accel || fabs(request.acceleration) == 0.0){
		motors[id].accel_setpoint = motors[id].max_accel;
	}else {
		motors[id].accel_setpoint = request.acceleration;
	}
	response.actual_accel = motors[id].accel_setpoint;
	response.status = 0; // need to change later to be meaningful

	return true;
}

void HwMotorIf::add_motor(HwMotor mtr){
	this->motors.push_back(mtr);
}

void HwMotorIf::get_motors_from_csv(){
	std::string ros_package_path(std::getenv("ROS_PACKAGE_PATH"));
	std::istringstream path_stream(ros_package_path);

	std::string src_dir_path;
	std::getline(path_stream, src_dir_path, ':'); // get the first path
	if(*(src_dir_path.end()) != '/'){
		src_dir_path.push_back('/');
	}
	std::string config_file_path = src_dir_path.append(config_file_fname);

	std::vector<std::vector<std::string>> csv = read_csv(config_file_path);
	int line_num = 0;
	int id_ind, name_ind, category_ind, device_type_ind, interface_ind, device_id_ind, aux_1_ind, aux_2_ind, aux_3_ind, aux_4_ind;
	for(auto line = csv.begin(); line != csv.end(); ++line){
		if(line_num == 0){
			for(int i = 0; i < (*line).size(); i++){
				if((*line)[i].compare(id_hddr)==0){
					id_ind = i;
				}else if((*line)[i].compare(name_hddr)==0){
					name_ind = i;
				}else if((*line)[i].compare(category_hddr)==0){
					category_ind = i;
				}else if((*line)[i].compare(device_type_hddr)==0){
					device_type_ind = i;
				}else if((*line)[i].compare(interface_hddr)==0){
					interface_ind = i;
				}else if((*line)[i].compare(device_id_hddr)==0){
					device_id_ind = i;
				}else if((*line)[i].compare(aux_1_hddr)==0){
					aux_1_ind = i;
				}else if((*line)[i].compare(aux_2_hddr)==0){
					aux_2_ind = i;
				}else if((*line)[i].compare(aux_3_hddr)==0){
					aux_3_ind = i;
				}else if((*line)[i].compare(aux_4_hddr)==0){
					aux_4_ind = i;
				}else{
					// do nothing
				}
			}
		}else{
			if((*line)[category_ind].compare(category_motor) == 0){
				// make a motor struct and populate with data
				HwMotor motor;
				motor.id 			= std::stoi((*line)[id_ind]);
				motor.name 			= (*line)[name_ind];
				motor.device_id 	= std::stoi((*line)[device_id_ind]);
				motor.motor_type 	= get_device_type((*line)[device_type_ind]);
				motor.if_type 		= get_if_type((*line)[interface_ind]);
				motor.rpm_coef 		= std::stof((*line)[aux_2_ind]); // this will be our gear reduction
				motor.max_rpm 		= std::stof((*line)[aux_3_ind]); // our max rpm
				motor.max_accel		= std::stof((*line)[aux_4_ind]); // our max acceleration

				if(motor.motor_type == DEVICE_VESC){
					motor.ctrl_type = CTRL_RPM;
				}else{
					motor.ctrl_type = CTRL_POSITION;
				}
				this->motors.push_back(motor);
			}

		}
		line_num++;
	}
	this->motor_it = this->motors.begin();
}

int HwMotorIf::get_num_motors(){
	return this->motors.size();
}

std::string HwMotorIf::list_motors(){
	std::string retval;
	for(auto mtr = this->motors.begin(); mtr != this->motors.end(); ++mtr){
		retval.append((*mtr).to_string());
	}
	return retval;
}


////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
InterfaceType get_if_type(std::string type_str){
	if(type_str.compare("can") == 0){
		return IF_CANBUS;
	}else if(type_str.compare("uart") == 0){
		return IF_UART;
	}else if(type_str.compare("spi") == 0){
		return IF_SPI;
	}else if(type_str.compare("gpio") == 0){
		return IF_GPIO;
	}else{
		return IF_NONE;
	}
}

DeviceType get_device_type(std::string type_str){
	if      (type_str.compare("vesc") 		== 0){
		return DEVICE_VESC;
	}else if(type_str.compare("sabertooth") == 0){
		return DEVICE_SABERTOOTH;
	}else if(type_str.compare("uwb") 		== 0){
		return DEVICE_UWB;
	}else if(type_str.compare("quad_enc") 	== 0){
		return DEVICE_QUAD_ENC;
	}else if(type_str.compare("limit") 	== 0){
		return DEVICE_LIMIT_SW;
	}else{
		return DEVICE_NONE;
	}
}

std::string HwMotor::to_string(){
	sprintf(this->scratch_buf,
		"\n==========\nMotor\t: %s\n\rID\t: %d\n",
		this->name.c_str(), this->id
		);
	return std::string(this->scratch_buf);
}

// OBSOLTETE
void can_rx_sub_callback(const boost::shared_ptr<hwctrl::CanFrame>& frame){
	uint32_t rx_id = (uint32_t)frame->can_id;

	if((rx_id & ~0xFF) != 0){
		// we can assume this is a VESC message
		uint8_t cmd = (uint8_t)(rx_id >> 8);
		int8_t id 	= (int8_t)(rx_id & 0xFF);
		switch(cmd){
			case CAN_PACKET_STATUS:{

				boost::shared_ptr<hwctrl::VescData> vesc_msg(new hwctrl::VescData);

				fill_msg_from_status_packet(frame->data, vesc_msg);
				vesc_data_pub.publish(vesc_msg);
				break;
			}
			case CAN_PACKET_FILL_RX_BUFFER:{
				if(id != 0x00){
						// this is not intended for us (our canID is 0)
					break;
				}
				memcpy(vesc_rx_buf + frame->data[0], frame->data + 1, frame->can_dlc - 1);
				break;
			}
			case CAN_PACKET_PROCESS_RX_BUFFER:{
				if(id != 0x00){
					// not intended for us
					break;
				}
				int ind = 0;
				uint16_t packet_len;
				uint16_t crc;
				int vesc_id = frame->data[ind++]; // which vesc sent the data
				int n_cmds 	= frame->data[ind++]; // how many commands
				packet_len 	= (frame->data[ind++] << 8);
				packet_len 	|= frame->data[ind++];
				crc 		= (frame->data[ind++] << 8);
				crc 		|= frame->data[ind++];

				uint16_t chk_crc = crc16(vesc_rx_buf, packet_len);
				// ROS_INFO("PROCESSING RX BUFFER\nPacket Len:\t%d\nRcvd CRC:\t%x\nComputed CRC:\t%x", packet_len, crc, chk_crc);

				if(crc != chk_crc){
					ROS_INFO("Error: Checksum doesn't match");
					// error in transmission
					break;
				}

				boost::shared_ptr<hwctrl::VescData> vesc_msg(new hwctrl::VescData);

				ind = 0;
				int comm_cmd = vesc_rx_buf[ind++];
				switch(comm_cmd){
					case COMM_GET_VALUES:{
						vesc_msg->timestamp 	= ros::Time::now();
						vesc_msg->motor_type 	= "VESC";
						vesc_msg->can_id 		= vesc_id;

						fill_msg_from_buffer(vesc_rx_buf, vesc_msg);

						vesc_data_pub.publish(vesc_msg); // publish motor data
						break;}
					}

				break;
			}
				// indicates the end of the cmd buffer
		}
	}else{
		// we can assume this is either UWB message or Quad Encoder message
		rx_id = rx_id & CAN_SFF_MASK;
		uint8_t type = frame->data[0];
		if(type == 0 || type == 0xFF){
			// yes this is an UWB msg
			hwctrl::UwbData uwb_msg;
			anchor_frames++;
			float distance;
			int16_t confidence;
			memcpy(&distance, frame->data+2, 4);
			memcpy(&confidence, frame->data+6, 2);
			// ROS_INFO("Distance from node %d to anchor %d: %.3f m", rx_id, dist_data.anchor_id, dist_data.distance);
			uwb_msg.timestamp 	= ros::Time::now();
			uwb_msg.node_id 	= rx_id;
			uwb_msg.anchor_id 	= frame->data[1];
			uwb_msg.distance 	= distance;
			uwb_msg.confidence = confidence;

			uwb_pub.publish(uwb_msg);
		}else{
			// quad encoder msg
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// SENSOR STUFF
////////////////////////////////////////////////////////////////////////////////
/**
 * thread to do sensor things
 */
void sensor_thread(SensorIf* sensor_if){
	while(ros::ok()){
		// check which UWB nodes have anchor data ready. read and publish them
		for(auto node = this->uwb_nodes.begin(); node != this->uwb_nodes.end(); ++node){
			int n_msgs = 4;
			hwctrl::UwbData uwb_msgs[n_msgs] = {};
			n_msgs = node->get_msgs_from_anchors(uwb_msgs, n_msgs);
			for(int i = 0; i < n_msgs; i++){
				this->uwb_data_pub.publish(uwb_msgs[i]);
			}
		}

		// poll limit switches

		// publish any limit switches that have occurred

		// read SPI bus sensors

		// publish those
		this->loop_rate.sleep();
	}

}

/**
 * SensorIf constructor
 * creates publishers, subscribers, timers, opens interfaces, initializes GPIO, etc
 */
SensorIf::SensorIf(ros::NodeHandle n)
: loop_rate(100), uwb_update_period(0.1) {
	this->nh = n;

	this->can_rx_sub = this->nh.subscribe("can_frames_rx", 128, &HwMotorIf::can_rx_callback, this);

	this->can_tx_pub = this->nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
	this->sensor_data_pub = this->nh.advertise<hwctrl::SensorData>("sensor_data", 128);
	this->limit_sw_pub = this->nh.advertise<hwctrl::LimitSwState>("limit_sw_states", 128);
	this->uwb_data_pub = this->nh.advertise<hwctrl::UwbData>("localization_data", 1024);

	this->spi_handle = spi_init("/dev/spidev0.0");
	if(this->spi_handle < 0){
		ROS_INFO("SPI init failed :(");
	}else{
		ROS_INFO("SPI init presumed successful");
	}

	int gpio_state = this->setup_gpio();
	if(gpio_state != 0){
		ROS_INFO("GPIO setup failed :(");
	}else{
		ROS_INFO("GPIO setup succeeded!");
	}
	// create the timer that will request data from the next UWB node
	this->uwb_update_timer = this->nh.createTimer(this->uwb_update_period, this->uwb_update_callback);
}

/**
 * necessary setup of GPIO's
 */
int SensorIf::setup_gpio(){
	return 0;
}

/**
 * do things with new CAN messages, really for UWB and quadrature encoder data
 */
void SensorIf::can_rx_callback(const boost::shared_ptr<hwctrl::CanFrame>& frame){
	uint32_t rx_id = (uint32_t)frame->can_id;

	if((uint32_t can_id = rx_id & ~0xFF) == 0){
		// was NOT a VESC frame
		// UWB frame or quad encoder frame
		UwbNode* uwb_node = this->get_uwb_by_can_id(can_id);
		if(uwb_node != NULL){
			uwb_node->add_can_data(frame->data, frame->can_dlc);
		}else{
			// no UWB node matching this CAN ID, so must be our quadrature encoder
			this->quad_encoder.add_can_data(frame->data, frame->can_dlc);
		}
	}
}

/**
 * returns a pointer to the UWB node object that has the given CAN ID
 */
UwbNode* SensorIf::get_uwb_by_can_id(int can_id){
	auto i = this->uwb_nodes.begin();
	UwbNode* node = NULL;
	while(i != this->uwb_nodes.end() && i->get_id() != can_id){
		++i; // increment to next UWB Node
		node = &(*i);
	}
	return node;
}

/**
 * will be called by a timer
 * when called, will send a remote request to the next UWB node to get ranging data
 */
void SensorIf::uwb_update_callback(const ros::TimerEvent& tim_event){
		UwbNode* nodes = this->uwb_nodes.data();
		UwbNode* uwb_node = &(nodes[this->uwb_ind]);

		boost::shared_ptr<hwctrl::CanFrame> can_msg(new hwctrl::CanFrame()); // empty can frame msg built as a shared pointer
		can_msg->can_id  = uwb_node->id | CAN_RTR_FLAG | CAN_SFF_FLAG; // can ID + remote request & std ID flags
		can_msg->can_dlc = UWB_CAN_HDDR_SIZE + 4; // this won't really be used but whatever
		this->can_tx_pub.publish(can_msg);

		this->uwb_ind++;
		if(this->uwb_ind >= this->uwb_nodes.size()){
			this->uwb_ind = 0;
		}
}

void SensorIf::get_sensors_from_csv(){
	std::string ros_package_path(std::getenv("ROS_PACKAGE_PATH"));
	std::istringstream path_stream(ros_package_path);

	std::string src_dir_path;
	std::getline(path_stream, src_dir_path, ':'); // get the first path
	if(*(src_dir_path.end()) != '/'){
		src_dir_path.push_back('/');
	}
	std::string config_file_path = src_dir_path.append(config_file_fname);

	std::vector<std::vector<std::string>> csv = read_csv(config_file_path);
	int line_num = 0;
	int id_ind, name_ind, category_ind, device_type_ind, interface_ind, device_id_ind, aux_1_ind, aux_2_ind, aux_3_ind, aux_4_ind;
	for(auto line = csv.begin(); line != csv.end(); ++line){
		if(line_num == 0){
			for(int i = 0; i < (*line).size(); i++){
				if((*line)[i].compare(id_hddr)==0){
					id_ind = i;
				}else if((*line)[i].compare(name_hddr)==0){
					name_ind = i;
				}else if((*line)[i].compare(category_hddr)==0){
					category_ind = i;
				}else if((*line)[i].compare(device_type_hddr)==0){
					device_type_ind = i;
				}else if((*line)[i].compare(interface_hddr)==0){
					interface_ind = i;
				}else if((*line)[i].compare(device_id_hddr)==0){
					device_id_ind = i;
				}else if((*line)[i].compare(aux_1_hddr)==0){
					aux_1_ind = i;
				}else if((*line)[i].compare(aux_2_hddr)==0){
					aux_2_ind = i;
				}else if((*line)[i].compare(aux_3_hddr)==0){
					aux_3_ind = i;
				}else if((*line)[i].compare(aux_4_hddr)==0){
					aux_4_ind = i;
				}else{
					// do nothing
				}
			}
		}else{
			if((*line)[category_ind].compare(category_sensor) == 0){
				// make a motor struct and populate with data
				HwMotor motor;
				motor.id 			= std::stoi((*line)[id_ind]);
				motor.name 			= (*line)[name_ind];
				motor.device_id 	= std::stoi((*line)[device_id_ind]);
				motor.motor_type 	= get_device_type((*line)[device_type_ind]);
				motor.if_type 		= get_if_type((*line)[interface_ind]);
				motor.rpm_coef 		= std::stof((*line)[aux_2_ind]); // this will be our gear reduction
				motor.max_rpm 		= std::stof((*line)[aux_3_ind]); // our max rpm
				motor.max_accel		= std::stof((*line)[aux_4_ind]); // our max acceleration

				if(motor.motor_type == DEVICE_VESC){
					motor.ctrl_type = CTRL_RPM;
				}else{
					motor.ctrl_type = CTRL_POSITION;
				}
				this->motors.push_back(motor);
			}

		}
		line_num++;
	}
	this->motor_it = this->motors.begin();
}
