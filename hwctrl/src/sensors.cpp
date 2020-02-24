#include <hwctrl.h>
////////////////////////////////////////////////////////////////////////////////
// SENSOR STUFF
////////////////////////////////////////////////////////////////////////////////
/**
 * thread to do sensor things
 */
void sensors_thread(SensorIf* sensor_if){
	ROS_INFO("Starting sensors_thread");
	ros::AsyncSpinner spinner(1, &(sensor_if->cb_queue));
	spinner.start();
	while(ros::ok()){
		// check which UWB nodes have anchor data ready. read and publish them
		/*for(auto node = sensor_if->uwb_nodes.begin(); node != sensor_if->uwb_nodes.end(); ++node){
			int n_msgs = 4;
			hwctrl::UwbData uwb_msgs[n_msgs] = {};
			n_msgs = node->get_msgs_from_anchors(uwb_msgs, n_msgs);
			ROS_INFO("We have %d UWB messages to publish", n_msgs);
			for(int i = 0; i < n_msgs; i++){
				sensor_if->uwb_data_pub.publish(uwb_msgs[i]);
			}
		}*/

		// poll limit switches

		// publish any limit switches that have occurred

		// read SPI bus sensors

		// publish those
		sensor_if->loop_rate.sleep();
	}

}

/**
 * SensorIf constructor
 * creates publishers, subscribers, timers, opens interfaces, initializes GPIO, etc
 */
SensorIf::SensorIf(ros::NodeHandle n)
: loop_rate(100), uwb_update_period(0.1) {
	this->nh = n;

	this->nh.setCallbackQueue(&(this->cb_queue)); // have this copy use this callback queue

	this->can_rx_sub = this->nh.subscribe("can_frames_rx", 128, &SensorIf::can_rx_callback, this);

	this->can_tx_pub = this->nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
	this->sensor_data_pub = this->nh.advertise<hwctrl::SensorData>("sensor_data", 128);
	this->limit_sw_pub = this->nh.advertise<hwctrl::LimitSwState>("limit_sw_states", 128);
	this->uwb_data_pub = this->nh.advertise<hwctrl::UwbData>("localization_data", 1024);

	this->get_sensors_from_csv();

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
	this->uwb_update_timer = this->nh.createTimer(this->uwb_update_period, &SensorIf::uwb_update_callback, this);
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
void SensorIf::can_rx_callback(boost::shared_ptr<hwctrl::CanFrame> frame){
	uint32_t rx_id = (uint32_t)frame->can_id;
	uint32_t can_id = 0;
	if((can_id = (rx_id & ~0xFF)) == 0){
		// was NOT a VESC frame
		// UWB frame or quad encoder frame
		UwbNode* uwb_node = this->get_uwb_by_can_id(rx_id);
		if(uwb_node != NULL){
			ROS_INFO("UWB frame (node id %d)", rx_id);
			// uwb_node->add_can_data(frame->data.data(), frame->can_dlc);
			hwctrl::UwbData msg;
			msg.anchor_id = frame->data[1];
			float temp = 0.0;
			memcpy(&temp, frame->data.data() + 2, 4);
			msg.distance = temp;
			msg.node_id = rx_id;
			this->uwb_data_pub.publish(msg);
		}else{
			// no UWB node matching this CAN ID, so must be our quadrature encoder
			//this->quad_encoder.add_can_data(frame->data.data(), frame->can_dlc);
		}
	}
}

/**
 * returns a pointer to the UWB node object that has the given CAN ID
 */
UwbNode* SensorIf::get_uwb_by_can_id(int can_id){
	auto i = this->uwb_nodes.begin();
	UwbNode* node = &(*i);
	while(i != this->uwb_nodes.end() && i->id != can_id){
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
	// ROS_INFO("===== UWB UPDATE CALLBACK =====");
	if(this->uwb_nodes.size() <= 0){
		ROS_INFO("No UWB nodes");
		return;
	}
	//UwbNode* nodes = this->uwb_nodes.data();
	//UwbNode* uwb_node = &(nodes[this->uwb_ind]);
	UwbNode node = this->uwb_nodes.at(this->uwb_ind);
	boost::shared_ptr<hwctrl::CanFrame> can_msg(new hwctrl::CanFrame()); // empty can frame msg built as a shared pointer
	can_msg->can_id  = node.id | CAN_RTR_FLAG; // can ID + remote request & std ID flags
	can_msg->can_dlc = 8; // I think it needs to be 8 for the legacy fw? UWB_CAN_HDDR_SIZE + 4; // this won't really be used but whatever
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
				ROS_INFO("A sensor!");
				DeviceType dev_type = get_device_type((*line)[device_type_ind]);
				if( (*line)[device_type_ind].compare("uwb") == 0){
					// this line is for an UWB node
					uint32_t id = (uint32_t)std::stoi((*line)[device_id_ind]);
					ROS_INFO("An UWB node, id: %d!", id);
					UwbNode uwb_node(id);
					this->uwb_nodes.push_back(uwb_node);
				}

			}

		}
		line_num++;
	}
}
