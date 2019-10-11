#include <canbus.h>

int main(int argc, char** argv){
	ROS_INFO("CANbus Node!!!!!");
	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;
	
	ros::Publisher can_pub = n.advertise<UWB_msg>("localization_data", 1024);
	ros::Publisher motor_data = n.advertise<motor_data_msg>("motor_data", 1024);
	ros::Rate loop_rate(50); // 20ms loop rate
	
	ROS_INFO("ROS init success");

	std::vector<CanDevice> can_devices;
	std::vector<UwbNode> nodes;
	std::vector<canbus::motor_data> motor_msgs; // array of messages for each VESC
	int nVescStartID 	= 0;
	int nVescEndID 		= 0;

	int config_read_status = read_can_config(can_config_fname, can_devices);
	if(config_read_status != 0){
		ROS_INFO("CONFIG READ FAILED!");
		UwbNode node;
		node.id = 1;
		nodes.push_back(node);
		node.id = 2;
		nodes.push_back(node);
		node.id = 3;
		nodes.push_back(node);
		node.id = 4;
		nodes.push_back(node);

		canbus::motor_data temp_msg;
		temp_msg.can_id = 5;
		motor_msgs.push_back(temp_msg);
		temp_msg.can_id = 6;
		motor_msgs.push_back(temp_msg);

	}else{
		ROS_INFO("CONFIG READ SUCCESS!");
		ROS_INFO("Read %d lines from can config file.", can_devices.size());
		for(int i = 0; i < can_devices.size(); i++){
			if((can_devices[i].type.compare("uwb")) == 0){
				ROS_INFO("Adding UWB Node");
				UwbNode new_node;
				new_node.id = can_devices[i].can_id;
				nodes.push_back(new_node);
			}else if((can_devices[i].type.compare("vesc")) == 0){
				ROS_INFO("Adding VESC device.");
				canbus::motor_data new_msg;
				new_msg.can_id = can_devices[i].can_id;
				new_msg.motor_type = can_devices[i].type;
				motor_msgs.push_back(new_msg);
			}
		}
	}
	nVescEndID 	= motor_msgs[motor_msgs.size()-1].can_id;
	nVescStartID 		= motor_msgs[0].can_id;
	int nVescID 	= nVescStartID;

	nNodes 			= (nodes[nodes.size()-1].id - nodes[0].id) + 1;

	UWB_msg msg;
	motor_data_msg motor_msg;
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct can_frame rx_frame;
	struct can_frame tx_frame;
	struct ifreq ifr;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 25000; // 25ms CAN bus read timeout 
	
	uint8_t vesc_rx_buf[1024];
	uint8_t rx_buf[8];
	DistanceFrame dist_data;

	const char* ifname = "can1";

	uint8_t node_ind = 0; //

	// initialize the CAN socket
	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
		perror("Error while opening socket");
		ROS_INFO("Error while opening CAN socket");
		return -1;
	}
	
	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	ROS_INFO("%s at index %d", ifname, ifr.ifr_ifindex);
	
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
	
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0){
		perror("Error in socket bind");
		ROS_INFO("Error in CAN socket bind");
		return -2;
	}

	while(ros::ok()){
		// REQUEST DATA FROM NEXT UWB NODE
		UwbNode* node = &nodes[node_ind];
		tx_frame.can_id = node->id | CAN_RTR_FLAG; 
		tx_frame.can_dlc = 8;
		nbytes = write(s, &tx_frame, sizeof(struct can_frame));
		node_ind++;
		if(node_ind >= nNodes){
			node_ind = 0; // start back at the beginning
		}

		// REQUEST VALUES FROM NEXT VESC
		int vesc_success = get_values(s, nVescID, 0);

		nVescID++;
		if(nVescID >= nVescEndID){
			nVescID = nVescStartID; // start back at the beginning
		}

		// SLEEP TO ALLOW DEVICES TO RESPOND
		loop_rate.sleep();


		// BRING RECEIVED FRAMES INTO USER SPACE
		nbytes = 0;
		while((nbytes = read(s, &rx_frame, sizeof(struct can_frame))) > 0){
			uint32_t rx_id = (uint32_t)rx_frame.can_id;
			if((rx_id & ~0xFF) != 0){
				// we can assume this is a VESC message
				uint8_t cmd = (uint8_t)(rx_frame.can_id >> 8);
				int8_t id 	= (int8_t)(rx_frame.can_id & 0xFF);
				switch(cmd){
					case CAN_PACKET_STATUS:{
						int index = id - nVescStartID;
						if(id <= 0){
							break;
						}
						fill_msg_from_status_packet(rx_frame.data, motor_msgs[index]);
						motor_data.publish(motor_msgs[index]);
						break;}
					case CAN_PACKET_FILL_RX_BUFFER:{
						if(id != 0x00){
							// this is not intended for us (our canID is 0)
							break;
						}
						memcpy(vesc_rx_buf + rx_frame.data[0], rx_frame.data + 1, rx_frame.can_dlc - 1);
						break;}
					case CAN_PACKET_PROCESS_RX_BUFFER:{
						if(id != 0x00){
							// not intended for us
							break;
						}
						int ind = 0;
						uint16_t packet_len;
						uint16_t crc;
						int vesc_id = rx_frame.data[ind++]; // which vesc sent the data
						int n_cmds 	= rx_frame.data[ind++]; // how many commands
						packet_len 	= (rx_frame.data[ind++] << 8);
						packet_len 	|= rx_frame.data[ind++];
						crc 		= (rx_frame.data[ind++] << 8);
						crc 		|= rx_frame.data[ind++];

						uint16_t chk_crc = crc16(vesc_rx_buf, packet_len);

						if(crc != chk_crc){
							ROS_INFO("Error: Checksum doesn't match");
							// error in transmission
							break;
						}

						int index = vesc_id - nVescStartID;
						if(index < 0){break;}
						
						ind = 0;
						int comm_cmd = vesc_rx_buf[ind++];
						switch(comm_cmd){
							case COMM_GET_VALUES:{
								motor_msgs[index].timestamp 	= ros::Time::now();
								motor_msgs[index].motor_type 	= "VESC";
								motor_msgs[index].can_id 		= vesc_id;
								
								fill_msg_from_buffer(vesc_rx_buf, *(motor_msgs.data()+index));

								motor_data.publish(motor_msgs[index]); // publish motor data
								break;}
						}

						break;}
						// indicates the end of the cmd buffer
				}
			}else{
				// we can assume this is a UWB message
				rx_id = rx_id & CAN_SFF_MASK;
				if(rx_id <= nNodes){ 
					// yes this is an UWB node
					memcpy(&rx_buf, rx_frame.data, 8);
					dist_data.type = rx_buf[0];
					dist_data.anchor_id = rx_buf[1];
					memcpy(&(dist_data.distance), rx_buf+2, sizeof(dist_data.distance));
					memcpy(&(dist_data.confidence), rx_buf+6, 2);
					ROS_INFO("Distance from node %d to anchor %d: %.3f m", rx_id, dist_data.anchor_id, dist_data.distance);
					msg.timestamp 	= ros::Time::now();
					msg.node_id 	= rx_id;
					msg.anchor_id 	= dist_data.anchor_id;
					msg.distance 	= dist_data.distance;
					msg.confidence 	= dist_data.confidence;
					can_pub.publish(msg);
				}
			}
		}

		ros::spinOnce();
	}

	return 0;
}

int get_nodes_from_file(string fname, string sType){
	return 0;
	
	ifstream node_data(fname.c_str());

	string line;
	
	int retval;
	
	if(node_data.is_open()){

		while(getline(node_data, line, '\n')){
			char* type = '\0';//strstr(line.c_str(), sType.c_str());
			if(type != NULL){
				retval++;
			}
		}
	}

	node_data.close();

	return retval;
}

int read_can_config(std::string fname, std::vector<CanDevice> &devices){
	int retval = 0;
	std::ifstream config_file;
	config_file.open(fname.c_str(), ios::in);
	std::string line;
	std::string word;

	std::vector<std::string> words;

	int line_num = 0;
	int can_id_ind = -1;
	int device_type_ind = -1;
	if(config_file.is_open()){
		while(std::getline(config_file, line)){
			line.push_back(','); // make sure we can read the last word
			std::istringstream line_stream(line);
			while(std::getline(line_stream, word, ',')){
				words.push_back(word);
			}
			if(line_num == 0){
				// figure out which ID corresponds to which column
				for(int i = 0; i < words.size(); i++){
					if(can_id_ind < 0 && words[i].compare(can_id_hddr) == 0){
						can_id_ind = i;
					}else if(device_type_ind < 0 && words[i].compare(device_type_hddr) == 0){
						device_type_ind = i;
					}
				}
			}else{
				if(can_id_ind < 0 || device_type_ind < 0){
					return 2;
				}
				CanDevice device;
				device.can_id 	= std::stoi(words[can_id_ind]);
				device.type 	= words[device_type_ind];
				devices.push_back(device);
			}
			line_num++;
			
			words.clear();
		}
	}else{
		return 1;
	}

	config_file.close();

	return 0;
}
