#include <canbus.h>

int main(int argc, char** argv){
	ROS_INFO("CANbus Node!!!!!");
	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;

	ros::Publisher can_pub = n.advertise<UWB_msg>("localization_data", 1024);
	ros::Publisher MotorData = n.advertise<MotorData_msg>("MotorData", 1024);
	ros::Rate loop_rate(50); // 20ms loop rate
	
	ROS_INFO("ROS init success");

	std::vector<CanDevice> can_devices_vect;

	int config_read_status = read_can_config(can_config_fname, can_devices_vect);
	if(config_read_status != 0){
		ROS_INFO("CONFIG READ FAILED!");
		CanDevice new_device;
		new_device.type = "uwb";
		new_device.can_id = 1;
		can_devices_vect.push_back(new_device);
		new_device.type = "uwb";
		new_device.can_id = 2;
		can_devices_vect.push_back(new_device);
		new_device.type = "uwb";
		new_device.can_id = 3;
		can_devices_vect.push_back(new_device);
		new_device.type = "uwb";
		new_device.can_id = 4;
		can_devices_vect.push_back(new_device);

		new_device.type = "vesc";
		new_device.can_id = 5;
		can_devices_vect.push_back(new_device);
		new_device.type = "vesc";
		new_device.can_id = 6;
		can_devices_vect.push_back(new_device);
	}else{
		ROS_INFO("CONFIG READ SUCCESS!");
		ROS_INFO("Read %d lines from can config file.", (int)can_devices_vect.size());
	}

	int nCanDevices = can_devices_vect.size();
	ROS_INFO("Number of CAN devices (including ourself): %d", nCanDevices + 1);

	// make our array of can devices
	// each device will be at index == can_id
	// we need one extra (unused) device for ourselves at index 0
	CanDevice can_devices[nCanDevices + 1]; 
	can_devices[0].type = "self";
	can_devices[0].can_id = 0;

	// find out how many of each node exists
	int nVescs = 0, nUwbNodes = 0;
	for(int i = 0; i < can_devices_vect.size(); i++){
		if((can_devices_vect[i].type.compare("uwb")) == 0){
			ROS_INFO("Adding UWB Node");
			nUwbNodes++;
		}else if((can_devices_vect[i].type.compare("vesc")) == 0){
			ROS_INFO("Adding VESC device.");
			nVescs++;
		}
	}
	ROS_INFO("nUwbNodes: %d, nVescs: %d", nUwbNodes, nVescs);

	// allocate memory for the messages
	canbus::UwbData UWB_msgs_arr[nUwbNodes];
	canbus::MotorData motor_msgs_arr[nVescs];
	
	int UwbInd 			= 0;
	int VescInd 		= 0;
	int nVescStartID 	= 128;
	int nVescEndID 		= 0;
	int nUwbStartID 	= 0x7FF;
	int nUwbEndID 		= 0;

	// populate our can_devices array such that it's sorted by can_id
	for(int i = 0; i < nCanDevices; i++){
		int j = can_devices_vect[i].can_id; // actual can ID
		if(j > nCanDevices){
			continue;
		}
		if((can_devices_vect[i].type.compare("uwb")) == 0){
			if(j < nVescStartID){nVescStartID = j;}
			if(j > nVescEndID){nVescEndID = j;}
			can_devices[j] = can_devices_vect[i];
			can_devices[j].uwb_msg = &(UWB_msgs_arr[UwbInd]);
			can_devices[j].uwb_msg->node_id = j;
			UwbInd++;
		}else if((can_devices_vect[i].type.compare("vesc")) == 0){
			if(j < nVescStartID){nVescStartID = j;}
			if(j > nVescEndID){nVescEndID = j;}
			can_devices[j]= can_devices_vect[i];
			can_devices[j].vesc_msg = &(motor_msgs_arr[VescInd]);
			can_devices[j].vesc_msg->can_id = j;
			VescInd++;
		}else{
			// do nothing
		}
	}
	ROS_INFO("Vesc Start,end ID: %d, %d",nVescStartID, nVescEndID);
	int nVescID 	= nVescStartID;

	int node_ind 	= nUwbStartID;
	nNodes 			= nUwbNodes;

	UWB_msg msg;
	MotorData_msg motor_msg;
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

	VescCan vesc_can_obj(s, 0); //
	
	// Provide the set_vesc service
	// we have to do it after CAN is set up
	ros::ServiceServer ser_vesc_srv = n.advertiseService("set_vesc", &VescCan::set_vesc_callback, &vesc_can_obj);
	

	while(ros::ok()){
		// REQUEST DATA FROM NEXT UWB NODE
		CanDevice* uwb_node = &(can_devices[node_ind]);
		tx_frame.can_id = uwb_node->can_id | CAN_RTR_FLAG; 
		tx_frame.can_dlc = 8;
		nbytes = write(s, &tx_frame, sizeof(struct can_frame));

		node_ind++;
		if(node_ind > nUwbEndID){
			node_ind = nUwbStartID; // start back at the beginning
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
						CanDevice* vesc = &(can_devices[id]);

						if(vesc->vesc_msg == NULL || vesc->type.compare("vesc") == 0){
							break;
						}
						fill_msg_from_status_packet(rx_frame.data, *(vesc->vesc_msg));
						MotorData.publish(*(vesc->vesc_msg));
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
						CanDevice* vesc = &(can_devices[vesc_id]);
						if(vesc->vesc_msg == NULL || vesc->type.compare("vesc") == 0){
							break;
						}
						
						ind = 0;
						int comm_cmd = vesc_rx_buf[ind++];
						switch(comm_cmd){
							case COMM_GET_VALUES:{
								vesc->vesc_msg->timestamp 	= ros::Time::now();
								vesc->vesc_msg->motor_type 	= "VESC";
								vesc->vesc_msg->can_id 		= vesc_id;
								
								fill_msg_from_buffer(vesc_rx_buf, *(vesc->vesc_msg));

								MotorData.publish(*(vesc->vesc_msg)); // publish motor data
								break;}
						}

						break;}
						// indicates the end of the cmd buffer
				}
			}else{
				// we can assume this is a UWB message
				rx_id = rx_id & CAN_SFF_MASK;
				if(rx_id <= nUwbEndID && rx_id >= nUwbEndID){ 
					// yes this is an UWB node
					CanDevice* uwb_boi = &(can_devices[rx_id]);
					memcpy(&rx_buf, rx_frame.data, 8);
					dist_data.type = rx_buf[0];
					dist_data.anchor_id = rx_buf[1];
					memcpy(&(dist_data.distance), rx_buf+2, sizeof(dist_data.distance));
					memcpy(&(dist_data.confidence), rx_buf+6, 2);
					ROS_INFO("Distance from node %d to anchor %d: %.3f m", rx_id, dist_data.anchor_id, dist_data.distance);
					uwb_boi->uwb_msg->timestamp 	= ros::Time::now();
					uwb_boi->uwb_msg->node_id 	= rx_id;
					uwb_boi->uwb_msg->anchor_id 	= dist_data.anchor_id;
					uwb_boi->uwb_msg->distance 	= dist_data.distance;
					uwb_boi->uwb_msg->confidence = dist_data.confidence;
					can_pub.publish(*(uwb_boi->uwb_msg));
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
	std::string ros_package_path(std::getenv("ROS_PACKAGE_PATH"));
	std::istringstream path_stream(ros_package_path);

	std::string src_dir_path;
	std::getline(path_stream, src_dir_path, ':'); // get the first path
	if(*(src_dir_path.end()) != '/'){
		src_dir_path.push_back('/');
	}
	fname = src_dir_path.append(fname);
	int retval = 0;
	std::ifstream config_file;
	config_file.open(fname.c_str(), ios::in);
	std::string line;
	std::string word;

	std::vector<std::string> words;

	int line_num = 0;
	int can_id_ind 		= -1;
	int device_if_ind 	= -1;
	int device_type_ind = -1;
	int device_name_ind = -1;
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
					}else if(device_if_ind < 0 && words[i].compare(device_if_hddr) == 0){
						device_if_ind = i;
					}else if(device_name_ind < 0 && words[i].compare(device_name_hddr) == 0){
						device_name_ind = i;
					}
				}
			}else{
				if(can_id_ind < 0 || device_type_ind < 0){
					return 2;
				}
				if(words[device_if_ind].compare("can") == 0){
					CanDevice device;
					device.can_id 	= std::stoi(words[can_id_ind]);
					device.type 	= words[device_type_ind];
					devices.push_back(device);
				}
			}
			line_num++;
			
			words.clear();
		}
	}else{
		ROS_INFO("File couldn't be opened :(");
		return 1;
	}

	config_file.close();

	return 0;
}
