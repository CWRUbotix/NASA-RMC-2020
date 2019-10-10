#include <canbus.h>

int main(int argc, char** argv){
	ROS_INFO("CANbus Node!!!!!");
	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;
	
	ros::Publisher can_pub = n.advertise<UWB_msg>("localization_data", 1024);
	ros::Publisher motor_data = n.advertise<motor_data_msg>("motor_data", 1024);
	ros::Rate loop_rate(10);
	
	ROS_INFO("ROS init success");
	//nNodes 		= get_nodes_from_file(node_config_fname, node_str);
	//nAnchors 	= get_nodes_from_file(node_config_fname, anchor_str);
	//ROS_INFO("# of Nodes: %d,\t# of Anchors: %d", nNodes, nAnchors);
	int nVescStartID 	= 5;
	int nVescEndID 		= 6;
	int nVescID 		= nVescStartID;

	nNodes = 3;
	nAnchors = 3;

	UwbNode nodes[nNodes] = {};

	nodes[0].id = 1;
	nodes[1].id = 2;
	nodes[2].id = 3;

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
	std::vector<struct can_frame> rx_frame_queue; 	// to store received can frames to process later
	std::vector<struct can_frame> vesc_frames; 		// stores received vesc frames
	std::vector<motor_data_msg> motor_msgs; 		// stores motor ROS messages
	tv.tv_sec = 0;
	tv.tv_usec = 50000; // 50ms
	
	uint8_t vesc_rx_buf[1024];
	uint8_t rx_buf[8];
	DistanceFrame dist_data;

	const char* ifname = "can1";

	uint8_t node_id = 1; // 0 is our ID

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
		UwbNode* node = &nodes[node_id];
		tx_frame.can_id = node->id | CAN_RTR_FLAG; 
		tx_frame.can_dlc = 8;
		nbytes = write(s, &tx_frame, sizeof(struct can_frame));
		node_id++;
		if(node_id > nNodes){
			node_id = 1; // start back at the beginning
		}

		// REQUEST VALUES FROM NEXT VESC
		int vesc_success = get_values(s, nVescID, 0);
		nVescID++;
		if(nVescID > nVescEndID){
			nVescID = nVescStartID;
		}

		// SLEEP TO ALLOW DEVICES TO RESPOND
		loop_rate.sleep();


		// BRING RECEIVED FRAMES INTO USER SPACE
		nbytes = 0;
		while((nbytes = read(s, &rx_frame, sizeof(struct can_frame))) > 0){
			int rx_id = (int)rx_frame.can_id;
			if(rx_id > 0xFF){
				// we can assume this is a VESC message
				vesc_frames.push_back(rx_frame);
				uint8_t cmd = (uint8_t)(rx_frame.can_id >> 8);
				int8_t id 	= (int8_t)(rx_frame.can_id);
				switch(cmd){
					case CAN_PACKET_FILL_RX_BUFFER:{
						memcpy(vesc_rx_buf + rx_frame.data[0], rx_frame.data + 1, rx_frame.can_dlc - 1);
						break;}
					case CAN_PACKET_PROCESS_RX_BUFFER:{
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
							// error in transmission
							break;
						}
						ind = 0;
						int comm_cmd = vesc_rx_buf[ind++];
						switch(comm_cmd){
							case COMM_GET_VALUES:{
								canbus::motor_data motor_msg;
								motor_msg.timestamp 	= ros::Time::now();
								motor_msg.motor_type 	= "VESC";
								motor_msg.can_id 		= vesc_id;

								fill_msg_from_buffer(vesc_rx_buf, motor_msg);

								motor_data.publish(motor_msg); // publish motor data
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
