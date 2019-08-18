#include <canbus.h>

int main(int argc, char** argv){
	ROS_INFO("CANbus Node!!!!!");
	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;
	
	ros::Publisher can_pub = n.advertise<UWB_msg>("localization_data", 1024);
	ros::Rate loop_rate(10);
	
	ROS_INFO("ROS init success");
	//nNodes 		= get_nodes_from_file(node_config_fname, node_str);
	//nAnchors 	= get_nodes_from_file(node_config_fname, anchor_str);
	//ROS_INFO("# of Nodes: %d,\t# of Anchors: %d", nNodes, nAnchors);
	
	nNodes = 3;
	nAnchors = 3;

	UwbNode nodes[nNodes] = {};

	nodes[0].id = 1;
	nodes[1].id = 2;
	nodes[2].id = 3;

	UWB_msg msg;
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct can_frame rx_frame;
	struct can_frame tx_frame;
	struct ifreq ifr;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 500000;
	
	
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
		for(node_id = 0; node_id < nNodes; node_id++){
			UwbNode* node = &nodes[node_id];

			// ----- REQUEST DATA FROM THIS NODE -----
			tx_frame.can_id = node->id | CAN_RTR_FLAG; 
			tx_frame.can_dlc = 8;
			nbytes = write(s, &tx_frame, sizeof(struct can_frame));
												
			ros::Duration(0.1).sleep();

			// ROS_INFO("Wrote %d bytes to CAN bus.", nbytes);
			for(int i = 0; i < nAnchors; i++){
				nbytes = 0;
				int n_tries = 0;
				while(nbytes <= 0 && n_tries < MAX_CAN_TRIES){

					nbytes = read(s, &rx_frame, sizeof(struct can_frame));
					int rx_id = (int)rx_frame.can_id & CAN_SFF_MASK;
				
					if(nbytes > 0 && rx_frame.can_dlc == 8 && rx_id == node->id){
						memcpy(&rx_buf, rx_frame.data, 8);
						dist_data.type = rx_buf[0];
						dist_data.anchor_id = rx_buf[1];
						memcpy(&(dist_data.distance), rx_buf+2, sizeof(dist_data.distance));
						memcpy(&(dist_data.confidence), rx_buf+6, 2);
						ROS_INFO("Distance from node %d to anchor %d: %.3f m", node->id, dist_data.anchor_id, dist_data.distance);
						msg.timestamp = ros::Time::now();
						msg.node_id = node->id;
						msg.anchor_id = dist_data.anchor_id;
						msg.distance = dist_data.distance;
						msg.confidence = dist_data.confidence;
						can_pub.publish(msg);
					}else{
						nbytes = 0;
						ROS_INFO("No bytes received");
						n_tries++;	
					}
				}

			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
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
			char* type = strstr(line.c_str(), sType.c_str());
			if(type != NULL){
				retval++;
			}
		}
	}

	node_data.close();

	return retval;
}
