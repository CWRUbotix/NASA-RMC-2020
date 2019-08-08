#include <canbus.h>

#define UWB_NODES 2
#define NUM_ANCHORS 3
int main(int argc, char** argv){
	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;
	
	ros::Publisher can_pub = n.advertise<UWB_msg>("localization_data", 1024);
	ros::Rate loop_rate(1);

	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct can_frame rx_frame;
	struct can_frame tx_frame;
	struct ifreq ifr;
	struct timeval tv;
	tv.tv_sec = 1;
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
		for(node_id = 1; node_id <= UWB_NODES; node_id++){

			// ----- REQUEST DATA FROM THIS NODE -----
			tx_frame.can_id = node_id | CAN_RTR_FLAG; 
			tx_frame.can_dlc = 8;
			nbytes = write(s, &tx_frame, sizeof(struct can_frame));
												
			ros::Duration(0.25).sleep();

			// ROS_INFO("Wrote %d bytes to CAN bus.", nbytes);
			for(int i = 0; i < NUM_ANCHORS; i++){
				nbytes = 0;
				while(nbytes <= 0){

					nbytes = read(s, &rx_frame, sizeof(struct can_frame));
		
					int rx_id = (int)rx_frame.can_id & CAN_SFF_MASK;
				
					if(nbytes > 0 && rx_frame.can_dlc == 8 && rx_id == node_id){
						memcpy(&rx_buf, rx_frame.data, 8);
						dist_data.type = rx_buf[0];
						dist_data.anchor_id = rx_buf[1];
						memcpy(&(dist_data.distance), rx_buf+2, sizeof(dist_data.distance));
						memcpy(&(dist_data.confidence), rx_buf+6, 2);
						ROS_INFO("Distance from node %d to anchor %d: %.3f m", node_id, dist_data.anchor_id, dist_data.distance);
					}else{
						nbytes = 0;
						ROS_INFO("No bytes received");
					}
				}
				UWB_msg msg;
				msg.timestamp = ros::Time::now();
				msg.node_id = node_id;
				msg.anchor_id = dist_data.anchor_id;
				msg.distance = dist_data.distance;
				msg.confidence = dist_data.confidence;

				can_pub.publish(msg);
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
