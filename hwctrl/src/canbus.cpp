#include <canbus.h>

/**
 * intended to be an argument when instantiating a std::thread
 */
void canbus_thread(CanbusIf* canbus_if){
	ROS_DEBUG("Starting canbus_thread");
	ros::AsyncSpinner spinner(1, &(canbus_if->cb_queue));
	spinner.start();
	while(ros::ok()){
		int frames_sent = canbus_if->read_can_frames();
		if(frames_sent > 0){
			//ROS_INFO("Read %d CAN frames", frames_sent);
		}
		canbus_if->loop_rate.sleep(); //
	}
	canbus_if->shutdown();
	ROS_DEBUG("Exiting canbus_thread.");
}

/**
 * Constructor for the CanbusIf object
 * initializes the can socket, creates publisher and subscriber for CanFrame messages
 */
CanbusIf::CanbusIf(ros::NodeHandle n)
: loop_rate(100) {
	//this->nh = n;

	this->nh.setCallbackQueue(&(this->cb_queue)); // have this copy use this callback queue

	int canbus_init = this->init(); // do setup things

	if(canbus_init != 0){
		ROS_ERROR("Canbus init failed: Error code %d", canbus_init);
	}else{
		ROS_DEBUG("Canbus init success");
	}

	// PUBLISHERS
	this->can_rx_pub 	= this->nh.advertise<hwctrl::CanFrame>("can_frames_rx", 128);

	// SUBSCRIBERS
	this->can_tx_sub 	= this->nh.subscribe("can_frames_tx", 128, &CanbusIf::can_tx_cb, this);
}

/**
 * all operating system level setup for CAN bus
 * puts the io handle for the CAN socket in this->sock
 * will set this->sock_ready if setup was successful
 */
int CanbusIf::init(){
	struct sockaddr_can addr;
	struct can_frame frame;
	struct can_frame rx_frame;
	struct can_frame tx_frame;
	struct ifreq ifr;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000; // 10ms CAN bus read timeout

	const char* ifname = "can1";

	// initialize the CAN socket
	if((this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
		ROS_ERROR("Error while opening CAN socket");
		this->sock_ready = false;
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(this->sock, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	ROS_DEBUG("%s at index %d", ifname, ifr.ifr_ifindex);

	setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

	// socket bind
	if(bind(this->sock, (struct sockaddr *)&addr, sizeof(addr)) < 0){
		ROS_ERROR("Error in CAN socket bind");
		this->sock_ready = false;
		return -2;
	}
	this->sock_ready = true;
	return 0; // success
}

/**
 * needs to be called periodically to read in frames received on CAN bus
 * it simply publishes each new frame to the can_rx_pub (topic "can_frames_rx")
 * @return number of CAN frames read and published
 */
int CanbusIf::read_can_frames(){
	//ROS_INFO("Reading CAN frames");
	if(!this->sock_ready){
		return -1;
	}
	int retval = 0;
	int nbytes = 0;
	struct can_frame rx_frame;
	// BRING RECEIVED FRAMES INTO USER SPACE
	while((nbytes = read(this->sock, &rx_frame, sizeof(struct can_frame))) > 0){
		boost::shared_ptr<hwctrl::CanFrame> frame_msg(new hwctrl::CanFrame());
		frame_msg->can_id = rx_frame.can_id;
		frame_msg->can_dlc = rx_frame.can_dlc;
		for(int i = 0; i < rx_frame.can_dlc; i++){
			frame_msg->data[i] = rx_frame.data[i];
		}
		// ROS_INFO("Publishing to can_frames_rx");
		can_rx_pub.publish(frame_msg); 	// publish message immediately
		retval++;
	}
	return retval;
}

void CanbusIf::can_tx_cb(boost::shared_ptr<hwctrl::CanFrame> frame){
	// ROS_INFO("Writing frame to CAN bus");
	struct can_frame f;
	f.can_id = frame->can_id;
	f.can_dlc = frame->can_dlc;
	for(int i = 0; i < frame->can_dlc; i++){
		f.data[i] = frame->data[i];
	}
	// WRITE CAN FRAME TO SOCKET IMMEDIATELY
	if(this->sock_ready){
		int n_bytes = write(this->sock, &f, sizeof(f));
	}
}

void CanbusIf::shutdown(void){
	close(this->sock);
}
