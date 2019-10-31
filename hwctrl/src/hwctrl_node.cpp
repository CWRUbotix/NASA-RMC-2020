#include <hwctrl.h>

boost::shared_ptr<ros::AsyncSpinner> async_spinner;


void test_cb(const std_msgs::EmptyConstPtr& msg){
  ROS_INFO("TEST 1");
}

void test_cb_2(const std_msgs::EmptyConstPtr& msg){
  ROS_INFO("TEST 2");
}



int main(int argc, char** argv){
	ROS_INFO("Hardware Controller Node");
	ros::init(argc, argv, "hwctrl");
	ros::NodeHandle n;
	ros::Rate loop_rate(1); // 5ms loop rate
	ros::AsyncSpinner spinner(4);

	ros::Publisher limit_switch_pub = n.advertise<hwctrl::LimitSwState>("limit_switch", 16);
	ros::Subscriber test_1_sub = n.subscribe<std_msgs::Empty>("test_1", 3, test_cb);
	ros::Subscriber test_2_sub = n.subscribe<std_msgs::Empty>("test_2", 1, test_cb_2);


	// client which sends commands to drive the VESC's
	ros::ServiceClient set_vesc_client = n.serviceClient<canbus::SetVescCmd>("set_vesc");
	
	// make the HwMotorIf object
	HwMotorIf motor_if;
	motor_if.vesc_client = set_vesc_client;

	// read config csv
	std::string ros_package_path(std::getenv("ROS_PACKAGE_PATH"));
	std::istringstream path_stream(ros_package_path);

	std::string src_dir_path;
	std::getline(path_stream, src_dir_path, ':'); // get the first path
	if(*(src_dir_path.end()) != '/'){
		src_dir_path.push_back('/');
	}
	std::string config_file_path = src_dir_path.append(config_file_fname);

	// make motor structs
	motor_if.get_motors_from_csv(config_file_path);

	// server which provides the set_motor service
	ros::ServiceServer set_motor_srv = n.advertiseService("set_motor", &HwMotorIf::set_motor_callback, &motor_if);

	ROS_INFO("ROS init success");

	ROS_INFO(motor_if.list_motors().c_str());

	// async_spinner.reset(new ros::AsyncSpinner(4));
	// async_spinner->start();
	// MAIN LOOP

	std::thread limit_sw_th_obj(limit_switch_thread, limit_switch_pub);

	spinner.start();

	while(ros::ok()){

	// 	// motor_if.maintain_next_motor();
		ROS_INFO("Main Loop");
		loop_rate.sleep();
	}
	ros::waitForShutdown();
	return 0;
}