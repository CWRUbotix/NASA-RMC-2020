#include <hwctrl.h>

int main(int argc, char** argv){
	ROS_INFO("Hardware Controller Node");
	ros::init(argc, argv, "hwctrl");
	ros::NodeHandle n;
	ros::Rate loop_rate(1); // 5ms loop rate
	ros::AsyncSpinner spinner(1); // create multithreaded spinner

	ros::Publisher limit_switch_pub = n.advertise<std_msgs::Int32>("limit_switch", 16);

	// client which sends commands to drive the VESC's
	ros::ServiceClient set_vesc_client = n.serviceClient<canbus::SetVescCmd>("SetVesc");

	// make the HwMotorIf object
	HwMotorIf motor_if;
	motor_if.vesc_client = set_vesc_client;

	// DO FILE STUFF
	std::string ros_package_path(std::getenv("ROS_PACKAGE_PATH"));
	std::istringstream path_stream(ros_package_path);

	std::string src_dir_path;
	std::getline(path_stream, src_dir_path, ':'); // get the first path
	if(*(src_dir_path.end()) != '/'){
		src_dir_path.push_back('/');
	}
	std::string config_file_path = src_dir_path.append(config_file_fname);
	vesc_log_path = src_dir_path.append(vesc_log_fname);

	// VESC DATA SUBSCRIBER
	// ros::Subscriber vesc_data_sub = n.subscribe("VescData", 1, &HwMotorIf::vesc_data_callback, &motor_if);

	// make motor structs
	motor_if.get_motors_from_csv(config_file_path);

	// server which provides the set_motor service
	ros::ServiceServer set_motor_srv = n.advertiseService("SetMotor", &HwMotorIf::set_motor_callback, &motor_if);

	ROS_INFO("ROS init success");

	ROS_INFO(motor_if.list_motors().c_str());

	// MAIN LOOP
	std::thread limit_sw_th_obj(limit_switch_thread, limit_switch_pub);
	std::thread motors_thread(maintain_motors_thread, &motor_if);

	spinner.start();

	ros::waitForShutdown();
	return 0;
}
