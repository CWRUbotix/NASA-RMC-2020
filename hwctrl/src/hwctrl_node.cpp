#include <hwctrl.h>

int main(int argc, char** argv){
	ROS_INFO("Hardware Controller Node");
	ros::init(argc, argv, "hwctrl");
	ros::NodeHandle n;
	ros::Rate loop_rate(200); // 5ms loop rate


	// client which sends commands to drive the VESC's
	ros::ServiceClient set_vesc_client = n.serviceClient<canbus::set_vesc_cmd>("set_vesc");
	
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

	// MAIN LOOP
	while(ros::ok()){

		motor_if.maintain_next_motor();

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}