#include <hwctrl.h>

int main(int argc, char** argv){
	ROS_INFO("Hardware Controller Node");
	ros::init(argc, argv, "hwctrl");
	ros::NodeHandle n;
	ros::Rate loop_rate(1); // 5ms loop rate

	// CREATE THE CanbusIf OBJECT
	CanbusIf canbus_if(n);

	// CREATE THE SensorIf OBJECT
	SensorIf sensor_if(n);

	// CREATE THE HwMotorIf OBJECT
	HwMotorIf motor_if(n);

	ROS_INFO("ROS init success");

	ROS_INFO(motor_if.list_motors().c_str());

	// create spinners for each thread
	ros::AsyncSpinner canbus_spinner(1, &(canbus_if.cb_queue));
	ros::AsyncSpinner sensor_spinner(1, &(sensor_if.cb_queue));
	ros::AsyncSpinner motor_spinner(1, &(motor_if.cb_queue));

	// start threads
	std::thread sensor_thread_obj(sensors_thread, &sensor_if);
	std::thread motors_thread_obj(maintain_motors_thread, &motor_if);
	std::thread canbus_thread_obj(canbus_thread, &canbus_if);

	// start the ROS spinners
	canbus_spinner.start();
	sensor_spinner.start();
	motor_spinner.start();

	ros::waitForShutdown();
	return 0;
}
