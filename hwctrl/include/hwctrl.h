#ifndef HWCTRL_H_
#define HWCTRL_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <ros/message_forward.h>
#include <hwctrl/UwbData.h>
#include <hwctrl/VescData.h>
#include <hwctrl/SetMotor.h>
#include <hwctrl/CanFrame.h>
#include <hwctrl/SensorData.h>
#include <hwctrl/LimitSwState.h>
#include <hwctrl/MotorData.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <thread>

#include <canbus.h>
#include <uwb.h>
#include <parse_csv.h>
#include <spi.h>

#define DEFAULT_MAX_ACCEL 	30.0
#define DEFAULT_MAX_RPM 	50.0
#define MOTOR_LOOP_PERIOD 	0.005

const std::string id_hddr 			= "id";
const std::string name_hddr 		= "name";
const std::string category_hddr 	= "category";
const std::string device_type_hddr 	= "device_type";
const std::string interface_hddr 	= "interface";
const std::string device_id_hddr 	= "device_id";
const std::string aux_1_hddr 		= "aux_1";
const std::string aux_2_hddr 		= "aux_2";
const std::string aux_3_hddr 		= "aux_3";
const std::string aux_4_hddr 		= "aux_4";

const std::string category_motor 	= "motor";
const std::string category_sensor = "sensor";

const std::string config_file_fname = "conf/hw_config.csv";

const std::string vesc_log_fname 	= "hwctrl/vesc_log.csv";

static std::string vesc_log_path 	= "";

const std::string adc_1_cs 			= "/sys/class/gpio/gpio67/"; 		// gpio file for ADC 1 chip select
const std::string adc_2_cs			= "/sys/class/gpio/gpio68/";
const std::string temp_sensor_cs= "/sys/class/gpio/gpio69/";
const std::string imu_cs 				= "/sys/class/gpio/gpio66/";
const std::string sys_power_on 	= "/sys/class/gpio/gpio45/";
const std::string ext_5V_ok 		= "/sys/class/gpio/gpio26/";
const std::string limit_1_gpio 	= "/sys/class/gpio/gpio60/";
const std::string limit_2_gpio 	= "/sys/class/gpio/gpio48/";
const std::string limit_3_gpio 	= "/sys/class/gpio/gpio49/";
const std::string limit_4_gpio 	= "/sys/class/gpio/gpio117/";
const std::string limit_5_gpio 	= "/sys/class/gpio/gpio115/";
const std::string limit_6_gpio 	= "/sys/class/gpio/gpio20/";

typedef enum MotorType {
	MOTOR_NONE,
	MOTOR_VESC,
	MOTOR_BRUSHED
}MotorType;

typedef enum DeviceType {
	DEVICE_NONE,
	DEVICE_UWB,
	DEVICE_VESC,
	DEVICE_SABERTOOTH,
	DEVICE_QUAD_ENC,
	DEVICE_LIMIT_SW
}DeviceType;

typedef enum InterfaceType {
	IF_NONE,
	IF_CANBUS,
	IF_UART,
	IF_SPI,
	IF_GPIO
}InterfaceType;

typedef enum ControlType {
	CTRL_NONE,
	CTRL_RPM,
	CTRL_POSITION
}ControlType;

// class to hold info about a motor
class HwMotor{
private:
	char scratch_buf[1024];
public:
	bool online = false;
	int id;
	std::string name;
	int device_id; // could be the CAN id, or something else
	VescData vesc_data; // struct to hold VESC data
	float rpm_coef;
	ControlType ctrl_type = CTRL_NONE;
	DeviceType motor_type = DEVICE_NONE;
	InterfaceType if_type = IF_NONE;
	ros::Time update_t;
	float setpoint = 0.0;
	float last_setpoint = 0.0;
	float accel_setpoint = DEFAULT_MAX_ACCEL;
	float max_rpm 	= DEFAULT_MAX_RPM;
	float max_accel = DEFAULT_MAX_ACCEL;
	std::string if_name;
	std::string to_string();
};

InterfaceType get_if_type(std::string type_str);
DeviceType get_device_type(std::string type_str);

// class to manage the interfaces to motors, be it canbus, uart, etc.
class HwMotorIf{
private:
	uint8_t vesc_rx_buf[1024]; // deprecated do not use
	std::vector<HwMotor>::iterator motor_it;
	ros::NodeHandle nh;
	ros::Publisher can_tx_pub; 		// publisher to publish CAN frames to send out
	ros::Publisher motor_data_pub;	// to publish motor data
	ros::Subscriber can_rx_sub; 	// to get vesc data frames
	ros::Subscriber sensor_data_sub;// to get sensor data
	ros::Subscriber limit_sw_sub; 	// listen for limit switch interrupts
	ros::ServiceServer set_motor_srv; // to provide the set_motor service
public:
	HwMotorIf(ros::NodeHandle);
	ros::CallbackQueue cb_queue;
	ros::Rate loop_rate; 		// 1ms delay in each loop
	std::vector<HwMotor> motors;
	int motor_ind = 0;
	bool set_motor_callback(hwctrl::SetMotor::Request& request, hwctrl::SetMotor::Response& response);
	void add_motor(HwMotor mtr);
	void get_motors_from_csv();
	std::string list_motors();
	void maintain_next_motor();
	int get_num_motors();
	void vesc_data_callback(boost::shared_ptr<hwctrl::VescData> msg);
	void can_rx_callback(boost::shared_ptr<hwctrl::CanFrame> frame); 	// to process received can frames
	void sensor_data_callback(hwctrl::SensorData data);
	void limit_sw_callback(boost::shared_ptr<hwctrl::LimitSwState> state);
	HwMotor* get_vesc_from_can_id(int can_id);
};


// class to handle sensor stuff
class SensorIf{
private:
	ros::NodeHandle nh;
	ros::Subscriber can_rx_sub; 	// get data from canbus
	int spi_handle; 							// for the spi interface
	ros::Timer uwb_update_timer;  // when to call the uwb_update_callback
	int uwb_ind = 0;
//	QuadEncoder quad_encoder; 		// object for our quadrature encoder
	void get_sensors_from_csv();
public:
	SensorIf(ros::NodeHandle);
	UwbNode* get_uwb_by_can_id(int can_id);
	ros::CallbackQueue cb_queue;
	ros::Publisher sensor_data_pub; // send data to rest of ROS system
	ros::Publisher can_tx_pub;		// send data to canbus
	ros::Publisher limit_sw_pub; 	// send out limit switch states
	ros::Publisher uwb_data_pub; 	// publish uwb data
	std::vector<UwbNode> uwb_nodes; // holds all the UWB nodes on the robot
	ros::Duration uwb_update_period; // how long to wait before we request data from next UWB node
	ros::Rate loop_rate; 		// 10ms sleep in every loop
	void can_rx_callback(boost::shared_ptr<hwctrl::CanFrame> frame); 			// do things when
	int setup_gpio();
	void uwb_update_callback(const ros::TimerEvent&);
};

void maintain_motors_thread(HwMotorIf* motor_if);
void sensors_thread(SensorIf* sensor_if);

#endif
