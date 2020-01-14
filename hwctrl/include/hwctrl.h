#ifndef HWCTRL_H_
#define HWCTRL_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <message_forward.h>
#include <hwctrl/UwbData.h>
#include <hwctrl/VescData.h>
#include <hwctrl/SetVescCmd.h>
#include <hwctrl/VescData.h>
#include <hwctrl/SetMotor.h>
#include <hwctrl/CanFrame.h>
#include <hwctrl/LimitSwState.h>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <thread>

#include <canbus.h>
#include <parse_csv.h>

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

const std::string config_file_fname = "conf/hw_config.csv";

const std::string vesc_log_fname 	= "hwctrl/vesc_log.csv";

static std::string vesc_log_path 	= "";

static std::vector<CanDevice> can_devices_vect;
static ros::Publisher uwb_pub; 			// so the hwctrl_node can publish uwb frames
static ros::Publisher vesc_data_pub;	// so the hwctrl_node can publish VescData to be consumed by HwMotorIf
static ros::Publisher limit_switch_pub; // so limit switch thread can publish limit switch data

static ros::Subscriber vesc_data_sub;

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
	DEVICE_QUAD_ENC
}DeviceType;

typedef enum InterfaceType {
	IF_NONE,
	IF_CANBUS,
	IF_UART
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
	VescData vesc_data;
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
	uint8_t vesc_rx_buf[1024];
	std::vector<HwMotor>::iterator motor_it;
	ros::NodeHandle nh;
	ros::Publisher can_tx_pub; 		// publisher to publish CAN frames to send out
	ros::Publisher motor_data_pub;	// to publish motor data
	ros::Subscriber can_rx_sub; 	// to get vesc data frames
	ros::Subscriber sensor_data_sub;// to get sensor data
	ros::Subscriber limit_sw_sub; 	// listen for limit switch interrupts
	ros::ServiceServer set_motor_srv; // to provide the set_motor service
	ros::Rate loop_rate(1000); 		// 1ms delay in each loop
public:
	HwMotorIf(ros::NodeHandle);
	std::vector<HwMotor> motors;
	int motor_ind = 0;
	bool set_motor_callback(hwctrl::SetMotor::Request& request, hwctrl::SetMotor::Response& response);
	void add_motor(HwMotor mtr);
	void get_motors_from_csv();
	std::string list_motors();
	void maintain_next_motor();
	int get_num_motors();
	void vesc_data_callback(const boost::shared_ptr<hwctrl::VescData>& msg);
	void can_rx_callback(const boost::shared_ptr<hwctrl::CanFrame>& frame); 	// to process received can frames
	void sensor_data_callback(const hwctrl::SensorData& data);
	void limit_sw_callback(const boost::shared_ptr<hwctrl::LimitSwState>& state);
	HwMotor* get_vesc_from_can_id(int can_id);
};


// class to handle sensor stuff
class SensorIf{
private:
	ros::NodeHandle nh;
	ros::Publisher sensor_data_pub; // send data to rest of ROS system
	ros::Publisher can_tx_pub;		// send data to canbus
	ros::publisher limit_sw_pub; 	// send out limit switch states
	ros::Subscriber can_rx_sub; 	// get data from canbus
	ros::Rate loop_rate(100); 		// 10ms sleep in every loop
public:
	SensorIf(ros::NodeHandle);
	void can_rx_callback(const boost::shared_ptr<hwctrl::CanFrame>& frame); 			// do things when 
};

void maintain_motors_thread(HwMotorIf* motor_if);

#endif
