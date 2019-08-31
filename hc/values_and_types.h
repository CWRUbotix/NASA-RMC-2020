

#ifndef VALUES_AND_TYPES_H_
#define VALUES_AND_TYPES_H_

#include <SPI.h>
#include "VESC/VESC.h"
// #include "LSM6DS3.h"
#include "SparkFunLSM6DS3.h"
#include "ADS1120.h"
#include "Herkulex.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <XYZrobotServo.h>
////////////////////////////////////////////////////////////////////////////////
//	PREPROCESSOR DEFINES
////////////////////////////////////////////////////////////////////////////////
#define NUM_SENSORS 	33
#define NUM_SENSOR_VALS 33
#define NUM_MOTORS 		9
#define NUM_DEVICES 	15
#define NUM_SPI_DEVICES 9

#define CLIENT 			Serial
#define CMD_MAX_LEN 	1024
#define RPY_MAX_LEN 	1024
#define HEADER_LEN 		5
#define CMD_BLOCK_LEN 	5
#define RPY_BLOCK_LEN 	5
#define RPY_BLOCK_LEN_L 9

#define NUM_PREV_VALUES 		20

#define DEBUG_SELECT_PIN 		2
#define ADC_2_CS_PIN 			14
#define ADC_1_CS_PIN 			15
#define ADC_0_CS_PIN 			16
#define DAC_2_CS_PIN 			17
#define DAC_1_CS_PIN 			18
#define DAC_0_CS_PIN 			19
#define ENCODER_CS_PIN 			22
#define ENCODER_INDEX_PIN 		23
#define ENCODER_A_PIN 			40
#define ENCODER_B_PIN 			41
#define EXC_ROT_FORE_LIM_PIN 	24
#define EXC_ROT_AFT_LIM_PIN 	25
#define EXC_TRANS_LOWER_LIM_PIN	29
#define EXC_TRANS_UPPER_LIM_PIN	30
#define DEP_LOWER_LIM_PIN 		36
#define IMU_0_CS_PIN 			35
#define IMU_1_CS_PIN 			38
#define DEP_UPPER_LIM_PIN 		37
#define ESTOP_SENSE_PIN 		39

#define EXC_NORMAL_POWER 		300.0
#define EXC_LOW_POWER 			200.0

#define LIN_ACT_ERR_MARGIN 		0.5
#define LIN_ACT_KP				400.0
#define LIN_ACT_KI 				0.0
#define LIN_ACT_KP_INC 			75.0
#define MOTOR_ANLG_CENTER_V 	2.5
#define DAC8551_SPEED 			2500000
#define ADS1120_SPEED 			2500000
#define IMU_SPEED 				2500000
#define ENCODER_SPEED 			1000000
#define DEBUG_SPEED 			1000000

#define LOOKY_SERIAL 			Serial6
#define DEBUG 					Serial5

#define NOP3 			"nop\n\t""nop\n\t""nop\n\t"
// should pause about 50 ns or so
#define PAUSE_SHORT 	__asm__(NOP3 NOP3 NOP3)

#define ROT_ENC_PPR 			2048.0
#define ROT_ENC_DEG_PER_PULSE 	360.0/ROT_ENC_PPR
#define ROT_ENC_RD_POS 			0x10
#define EXC_MM_PER_ROT 			60.96
////////////////////////////////////////////////////////////////////////////////
//  DEFINE TYPES
////////////////////////////////////////////////////////////////////////////////
enum DEVICE_INDICES {
	ADC_0,
	ADC_1,
	ADC_2,
	DAC_0,
	DAC_1,
	DAC_2,
	TRANS_ENCODER,
	IMU_0,
	IMU_1,
	VESC_1,
	VESC_2,
	VESC_3,
	VESC_4,
	LOOKY_0,
	LOOKY_1
};

enum SENSOR_INDICES {
	DRIVE_PORT_ENC,
	DRIVE_STBD_ENC,
	DEP_WINCH_ENC,
	EXC_BELT_ENC,
	EXC_TRANS_ENC,
	EXC_ROT_PORT_ENC,
	EXC_ROT_STBD_ENC,
	LOOKY_PORT_ENC,
	LOOKY_STBD_ENC,
	DEP_LOAD_CELL,
	EXC_LOAD_CELL,
	GYRO_0_X,
	GYRO_0_Y,
	GYRO_0_Z,
	ACCEL_0_X,
	ACCEL_0_Y,
	ACCEL_0_Z,
	GYRO_1_X,
	GYRO_1_Y,
	GYRO_1_Z,
	ACCEL_1_X,
	ACCEL_1_Y,
	ACCEL_1_Z,
	DEP_LIMIT_LOWER,
	DEP_LIMIT_UPPER,
	EXC_LIMIT_FORE,
	EXC_LIMIT_AFT,
	EXC_CONV_LIMIT_LOWER,
	EXC_CONV_LIMIT_UPPER,
	ESTOP_SENSE_INDEX,
	ANGULAR_DISP_X,
	ANGULAR_DISP_Y,
	ANGULAT_DISP_Z
};

enum MOTOR_INDICES {
	PORT_VESC,
	STBD_VESC,
	DEP_VESC,
	EXC_VESC,
	EXC_TRANS,
	EXC_ROT_PORT,
	LOOKY_PORT,
	LOOKY_STBD,
	EXC_ROT_STBD
};

enum COMMANDS {
	CMD_SET_OUTPUTS = 0x51,
	CMD_READ_VALUES = 0x52,
	CMD_TEST 		= 0x53,
	CMD_T_SYNC 		= 0x54
};

enum REPLYS {
	RPY_SET_OUTPUTS 	= CMD_SET_OUTPUTS,
	RPY_READ_VALUES 	= CMD_READ_VALUES,
	RPY_TEST 			= CMD_TEST,
	RPY_T_SYNC 			= CMD_T_SYNC,
	RPY_INVALID_CMD 	= 0xA1,
	RPY_LEN_MISMATCH 	= 0xA2,
	RPY_CHKSUM_MISMATCH = 0xA3
};

typedef enum FAULT {
	NO_FAULT,
	INVALID_CMD,
	LEN_MISMATCH,
	CHKSUM_MISMATCH
} FAULT_T;

typedef enum Interface {
	NONE,
	DIGITAL_IO,
	LOOKY_UART,
	VESC_UART,
	SPI_BUS
} Interface;

typedef enum SensorType {
	SENS_NONE,
	SENS_DIGITAL_INPUT,
	SENS_LIMIT,
	SENS_LOAD_CELL,
	SENS_GYRO,
	SENS_ACCEL,
	SENS_ROT_ENC,
	SENS_BLDC_ENC,
	SENS_POT_ENC,
	SENS_LOOKY_ENC
} SensorType;

typedef enum MotorType {
	MTR_NONE,
	MTR_VESC,
	MTR_SABERTOOTH,
	MTR_SABERTOOTH_RC,
	MTR_LOOKY
} MotorType;

typedef struct Device{
	Interface interface = NONE;
	SPISettings* spi_settings;
	ADS1120* adc;
	VESC* vesc;
	LSM6DS3* imu;
	HardwareSerial* serial;
	uint8_t spi_cs 		= 0;
	uint8_t id 			= 0;
	bool is_setup 		= false; // field to prevent unnecessary setup or doomed reading
}Device;

typedef struct SensorInfo SensorInfo;
typedef struct MotorInfo MotorInfo;

struct SensorInfo{
	SensorType type 	= SENS_NONE;
	Device* device;
	MotorInfo* motor;
	char* name = "--- NO NAME ----";
	uint8_t pin 		= 0;
	int n_value; 				// holds any relevant integer value
	float value; 				// holds the value to be sent over USB, updated at t_stamp
	float last_value; 			// holds the last value for whatever
	float* prev_values; 		// points to an array of previous values, for filtering, etc.
	float baseline; 			// 
	float last_baseline; 		// 
	float peak_value; 			// 
	uint8_t val_ind 	= 0; 	// index in this array
	bool value_good;
	int t_stamp; 				// update time-stamp
	uint8_t adc_channel_config;
	float slope 		= 1.0; 	// linear equation coefficient
	float offset 		= 0.0; 	// linear equation offset
	float rots;
	float (*get_value)(void);
	char imu_axis;
	float min;
	float max;
	int allowed_dir;
};

struct MotorInfo{
	MotorType type 		= MTR_NONE;
	Device* device;
	SensorInfo* sensor;
	SensorInfo* limit_1;
	SensorInfo* limit_2;
	char* name = "--- NO NAME ----";
	float setpt 		= 0.0; 	
	float last_setpt 	= 0.0;
	float last_rpm 		= 0.0;
	float rpm_factor 	= 1.0;
	float volts 		= MOTOR_ANLG_CENTER_V;
	long t_stamp 		= 0; 	// time-stamp of last update
	float kp 			= 0.0;
	float ki 			= 0.0;
	float err_margin 	= 0.0;
	float integ 		= 0.0;
	float max_integ 	= 0.0;
	float max_setpt 	= 0.0;
	float min_setpt 	= 0.0;
	float max_delta 	= 0.0;
	float deadband 		= 0.0;
	int looky_id 		= 0;
	float power 		= 0.0;
	float max_power 	= 0.0;
	float min_power 	= 0.0;
};

////////////////////////////////////////////////////////////////////////////////
//
//  GLOBAL VARIABLES
//
////////////////////////////////////////////////////////////////////////////////
SensorInfo 	sensor_infos 	[NUM_SENSORS] 		= {};
MotorInfo 	motor_infos 	[NUM_MOTORS] 		= {};
Device 		device_infos 	[NUM_DEVICES] 		= {};
Device 		SPI_devices 	[NUM_SPI_DEVICES] 	= {};
float 		exc_lc_values 	[NUM_PREV_VALUES] 	= {};
float 		dep_lc_values 	[NUM_PREV_VALUES] 	= {};
float 		accel_0_x_values[NUM_PREV_VALUES] 	= {};
float 		accel_0_y_values[NUM_PREV_VALUES] 	= {};
float 		accel_0_z_values[NUM_PREV_VALUES] 	= {};
float 		accel_1_x_values[NUM_PREV_VALUES] 	= {};
float 		accel_1_y_values[NUM_PREV_VALUES] 	= {};
float 		accel_1_z_values[NUM_PREV_VALUES] 	= {};
float 		gyro_0_x_values [NUM_PREV_VALUES] 	= {};
float 		gyro_0_y_values [NUM_PREV_VALUES] 	= {};
float 		gyro_0_z_values [NUM_PREV_VALUES] 	= {};
float 		gyro_1_x_values [NUM_PREV_VALUES] 	= {};
float 		gyro_1_y_values [NUM_PREV_VALUES] 	= {};
float 		gyro_1_z_values [NUM_PREV_VALUES] 	= {};


volatile int encoder_rots = 0;
volatile int encoder_A_pulses = 0;

bool estop_state 		= false; 	// false means off
bool estop_state_last 	= false; 	// false means off

int t_micros 			= 0; 		// will be updated with micros()
int t_offset 			= 0; 		// offset = micros() - t_sync,   t_stamp = micros() - t_offset

ADS1120 adc0(ADC_0_CS_PIN);
ADS1120 adc1(ADC_1_CS_PIN);
ADS1120 adc2(ADC_2_CS_PIN);


// XYZrobotServo looky_servo_port(&Serial6, 128);
// XYZrobotServo looky_servo_starboard(&Serial6, 129);
// XYZrobotServoStatus looky_servoStatus_port;
// XYZrobotServoStatus looky_servoStatus_starboard;

VESC vesc1(&Serial1);
VESC vesc2(&Serial2);
VESC vesc3(&Serial3);
VESC vesc4(&Serial4);

LSM6DS3 imu0(SPI_MODE, IMU_0_CS_PIN);
LSM6DS3 imu1(SPI_MODE, IMU_1_CS_PIN);

SPISettings DAC_SPI_settings(DAC8551_SPEED, MSBFIRST, SPI_MODE1);
SPISettings ADC_SPI_settings(ADS1120_SPEED, MSBFIRST, SPI_MODE1);
SPISettings IMU_SPI_settings(IMU_SPEED, MSBFIRST, SPI_MODE0);
SPISettings Encoder_SPI_settings(ENCODER_SPEED, MSBFIRST, SPI_MODE0);
SPISettings Debug_SPI_settings(DEBUG_SPEED, MSBFIRST, SPI_MODE0);

#define TIME_STAMP (micros() - t_offset)

int get_sign(float f){
	return (f < 0.0 ? -1 : (f > 0.0 ? 1 : 0));
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float fconstrain(float f, float a, float b){
	if(f < a){
		return a;
	}else if(f > b){
		return b;
	}else{
		return f;
	}
}

float max_in_array(float* arr, int len){
	float max = 0.0;
	for(int i = 0; i<len; i++){
		if(arr[i] > max){
			max = arr[i];
		}
	}
	return max;
}

float faverage(float* arr, int len){
	float avg = 0.0;
	for(int i = 0; i<len; i++){
		avg += arr[i]/len;
	}
	return avg;
}

#endif
