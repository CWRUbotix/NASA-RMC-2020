#pragma once

#include <string>
#include <vector>
#include <array>

#define DEFAULT_MAX_ACCEL 			30.0
#define DEFAULT_MAX_RPM 			50.0
#define MOTOR_LOOP_PERIOD 			0.005

#define NUMBER_OF_SPI_DEVICES   	4
#define MAX_NUMBER_OF_SENSORS   	32

#define SENSOR_SAMPLES 			    5

#define ADC_1_IND 					0
#define ADC_2_IND 					1
#define TEMP_SENSOR_IND 			2
#define IMU_IND 					3

namespace paths {
    const std::string spidev_path    	= "/dev/spidev1.0";
    const std::string sys_gpio_base     = "/sys/class/gpio/";
    const std::string cal_file_default 	= "sensor_calibration.dat"; // this should be created in the HOME directory
    const std::string adc_1_cs 			= "/sys/class/gpio/gpio67/"; 		// gpio file for ADC 1 chip select
    const std::string adc_2_cs			= "/sys/class/gpio/gpio68/";
    const std::string temp_sensor_cs    = "/sys/class/gpio/gpio69/";
    const std::string imu_cs 			= "/sys/class/gpio/gpio66/";
    const std::string sys_power_on 	    = "/sys/class/gpio/gpio45/"; 		// 24V_PRESENT
    const std::string ext_5V_ok 		= "/sys/class/gpio/gpio26/";
    const std::string limit_1_gpio 	    = "/sys/class/gpio/gpio60/";
    const std::string limit_2_gpio    	= "/sys/class/gpio/gpio48/";
    const std::string limit_3_gpio   	= "/sys/class/gpio/gpio49/";
    const std::string limit_4_gpio   	= "/sys/class/gpio/gpio117/";
    const std::string limit_5_gpio 	    = "/sys/class/gpio/gpio115/";
    const std::string limit_6_gpio 	    = "/sys/class/gpio/gpio20/";
    const std::string over_temp_gpio    = "/sys/class/gpio/gpio27/";
    const std::string crit_temp_gpio    = "/sys/class/gpio/gpio61/";
}

const std::string param_base = "/hardware";

const std::vector<std::string> sensor_param_names{
	"uwb_node_1",
	"uwb_node_2",
	"uwb_node_3",
	"uwb_node_4",
	"ebay_temperature",
    "quad_encoder_1",
	"imu",
	"adc_1",
	"adc_2",
	"limit_1",
	"limit_2",
	"limit_3",
	"limit_4"
};

const std::vector<std::string> motor_param_names{
	"port_drive",
	"starboard_drive",
	"dep",
	"exc_belt",
	"exc_translation",
	"exc_port_act",
	"exc_starboard_act"
};