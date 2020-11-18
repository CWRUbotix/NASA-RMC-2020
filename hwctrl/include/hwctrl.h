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