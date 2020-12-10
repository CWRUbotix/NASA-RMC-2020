#pragma once

#include <string>
#include <vector>

#define DEFAULT_MAX_ACCEL 30.0
#define DEFAULT_MAX_RPM 50.0
#define MOTOR_LOOP_PERIOD 0.005

#define NUMBER_OF_SPI_DEVICES 4
#define MAX_NUMBER_OF_SENSORS 32

#define SENSOR_SAMPLES 5
namespace hwctrl {

const std::string spidev_path   = "/dev/spidev1.0";
const std::string sys_gpio_base = "/sys/class/gpio/";
const std::string cal_file_default = "sensor_calibration.dat";  // this should be created in the HOME directory

const std::string param_base = "/hardware";

const std::vector<std::string> motor_param_names{
    "port_drive",   "starboard_drive",  "dep", "exc_belt", "exc_translation",
    "exc_port_act", "exc_starboard_act"};


const std::vector<std::string> sensor_param_names{"uwb_node_1",
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
                                                  "limit_4",
                                                  "power_sense", "estop"};
}
