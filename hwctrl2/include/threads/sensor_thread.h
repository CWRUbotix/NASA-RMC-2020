#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/utility/string_view.hpp>

#include <vector>

#include "hardware/sensor.h"
#include "hardware/quad_encoder.h"
#include "hardware/uwb.h"
#include "hardware/ads1120.h"
#include "hardware/adt7310.h"
#include "hardware/lsm6ds3.h"

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

class SensorThread {
public:
    SensorThread(ros::NodeHandle nh);
    ~SensorThread() = default;

    void configure_from_server(boost::shared_ptr<Spi> spi);
    // void configure_from_csv();

    void setup_sensors();
    void update_sensors();

    void shutdown();
    void sleep();

    void operator()();
private:
    boost::shared_ptr<Sensor> create_sensor_from_values(
        ros::NodeHandle nh, std::string name, SensorType type, uint32_t id,
        ros::Duration period, boost::shared_ptr<Gpio> gpio, boost::shared_ptr<Spi> spi
    );
private:
    ros::NodeHandle    m_nh;
    ros::Rate          m_loop_rate;
    ros::CallbackQueue m_cb_queue;

    std::vector<Sensor>  m_sensors;
    std::vector<UwbNode> m_uwb_nodes;
    uint32_t             m_uwb_ind;
};
