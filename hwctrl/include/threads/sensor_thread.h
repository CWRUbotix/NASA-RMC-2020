#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/timer.h>
#include <ros/param.h>
#include <ros/spinner.h>

#include <boost/utility/string_view.hpp>
#include <boost/move/unique_ptr.hpp>
#include <boost/move/make_unique.hpp>
#include <boost/filesystem.hpp>

#include "hwctrl.h"
#include "hardware/sensor.h"
#include "hardware/quad_encoder.h"
#include "hardware/uwb.h"
#include "hardware/ads1120.h"
#include "hardware/adt7310.h"
#include "hardware/lsm6ds3.h"

#include <string>
#include <utility>
#include <vector>
#include <thread>



using boost::movelib::unique_ptr;
using boost::movelib::make_unique;

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
	"limit_4",
    "power_sense"
};

class SensorThread {
public:
    SensorThread(ros::NodeHandle nh);
    virtual ~SensorThread() = default;

    void configure_from_server(boost::shared_ptr<Spi> spi);

    // Dont think we need this anymore
    // void configure_from_csv();

    void setup_sensors();
    void update_sensors();

    void shutdown();
    void sleep();

    virtual void operator()();
private:
    boost::shared_ptr<Sensor> create_sensor_from_values(
        ros::NodeHandle nh, std::string name, std::string topic, SensorType type, uint32_t id,
        ros::Duration period, unique_ptr<Gpio> gpio, boost::shared_ptr<Spi> spi
    );

    void uwb_ping_callback(const ros::TimerEvent&);
protected:
    ros::NodeHandle    m_nh;
    ros::Rate          m_loop_rate;
    ros::CallbackQueue m_cb_queue;

    using SensorVec = std::vector<boost::shared_ptr<Sensor>>;
    using SensorVecIter = SensorVec::iterator;

    SensorVec  m_sensors;

    std::vector<boost::shared_ptr<UwbNode>> m_uwb_nodes;
    uint32_t             m_uwb_idx;
    ros::Duration        m_uwb_update_pd;
    ros::Timer           m_uwb_update_timer;
};
