#include "threads/sensor_thread.h"

#include "hwctrl.h"

#include <ros/param.h>
#include <ros/spinner.h>

#include <boost/move/unique_ptr.hpp>

#include <string>

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


SensorThread::SensorThread(ros::NodeHandle nh) : m_loop_rate(1000) {
    m_sensors  .reserve(MAX_NUMBER_OF_SENSORS);
    m_uwb_nodes.reserve(4);
}

void SensorThread::configure_from_server() {
    const std::string base = param_base + "/sensor";
    for(auto name = sensor_param_names.begin(); name != sensor_param_names.end(); ++name) {
        const std::string full_name = base + "/" + *name;

        const std::string name_param      = full_name + "/name";
		const std::string type_param      = full_name + "/type";
		const std::string can_id_param    = full_name + "/can_id";
		const std::string interface_param = full_name + "/interface";
		const std::string period_param    = full_name + "/update_period";
		const std::string gpio_param      = full_name + "/gpio";

        // Sensor sensor;
        std::string name_str;
        SensorType type;
        ros::Duration period;
        int id;
        boost::shared_ptr<Gpio> gpio;


        if(m_nh.hasParam(name_param)) {
            m_nh.getParam(name_param, name_str);
        }
        if(m_nh.hasParam(type_param)) {
            std::string type_str;
            m_nh.getParam(type_param, type_str);
            type = get_sensor_type_from_param(type_str);
        }
        if(m_nh.hasParam(period_param)){
            double temp;
			m_nh.getParam(period_param, temp);
			ROS_INFO(" - Found sensor update period: %.3fs", temp);
			period = ros::Duration(temp);
		}
        if(m_nh.hasParam(can_id_param)){
			m_nh.getParam(can_id_param, id);
			ROS_INFO(" - Found CAN id: %d", id);
		}
		if(m_nh.hasParam(gpio_param)){
            std::string gpio_path;
			m_nh.getParam(gpio_param, gpio_path);
			ROS_INFO(" - Found gpio file path: %s", gpio_path.c_str());
            *gpio = Gpio(gpio_path);
		}

        auto sensor = create_sensor_from_values(m_nh, name_str, type, id, period, gpio);
        m_sensors.push_back(*sensor); 
    }
}

boost::shared_ptr<Sensor> SensorThread::create_sensor_from_values(
    ros::NodeHandle nh, std::string name, SensorType type, uint32_t id, ros::Duration period, boost::shared_ptr<Gpio> gpio
)
{
    boost::shared_ptr<Sensor> sensor;
    switch(type) {
        case SensorType::UWB: {
            auto node = boost::make_shared<UwbNode>(nh, name, "", type, id, "localization_data", 1024, period);
            m_uwb_nodes.push_back(*node);
            sensor = node;
            break;
        }
        case SensorType::QUAD_ENC: {
            static uint8_t idx = 0;
            std::string topic("sensor_data");
            auto q = boost::make_shared<QuadEncoder>(nh, name, "", type, id, topic, 128, period);
            sensor = q;
            break;
        }
        case SensorType::LIMIT_SW: {
            static uint8_t idx = 0;
            std::string topic("limit_sw" + std::to_string(++idx) + "_state");
            // check the last two arguments, cant find where theyre defined in hwctrl
            auto sw = boost::make_shared<LimitSwitch>(nh, name, "", type, id, topic, 128, period, *gpio, 0, 0);
            sensor = sw;
            break;
        }
        case SensorType::POWER_SENSE: {
            std::string topic("sensor_data");
            auto ps = boost::make_shared<GenericGpioSensor>(nh, name, "", type, id, topic, 128, period, *gpio);
            sensor = ps;
            break;
        }
        default:
            break;
    };
    return sensor;
}

void SensorThread::sleep() {
    m_loop_rate.sleep();
}

void SensorThread::shutdown() {
    // something maybe?
}

void SensorThread::setup_sensors() {
    for(Sensor sensor : m_sensors) {
        sensor.setup();
    }
}

void SensorThread::update_sensors() {
    for(Sensor sensor : m_sensors) {
        if(sensor.ready_to_update()) sensor.update();
    }
}


// run thread
void SensorThread::operator()() {
    ROS_INFO("Starting sensor_thread");
    m_nh.setCallbackQueue(&m_cb_queue);
    ros::AsyncSpinner spinner(1, &m_cb_queue);

    setup_sensors();
    
    while(ros::ok()) {
        update_sensors();
        sleep();   
    }
    shutdown();
}