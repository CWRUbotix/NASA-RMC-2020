#include "pch.h"
#include "threads/sensor_thread.h"

#include "hwctrl.h"

#include <ros/param.h>
#include <ros/spinner.h>

SensorThread::SensorThread(ros::NodeHandle nh) : m_nh(nh), m_loop_rate(1000), m_uwb_update_pd(0.01), m_uwb_idx(0) {
    m_sensors  .reserve(MAX_NUMBER_OF_SENSORS);
    m_uwb_nodes.reserve(4);

    m_nh.setCallbackQueue(&m_cb_queue);

    // initalize spi
    // needs to be available to all sensors since all sensors are on same spi line
    ROS_DEBUG("Initializing SPI bus from %s...", paths::spidev_path.c_str());
    auto spi = boost::make_shared<Spi>(paths::spidev_path);
    if(spi->has_error()) {
        ROS_ERROR("SPI init failed :(");
    } else {
        ROS_INFO("SPI bus initialization on %s presumed successful", paths::spidev_path.c_str());
    }

    configure_from_server(spi);

    // read calibrations
    std::istringstream path_stream(std::string(std::getenv("HOME")));
	std::string cal_file_path;
	std::getline(path_stream, cal_file_path, ':'); // get the first path
	if(*(cal_file_path.end()) != '/'){
		cal_file_path.push_back('/');
	}
	cal_file_path.append(paths::cal_file_default.c_str());

	if(boost::filesystem::exists(cal_file_path.c_str())){
		ROS_INFO("Loading sensor calibrations from %s...", cal_file_path.c_str());
		std::vector<Calibration> cals;
		read_calibration(cal_file_path, cals);

		for(auto cal : cals){
			SensorVecIter sensor = m_sensors.begin();
			size_t pos = std::string::npos;
			// if sensor name exists within calibration name
			while( (pos = cal.name.find((*sensor)->get_name())) == std::string::npos && sensor != m_sensors.end()){
				sensor++;
			}
			if(pos != std::string::npos){
				(*sensor)->add_calibration(cal);
			}
		}
	}

    // start uwb updates
    m_uwb_update_timer = m_nh.createTimer(m_uwb_update_pd, &SensorThread::uwb_ping_callback, this);
}

void SensorThread::uwb_ping_callback(const ros::TimerEvent&) {
    m_uwb_nodes.at(m_uwb_idx++)->ping();
    m_uwb_idx %= m_uwb_nodes.size();
}

void SensorThread::configure_from_server(boost::shared_ptr<Spi> spi) {
    const std::string base = param_base + "/sensor";
    for(auto name = sensor_param_names.begin(); name != sensor_param_names.end(); ++name) {
        const std::string full_name = base + "/" + *name;

        const std::string name_param      = full_name + "/name";
        const std::string topic_param     = full_name + "/topic";
		const std::string type_param      = full_name + "/type";
		const std::string can_id_param    = full_name + "/can_id";
		const std::string period_param    = full_name + "/update_period";
		const std::string gpio_param      = full_name + "/gpio";

        // Sensor sensor;
        std::string name_str;
        std::string topic_str;
        SensorType type;
        ros::Duration period;
        int can_id;
        unique_ptr<Gpio> gpio;


        if(m_nh.hasParam(name_param)) {
            m_nh.getParam(name_param, name_str);
        }
        if(m_nh.hasParam(topic_param)) {
            m_nh.getParam(topic_param, topic_str);
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
			m_nh.getParam(can_id_param, can_id);
			ROS_INFO(" - Found CAN id: %d", can_id);
		}
		if(m_nh.hasParam(gpio_param)){
            std::string gpio_path;
			m_nh.getParam(gpio_param, gpio_path);
			ROS_INFO(" - Found gpio file path: %s", gpio_path.c_str());
            gpio = boost::movelib::make_unique<Gpio>(gpio_path);
		}

        auto sensor = create_sensor_from_values(m_nh, topic_str, name_str, type, can_id, period, std::move(gpio), spi);
        if(sensor != nullptr)
            m_sensors.push_back(sensor); 
    }
}

boost::shared_ptr<Sensor> SensorThread::create_sensor_from_values(
    ros::NodeHandle nh, std::string name, std::string topic, SensorType type, uint32_t can_id, ros::Duration period,
    unique_ptr<Gpio> gpio, boost::shared_ptr<Spi> spi
)
{
    static uint8_t sys_id_idx = 0;
    boost::shared_ptr<Sensor> sensor;
    switch(type) {
        case SensorType::UWB: {
            // I dont think we want to update all of these at the same time.
            auto node = boost::make_shared<UwbNode>(nh, name, type, sys_id_idx++, "localization_data", 1024, period, can_id);
            m_uwb_nodes.push_back(node);
            sensor = node;
            break;
        }
        case SensorType::QUAD_ENC: {
            auto q = boost::make_shared<QuadEncoder>(nh, name, type, sys_id_idx++, "sensor_data", 128, period, can_id);
            sensor = q;
            break;
        }
        case SensorType::LIMIT_SW: {
            auto sw = boost::make_shared<LimitSwitch>(nh, name, type, sys_id_idx++, topic, 128, period, std::move(gpio), 0, 0);
            sensor = sw;
            break;
        }
        case SensorType::POWER_SENSE: {
            auto ps = boost::make_shared<GenericGpioSensor>(nh, name, type, sys_id_idx++, "sensor_data", 128, period, std::move(gpio));
            sensor = ps;
            break;
        }
        case SensorType::LSM6DS3: {
            auto imu = boost::make_shared<Lsm6ds3>(nh, name, sys_id_idx++, "imu_data", 128, period, spi, std::move(gpio));
            sensor = imu;
            break;
        }
        case SensorType::LOAD_CELL: {
            auto lc = boost::make_shared<LoadCellADC>(nh, name, sys_id_idx++, "sensor_data", 128, period, spi, std::move(gpio));
            sensor = lc;
            break;
        }
        case SensorType::POT: {
            auto pot = boost::make_shared<PotentiometerADC>(nh, name, sys_id_idx++, "sensor_data", 128, period, spi, std::move(gpio));
            sensor = pot;
            break;
        }
        case SensorType::TEMP_SENSE: {
            auto ts = boost::make_shared<EbayTempSensor>(nh, name, sys_id_idx++, "ebay_temperature", 128, period, spi, std::move(gpio));
            sensor = ts;
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
    for(auto sensor : m_sensors) {
        sensor -> setup();
    }
}

void SensorThread::update_sensors() {
    for(auto sensor : m_sensors) {
        if(sensor -> ready_to_update()) sensor -> update();
    }
}


// run thread
void SensorThread::operator()() {
    ROS_INFO("Starting sensor_thread");
    ros::AsyncSpinner spinner(1, &m_cb_queue);
    spinner.start();
    setup_sensors();
    
    while(ros::ok()) {
        update_sensors();
        sleep();   
    }
    shutdown();
}