#include "pch.h"
#include "hardware/sensor.h"

#include "util.h"

void read_calibration(const std::string& path, std::vector<Calibration>& cals) {
    std::vector<std::vector<std::string>> data = csv::read_csv(path);
    for(auto line : data) {
        Calibration cal;
        cal.name = line.at(0);
        cal.scale =    std::stof(line.at(1));
        cal.offset =   std::stof(line.at(2));
        cal.variance = std::stof(line.at(3));
        cals.push_back(cal);
    }
}

bool write_calibration(const std::string& path, std::vector<Calibration>& cals) {
	std::vector<std::vector<std::string>> data;
	char word[256];
	for(auto cal : cals){
		std::vector<std::string> line;
		printf("Writing calibration for %s\r\n", cal.name.c_str());
		line.push_back(cal.name);
		sprintf(word, "%.6f",cal.scale);
		line.push_back(std::string(word));
		sprintf(word, "%.6f",cal.offset);
		line.push_back(std::string(word));
		sprintf(word, "%.6g",cal.variance);
		line.push_back(std::string(word));
		data.push_back(line); // add the line finally
	}
	csv::write_csv(path, data);
}

std::string print_calibration(Calibration& cal){
	char buf[256];
	sprintf(buf, "=== Name: %s ===\r\nScale:\t%.4g\r\nOffset:\t%.4g\r\nVariance:\t%.4g\r\n", 
		cal.name.c_str(),
		cal.scale,
		cal.offset,
		cal.variance
		);
	return std::string(buf);
}

SensorType get_sensor_type_from_param(boost::string_view type_str) {
	if(      type_str.compare("uwb")         == 0){
		return SensorType::UWB;
	}else if(type_str.compare("quad_enc")    == 0){
		return SensorType::QUAD_ENC;
	}else if(type_str.compare("limit")       == 0){
		return SensorType::LIMIT_SW;
	}else if(type_str.compare("imu")         == 0){
		return SensorType::LSM6DS3;
	}else if(type_str.compare("temperature") == 0){
		return SensorType::TEMP_SENSE;
	}else if(type_str.compare("pot")         == 0){
		return SensorType::POT;
	}else if(type_str.compare("load_cell") 	 == 0){
		return SensorType::LOAD_CELL;
	}else if(type_str.compare("power_sense") == 0){
		return SensorType::POWER_SENSE;
	}else{
		return SensorType::NONE;
	}
}

Sensor::Sensor(SensorBaseArgs)
: m_nh(nh), m_id(id), m_name(name), m_topic(topic), m_update_period(update_period), m_type(type), m_update(true)
{
    m_update_timer = m_nh.createTimer(m_update_period, &Sensor::set_update_flag, this);
};


void Sensor::add_calibration(const Calibration& cal)  {
    m_calibrations.push_back(cal);
}

boost::optional<const Calibration&> Sensor::get_calibration_by_name(boost::string_view s) const {
    for( auto it = m_calibrations.begin(); it != m_calibrations.end(); ++it) {
        if(s.compare(it->name) == 0) {
            return *it;
        } 
    }
    return boost::optional<const Calibration&>();
}

template<typename T>
CanSensor<T>::CanSensor(CanSensorArgs)
: SensorImplArgsPass(T)
{
    m_can_rx_sub = CanSensor<T>::m_nh.subscribe("can_frames_rx", 128, &CanSensor::can_rx_callback, this);
    m_can_tx_pub = CanSensor<T>::m_nh.template advertise<hwctrl::CanFrame>("can_frames_tx", 128);
}

template<typename T>
SpiSensor<T>::SpiSensor(SpiSensorArgs)
: SensorImplArgsPass(T), m_spi(spi), m_spi_speed(spi_speed), m_spi_mode(spi_mode), m_cs(std::move(cs_pin))
{}

GenericGpioSensor::GenericGpioSensor(GpioSensorArgs, Gpio::State on_state)
: GpioSensorArgsPass, m_on_state(on_state)
{}

void GenericGpioSensor::setup() {
    m_gpio->set_direction(Gpio::Direction::Input);
    m_is_setup = true;
}

void GenericGpioSensor::update() {
    if(!m_is_setup){
        ROS_WARN("Cannot update sensor %s (id: %d): not setup yet", m_name.c_str(), m_id);
        return;
    }
    auto msg = boost::make_shared<hwctrl::SensorData>();
    auto val = m_gpio->read_state();
    msg -> name = m_name;
    msg -> sensor_id = m_id;
    msg -> value = (float) (val == m_on_state);
    m_pub.publish(msg);

    m_update = false;
}

LimitSwitch::LimitSwitch(GpioSensorArgs, uint32_t motor_id, uint32_t allowed_dir)
: GpioSensorArgsPass, m_motor_id(motor_id), m_allowed_dir(allowed_dir)
{}


void LimitSwitch::setup() {
    m_gpio->set_direction(Gpio::Direction::Input);
    m_is_setup = true;
}

void LimitSwitch::update() {
    if(!m_is_setup){
        ROS_WARN("Cannot update sensor %s (id: %d): not setup yet", m_name.c_str(), m_id);
        return;
    }
    auto msg = boost::make_shared<hwctrl::LimitSwState>();
    auto state = m_gpio->read_state(); 
    msg->id = m_id;
    msg->state = (state == Gpio::State::Set);
    msg->motor_id = m_motor_id;
    msg->allowed_dir = m_allowed_dir;
    msg->timestamp = ros::Time::now();
    m_pub.publish(msg);

    m_update = false;
}


// Define these types to avoid linking errors
// this also prevents random things from being published (totally a feature and NOT a bug)
// eventually I should restructure the project so we dont have to do this
template class CanSensor<hwctrl::UwbData>;
template class CanSensor<hwctrl::SensorData>;
template class CanSensor<hwctrl::LimitSwState>;
template class CanSensor<sensor_msgs::Imu>;

template class SpiSensor<hwctrl::SensorData>;
template class SpiSensor<hwctrl::LimitSwState>;
template class SpiSensor<sensor_msgs::Imu>;

template class GpioSensor<hwctrl::SensorData>;
template class GpioSensor<hwctrl::LimitSwState>;