#include "hardware/sensor.h"

Sensor::Sensor(SensorBaseArgs)
: m_nh(nh), m_id(id), m_name(name), m_desc(desc), m_topic(topic), m_update_period(update_period), m_type(type)
{
    // TODO: Check if this is ok to call in this construtor
    m_update_timer = m_nh.createTimer(m_update_period, &Sensor::update, this);
};


void Sensor::add_calibration(const Calibration& cal)  {
    m_calibrations.push_back(cal);
}

boost::optional<const Calibration&> Sensor::get_calibration_by_name(const std::string& name) const {
    for( auto it = m_calibrations.begin(); it != m_calibrations.end(); ++it) {
        if(it->name.compare(name) == 0) {
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
    m_can_tx_pub = CanSensor<T>::m_nh.template advertise<hwctrl2::CanFrame>("can_frames_tx", 128);
}

template<typename T>
SpiSensor<T>::SpiSensor(SpiSensorArgs)
: SensorImplArgsPass(T), m_spi_handle(spi_handle), m_spi_speed(spi_speed), m_spi_mode(spi_mode), m_cs(cs_pin)
{}

template<typename T>
void SpiSensor<T>::update(const ros::TimerEvent&) {
    spi::spi_set_speed(m_spi_handle, m_spi_speed);
    spi::spi_set_mode(m_spi_handle, m_spi_mode);
    ros::Duration(0.0005).sleep();

    // set chip select low
    m_cs.reset();

    spi_update();

    // set chip select high
    m_cs.set();
}

GenericGpioSensor::GenericGpioSensor(GpioSensorArgs, Gpio::State on_state)
: GpioSensorArgsPass, m_on_state(on_state)
{
    m_pub = m_nh.advertise<hwctrl2::SensorData>(m_topic, topic_size);
}

void GenericGpioSensor::setup() {
    m_gpio.set_direction(Gpio::Direction::Input);
    m_is_setup = true;
}

void GenericGpioSensor::update(const ros::TimerEvent&) {
    if(!m_is_setup){
        ROS_WARN("Cannot update sensor %s (id: %d): not setup yet", m_name.c_str(), m_id);
        return;
    }
    auto msg = boost::make_shared<hwctrl2::SensorData>();
    auto val = m_gpio.read_state();
    msg -> name = m_name;
    msg -> sensor_id = m_id;
    msg -> value = (float) (val == m_on_state);
    m_pub.publish(msg);
}

LimitSwitch::LimitSwitch(GpioSensorArgs, uint32_t motor_id, uint32_t allowed_dir)
: GpioSensorArgsPass, m_motor_id(motor_id), m_allowed_dir(allowed_dir)
{
    m_pub = m_nh.advertise<hwctrl2::LimitSwState>(m_topic, topic_size);
}


void LimitSwitch::setup() {
    m_gpio.set_direction(Gpio::Direction::Input);
    m_is_setup = true;
}

void LimitSwitch::update(const ros::TimerEvent&) {
    if(!m_is_setup){
        ROS_WARN("Cannot update sensor %s (id: %d): not setup yet", m_name.c_str(), m_id);
        return;
    }
    auto msg = boost::make_shared<hwctrl2::LimitSwState>();
    auto state = m_gpio.read_state();
    msg->id = m_id;
    msg->state = (state == Gpio::State::Set);
    msg->motor_id = m_motor_id;
    msg->allowed_dir = m_allowed_dir;
    msg->timestamp = ros::Time::now();
    m_pub.publish(msg);
}


// Define these types to avoid linking errors
// this also prevents random things from being published (totally a feature and NOT a bug)
// eventually I should restructure the project so we dont have to do this
template class CanSensor<hwctrl2::UwbData>;
template class CanSensor<hwctrl2::SensorData>;
template class CanSensor<hwctrl2::LimitSwState>;
template class CanSensor<sensor_msgs::Imu>;

template class SpiSensor<hwctrl2::SensorData>;
template class SpiSensor<hwctrl2::LimitSwState>;
template class SpiSensor<sensor_msgs::Imu>;

template class GpioSensor<hwctrl2::SensorData>;
template class GpioSensor<hwctrl2::LimitSwState>;