#pragma once

#include <ros/ros.h>

#include <hwctrl2/CanFrame.h>
#include <hwctrl2/SensorData.h>
#include <hwctrl2/UwbData.h>
#include <hwctrl2/LimitSwState.h>

#include <sensor_msgs/Imu.h>

#include <memory>
#include <string>
#include <vector>
#include <inttypes.h>
#include <system_error>

#include <boost/optional.hpp>
#include <boost/utility/string_view.hpp>
#include <boost/move/unique_ptr.hpp>
#include <boost/move/make_unique.hpp>

#include "interface/gpio.h"
#include "interface/spi.h"

#include "util.h"

struct Calibration {
    std::string name;
    float scale;
    float offset;
    float variance;
};

void read_calibration(const std::string& path, std::vector<Calibration>& cals);

enum SensorType {
	NONE,
	UWB,
	QUAD_ENC,
	LIMIT_SW,
	POT,
	LOAD_CELL,
    TEMP_SENSE,
	LSM6DS3,
	POWER_SENSE,
};

SensorType get_sensor_type_from_param(boost::string_view type_str);

#define SensorBaseArgs ros::NodeHandle nh, const std::string& name, SensorType type, uint32_t id, const std::string& topic, uint32_t topic_size, ros::Duration update_period
#define SensorBaseArgsPass Sensor(nh, name, type, id, topic, topic_size, update_period)

class Sensor {
public:
    Sensor(
        SensorBaseArgs
    );
    virtual ~Sensor() = default;

    // override these
    virtual void setup()   { ROS_WARN("Override me for %s (id: %d)", m_name.c_str(), m_id); };
    virtual void update()  { ROS_WARN("Override me for %s (id: %d)", m_name.c_str(), m_id); };

    bool ready_to_update() const { return m_update; }

    void set_update_flag(const ros::TimerEvent&) { m_update = true; }

    void add_calibration(const Calibration& cal);
    boost::optional<const Calibration&> get_calibration_by_name(boost::string_view name) const;

    inline uint32_t           get_id()    const  { return m_id;       }
    inline SensorType         get_type()  const  { return m_type;     }
    inline bool               is_setup()  const  { return m_is_setup; }
    inline const std::string& get_topic() const  { return m_topic;    }
    inline const std::string& get_name()  const  { return m_name;     }
    
protected:
    uint32_t    m_id;
    SensorType  m_type;
    std::string m_name;
    std::string m_topic;
    bool        m_is_setup;
    bool        m_update;

    std::vector<Calibration> m_calibrations;

    ros::Publisher  m_pub;
    ros::NodeHandle m_nh;
    ros::Duration   m_update_period;
    ros::Timer      m_update_timer;
};

#define SensorImplArgsPass(x) SensorImpl<x>(nh, name, type, id, topic, topic_size, update_period)

// TODO: maybe make this fully templated later?
template<typename T>
class SensorImpl : public Sensor {
public:

    SensorImpl(SensorBaseArgs) : SensorBaseArgsPass {
        m_pub = m_nh.advertise<T>(m_topic, topic_size);
    }

protected:
    using PubData = T;
};

#define CanSensorArgs SensorBaseArgs, uint32_t can_id
#define CanSensorArgsPass(x) CanSensor<x>(nh, name, type, id, topic, topic_size, update_period, can_id)

template<typename T>
class CanSensor : public SensorImpl<T> {
public:

    CanSensor(
        CanSensorArgs
    );
    virtual ~CanSensor() = default;

protected:
    uint32_t        m_can_id;

    ros::Subscriber m_can_rx_sub;
    ros::Publisher  m_can_tx_pub;

    using FramePtr = boost::shared_ptr<hwctrl2::CanFrame>;
    virtual void can_rx_callback(FramePtr frame) = 0;
};

#define SpiSensorArgs SensorBaseArgs, boost::shared_ptr<Spi> spi, uint32_t spi_speed, uint32_t spi_mode, boost::movelib::unique_ptr<Gpio> cs_pin
#define SpiSensorArgsPass(x) SpiSensor<x>(nh, name, type, id, topic, topic_size, update_period, spi, spi_speed, spi_mode, std::move(cs_pin))

template<typename T>
class SpiSensor : public SensorImpl<T> {
public:

    SpiSensor(
        SpiSensorArgs
    );
    virtual ~SpiSensor() {
        // make sure we release gpio handle
        m_cs->release_handle();
    };
protected:
    void config_spi_settings() {
        m_spi->set_speed(m_spi_speed);
        m_spi->set_mode(m_spi_mode);
        ros::Duration(0.001).sleep();
    }
    void transfer_to_device(uint8_t* buf, int buf_len) {
        m_cs ->reset();
        m_spi->transfer(buf, buf_len);
        m_cs ->set();
        ros::Duration(0.001).sleep();
    }

protected:
    boost::shared_ptr<Spi> m_spi;
    uint32_t m_spi_speed;
    uint32_t m_spi_mode;

    // SHOULD THIS BE A REFERENCE OR A PTR???
    boost::movelib::unique_ptr<Gpio>     m_cs;
};

#define GpioSensorArgs SensorBaseArgs, boost::movelib::unique_ptr<Gpio> gpio
#define GpioSensorArgsPass GpioSensor(nh, name, type, id, topic, topic_size, update_period, std::move(gpio))

template<typename T>
class GpioSensor : public SensorImpl<T> {
public:
    GpioSensor(
        GpioSensorArgs
    ) : SensorImplArgsPass(T), m_gpio(std::move(gpio)) {}

    virtual ~GpioSensor() {
        // make sure we release gpio handle
        m_gpio->release_handle();
    };

protected:
    boost::movelib::unique_ptr<Gpio> m_gpio;
};

class GenericGpioSensor : public GpioSensor<hwctrl2::SensorData> {
public:
    GenericGpioSensor(GpioSensorArgs, Gpio::State on_state = Gpio::State::Set);
    virtual ~GenericGpioSensor() = default;

    virtual void setup()  override final;
    virtual void update() override final;

private:
    Gpio::State m_on_state;
};

class LimitSwitch : public GpioSensor<hwctrl2::LimitSwState> {
public:
    LimitSwitch(GpioSensorArgs, uint32_t motor_id, uint32_t allowed_dir);
    virtual ~LimitSwitch() = default;

    virtual void setup()  override final;
    virtual void update() override final;
protected:
    uint32_t m_motor_id;
    uint32_t m_allowed_dir;
};

