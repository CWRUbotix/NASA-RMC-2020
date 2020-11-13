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

#include "interface/gpio.h"
#include "interface/spi.h"

#include "util.h"

struct Calibration {
    std::string name;
    float scale;
    float offset;
    float variance;
};

enum SensorType {
	NONE,
	UWB,
	QUAD_ENC,
	LIMIT_SW,
	POT,
	LOAD_CELL,
	ADS1120,
	ADT7310,
	LSM6DS3,
	POWER_SENSE,
};

SensorType get_sensor_type_from_param(boost::string_view type_str);

#define SensorBaseArgs ros::NodeHandle nh, const std::string& name, const std::string& desc, SensorType type, uint32_t id, const std::string& topic, uint32_t topic_size, ros::Duration update_period
#define SensorBaseArgsPass Sensor(nh, name, desc, type, id, topic, topic_size, update_period)

class Sensor {
public:
    Sensor(
        SensorBaseArgs
    );
    virtual ~Sensor() = default;

    // override these
    virtual void setup()                        { ROS_WARN("Override me for %s (id: %d)", m_name.c_str(), m_id); };
    virtual void update()                       { ROS_WARN("Override me for %s (id: %d)", m_name.c_str(), m_id); };

    bool ready_to_update() const { return m_update; }

    void set_update_flag(const ros::TimerEvent&) { m_update = true; }

    void add_calibration(const Calibration& cal);
    boost::optional<const Calibration&> get_calibration_by_name(const std::string& name) const;

    inline uint32_t           get_id()    const  { return m_id;       }
    inline SensorType         get_type()  const  { return m_type;     }
    inline bool               is_setup()  const  { return m_is_setup; }
    inline const std::string& get_topic() const  { return m_topic;    }
    inline const std::string& get_name()  const  { return m_name;     }
    inline const std::string& get_desc()  const  { return m_desc;     }

protected:
    uint32_t    m_id;
    SensorType  m_type;
    std::string m_name;
    std::string m_desc;
    std::string m_topic;
    bool        m_is_setup;
    bool        m_update;

    std::vector<Calibration> m_calibrations;

    ros::Publisher  m_pub;
    ros::NodeHandle m_nh;
    ros::Duration   m_update_period;
    ros::Timer      m_update_timer;
};

#define SensorImplArgsPass(x) SensorImpl<x>(nh, name, desc, type, id, topic, topic_size, update_period)

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

#define CanSensorArgs SensorBaseArgs
#define CanSensorArgsPass(x) CanSensor<x>(nh, name, desc, type, id, topic, topic_size, update_period)

template<typename T>
class CanSensor : public SensorImpl<T> {
public:

    CanSensor(
        CanSensorArgs
    );
    virtual ~CanSensor() = default;

protected:
    ros::Subscriber m_can_rx_sub;
    ros::Publisher  m_can_tx_pub;

    using FramePtr = boost::shared_ptr<hwctrl2::CanFrame>;
    virtual void can_rx_callback(FramePtr frame) = 0;
};

#define SpiSensorArgs SensorBaseArgs, uint32_t spi_handle, uint32_t spi_speed, uint32_t spi_mode, Gpio& cs_pin
#define SpiSensorArgsPass SpiSensor(nh, name, desc, type, id, topic, topic_size, update_period, spi_handle, spi_speed, spi_mode, cs_pin)

template<typename T>
class SpiSensor : public SensorImpl<T> {
public:

    SpiSensor(
        SpiSensorArgs
    );
    virtual ~SpiSensor() {
        // make sure we release gpio handle
        m_cs.release_handle();
    };

protected:
    uint32_t m_spi_handle;
    uint32_t m_spi_speed;
    uint32_t m_spi_mode;

    // SHOULD THIS BE A REFERENCE OR A PTR???
    Gpio&     m_cs;
};

#define GpioSensorArgs SensorBaseArgs, Gpio& gpio
#define GpioSensorArgsPass GpioSensor(nh, name, desc, type, id, topic, topic_size, update_period, gpio)

template<typename T>
class GpioSensor : public SensorImpl<T> {
public:
    GpioSensor(
        GpioSensorArgs
    ) : SensorImplArgsPass(T), m_gpio(gpio) {}

    virtual ~GpioSensor() {
        // make sure we release gpio handle
        m_gpio.release_handle();
    };

protected:
    Gpio& m_gpio;
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

