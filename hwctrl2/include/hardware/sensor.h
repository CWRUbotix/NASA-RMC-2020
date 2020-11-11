#pragma once

#include <ros/ros.h>

#include <hwctrl2/CanFrame.h>

#include <memory>
#include <string>
#include <vector>
#include <inttypes.h>
#include <system_error>

#include <boost/optional.hpp>

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

};

#define SensorBaseArgs ros::NodeHandle nh, const std::string& name, const std::string& desc, SensorType type, uint32_t id, const std::string& topic, uint32_t topic_size, ros::Duration update_period
#define SensorBaseArgsPass Sensor(nh, name, desc, type, id, topic, topic_size, update_period)

class Sensor {
public:
    Sensor(
        SensorBaseArgs
    );
    virtual ~Sensor() = default;

    virtual void setup() = 0;
    virtual void update(const ros::TimerEvent&) = 0;

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

    std::vector<Calibration> m_calibrations;

    ros::Publisher  m_pub;
    ros::NodeHandle m_nh;
    ros::Duration   m_update_period;
    ros::Timer      m_update_timer;
};


// TODO: maybe make this fully templated later?
// template<typename _T>
// class SensorImpl : public Sensor {
//     using T = _T;

// };

#define CanSensorArgs SensorBaseArgs
#define CanSensorArgsPass CanSensor(nh, name, desc, type, id, topic, topic_size, update_period)

class CanSensor : public Sensor {
public:

    CanSensor(
        CanSensorArgs
    );
    virtual ~CanSensor() = default;

protected:
    ros::Subscriber m_can_rx_sub;
    ros::Publisher  m_can_tx_pub;

    virtual void can_rx_callback(boost::shared_ptr<hwctrl2::CanFrame> frame) = 0;
};

#define SpiSensorArgs SensorBaseArgs, uint32_t spi_handle, uint32_t spi_speed, uint32_t spi_mode, Gpio& cs_pin
#define SpiSensorArgsPass SpiSensor(nh, name, desc, type, id, topic, topic_size, update_period, spi_handle, spi_speed, spi_mode, cs_pin)

class SpiSensor : public Sensor {
public:
    SpiSensor(
        SpiSensorArgs
    );
    virtual ~SpiSensor() = default;

    virtual void update(const ros::TimerEvent&) override final;

    virtual void spi_update() = 0;

protected:
    uint32_t m_spi_handle;
    uint32_t m_spi_speed;
    uint32_t m_spi_mode;

    // SHOULD THIS BE A REFERENCE OR A PTR???
    Gpio&     m_cs;
};

#define GpioSensorArgs SensorBaseArgs, Gpio& gpio
#define GpioSensorArgsPass GpioSensor(nh, name, desc, type, id, topic, topic_size, update_period, gpio)

class GpioSensor : public Sensor {
public:
    GpioSensor(
        GpioSensorArgs
    ) : SensorBaseArgsPass, m_gpio(gpio) {}

    virtual ~GpioSensor() = default;

protected:
    Gpio& m_gpio;
};

class GenericGpioSensor : public GpioSensor {
public:
    GenericGpioSensor(GpioSensorArgs, Gpio::State on_state = Gpio::State::Set);
    virtual ~GenericGpioSensor() = default;

    virtual void setup() override final;
    virtual void update(const ros::TimerEvent&) override final;

private:
    Gpio::State m_on_state;
};

class LimitSwitch : public GpioSensor {
public:    
    LimitSwitch(GpioSensorArgs, uint32_t motor_id, uint32_t allowed_dir);
    virtual ~LimitSwitch() = default;

    virtual void setup() override final;
    virtual void update(const ros::TimerEvent&) override final;
protected:
    uint32_t m_motor_id;
    uint32_t m_allowed_dir;
};

