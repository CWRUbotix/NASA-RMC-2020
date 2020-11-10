#pragma once

#include <ros/ros.h>

#include <hwctrl2/CanFrame.h>
#include <hwctrl2/LimitSwState.h>

#include <memory>
#include <string>
#include <vector>
#include <inttypes.h>

#include <linux/spi/spidev.h>

#include "interface/gpio.h"
#include "interface/spi.h"

struct Calibration {
    std::string name;
    float scale;
    float offset;
    float variance;
};

enum SensorType {

};

class Sensor {
public:

    template<typename T, uint32_t topic_size = 128>
    Sensor(ros::NodeHandle nh, std::string name, std::string desc, uint32_t id, std::string topic)
    : m_nh(nh), m_id(id), m_name(name), m_desc(desc), m_topic(topic)
    {
        m_pub = nh.advertise<T>(m_topic, topic_size);
    };
    virtual ~Sensor() = default;

    virtual void setup();
    virtual void update();

    inline uint32_t           get_id()    const  { return m_id;       }
    inline SensorType         get_type()  const  { return m_type;     }
    inline bool               is_setup()  const  { return m_is_setup; }
    inline const std::string& get_topic() const  { return m_topic;    }
    inline const std::string& get_name()  const  { return m_name;     }
    inline const std::string& get_desc()  const  { return m_desc;     }

private:
    uint32_t m_id;
    SensorType m_type;
    std::string m_name;
    std::string m_desc;
    bool m_is_setup;

    std::vector<Calibration> m_calibrations;

    std::string     m_topic;

    ros::Publisher  m_pub;
    ros::NodeHandle m_nh;
    ros::Duration   m_update_duration;
    ros::Timer      m_update_timer;
};

class CanSensor : public Sensor {
public:
    CanSensor();
    virtual ~CanSensor() = default;

private:
    ros::Subscriber can_rx_sub;
    ros::Publisher  can_tx_sub;

    void can_rx_callback(boost::shared_ptr<hwctrl2::CanFrame> frame);
};

class SpiSensor : public Sensor {
public:
    template<typename T>
    SpiSensor(
        ros::NodeHandle nh, std::string name, std::string desc, uint32_t id, std::string topic,
        uint32_t spi_handle, uint32_t spi_speed, uint32_t spi_mode, Gpio& cs_pin
    )
    :
    Sensor(nh, name, desc, id, topic),
    m_spi_handle(spi_handle), m_spi_speed(spi_speed), m_spi_mode(spi_mode), m_cs(cs_pin)
    {}

    virtual ~SpiSensor() = default;

    virtual void update() override {
        spi::spi_set_speed(m_spi_handle, m_spi_speed);
        spi::spi_set_mode(m_spi_handle, m_spi_mode);
        ros::Duration(0.0005).sleep();

        // set chip select low
        m_cs.reset();

        spi_update();

        // set chip select high
        m_cs.set();
    }

    virtual void spi_update() = 0;
private:
    uint32_t m_spi_handle;
    uint32_t m_spi_speed;
    uint32_t m_spi_mode;

    // SHOULD THIS BE A REFERENCE OR A PTR
    Gpio&     m_cs;
};

class GpioSensor : public Sensor {
public:
    template<typename T>
    GpioSensor(
        ros::NodeHandle nh, std::string name, std::string desc, uint32_t id, std::string topic, Gpio& gpio
    ) : Sensor(nh, name, desc, id, topic), m_gpio(gpio) {}

    virtual ~GpioSensor() = default;

private:
    Gpio& m_gpio;
};

class GenericGpioSensor : public Sensor {
}

class LimitSwitch : public GpioSensor {
public:
    using DataType = hwctrl2::LimitSwState;
    
    LimitSwitch();
    virtual ~LimitSwitch() = default;

    virtual void update() override {
        auto msg = boost::make_shared<hwctrl2::LimitSwState>();
        auto state = m_gpio.read();
        msg->id = m_id;
        msg
    }
};


