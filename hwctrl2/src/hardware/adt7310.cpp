#include "hardware/adt7310.h"

#include <linux/spi/spidev.h>

EbayTempSensor::EbayTempSensor(
    ros::NodeHandle nh, const std::string& name, uint32_t id, const std::string& topic, uint32_t topic_size, ros::Duration update_period,
    boost::shared_ptr<Spi> spi, boost::movelib::unique_ptr<Gpio> cs_pin
) : SpiSensor<PubData>(nh, name, SensorType::TEMP_SENSE, id, topic, topic_size, update_period, spi, ADT7310_SPI_SPEED, ADT7310_SPI_MODE, std::move(cs_pin))
{
}

void EbayTempSensor::setup() {
    uint8_t buf[3] = {};
    memset(buf, 0, 3);
    ROS_INFO("Setting up the ADT7310");
    ros::Duration(0.001).sleep();
    config_spi_settings();
    buf[0] = ADT7310_CMD_READ_REG | (ADT7310_REG_ID << 3); // read ID register
    buf[1] = 0x00;

    transfer_to_device(buf, 2);

    if((buf[1] & ADT7310_MFG_ID_MASK) == ADT7310_MFG_ID){
        ROS_INFO("Great, ADT7310 Mfg ID read correctly!");
        buf[0] = (ADT7310_REG_CONFIG << 3); // writing to config register
        buf[1] = ADT7310_FAULTS_1 | ADT7310_RES_16_BIT; // why not 16 bit?
        transfer_to_device(buf, 2);

        int16_t temp;
        buf[0] = (ADT7310_REG_T_CRIT << 3); // writing to critical temperature register
        temp   = T_CRIT_16_BIT;
        buf[1] = (temp >> 8);
        buf[2] = temp & 0xFF;
        transfer_to_device(buf, 3);

        buf[0] = (ADT7310_REG_T_HIGH << 3); // writing to critical temperature register
        temp   = T_HIGH_16_BIT;
        buf[1] = (temp >> 8);
        buf[2] = temp & 0xFF;
        transfer_to_device(buf, 3);

        m_is_setup = true;
    }else{
        ROS_INFO("Read %x from ADT7310", buf[1]);
    }
}

void EbayTempSensor::update() {
    uint8_t buf[3] = {};
    memset(buf, 0, 3);

    buf[0] = ADT7310_CMD_READ_REG | (ADT7310_REG_TEMP_VAL << 3);
    transfer_to_device(buf, 3);
    int16_t raw = (buf[1] << 8) | buf[2];

    hwctrl2::SensorData msg;
    msg.sensor_id = m_id;
    msg.value = (float)raw * ADT7310_LSB_16_BIT; // convert to degrees C
    msg.name = m_name;
    m_pub.publish(msg);

    m_update = false;
}