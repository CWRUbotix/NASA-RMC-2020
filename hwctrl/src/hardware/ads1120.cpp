#include "pch.h"
#include "hardware/ads1120.h"

#include <linux/spi/spidev.h>

PotentiometerADC::PotentiometerADC(
    ros::NodeHandle nh, const std::string& name, uint32_t id, const std::string& topic,
    uint32_t topic_size, ros::Duration update_period, boost::shared_ptr<Spi> spi, boost::movelib::unique_ptr<Gpio> cs_pin
): SpiSensor<PubData>(nh, name, SensorType::POT, id, topic, topic_size, update_period, spi, ADS1120_SPI_SPEED, ADS1120_SPI_MODE, std::move(cs_pin))
{}

void PotentiometerADC::setup() {
    uint8_t buf[2] = {};
    config_spi_settings();

    buf[0] = ADS1120_CMD_RESET;
    transfer_to_device(buf, 1);

    buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
    buf[1] = ADS1120_PGA_BYPASS; 			// we want to bypass the PGA
    transfer_to_device(buf, 2);

    buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
	buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_4 | ADS1120_CM_CONT; // 330 SPS, continuous conversion
    transfer_to_device(buf, 2);

    buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
	buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
    transfer_to_device(buf, 2);

    buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1;
    buf[1] = ADS1120_MUX_P1_NVSS | ADS1120_PGA_BYPASS;
    transfer_to_device(buf, 2);

    m_is_setup = true;
}

void PotentiometerADC::update() {
    config_spi_settings();
    // TODO: implement this. Does nothing at the moment

    m_update = false;
}

LoadCellADC::LoadCellADC(
    ros::NodeHandle nh, const std::string& name, uint32_t id, const std::string& topic,
    uint32_t topic_size, ros::Duration update_period, boost::shared_ptr<Spi> spi, boost::movelib::unique_ptr<Gpio> cs_pin
) : SpiSensor<PubData>(nh, name, SensorType::LOAD_CELL, id, topic, topic_size, update_period, spi, ADS1120_SPI_SPEED, ADS1120_SPI_MODE, std::move(cs_pin))
{}

void LoadCellADC::setup() {
    uint8_t buf[2] = {};
    config_spi_settings();

    buf[0] = ADS1120_CMD_RESET;
    transfer_to_device(buf, 1);

    buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
	buf[1] = ADS1120_MUX_P1_N2 | ADS1120_PGA_GAIN_128; // set mux, plus lotsa gain
    transfer_to_device(buf, 2);

    buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
    buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_1 | ADS1120_CM_CONT; // 45 SPS, continuous conversion
    transfer_to_device(buf, 2);

    buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
	buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
    transfer_to_device(buf, 2);

    m_is_setup = true;
}

void LoadCellADC::update() {
    // does nothing for now
    config_spi_settings();

    m_update = false;
}