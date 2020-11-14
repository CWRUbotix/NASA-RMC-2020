#include "hardware/ads1120.h"

PotentiometerADC::PotentiometerADC() {}

void PotentiometerADC::setup() {
    uint8_t buf[2] = {};
    m_spi->set_speed(m_spi_speed);
    m_spi->set_mode(m_spi_mode);

    buf[0] = ADS1120_CMD_RESET;
    m_cs.reset();
    m_spi->transfer(buf, 1);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
    buf[1] = ADS1120_PGA_BYPASS; 			// we want to bypass the PGA
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
	buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_4 | ADS1120_CM_CONT; // 330 SPS, continuous conversion
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
	buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1;
    buf[1] = ADS1120_MUX_P1_NVSS | ADS1120_PGA_BYPASS;
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    m_is_setup = true;
}

void PotentiometerADC::update() {
    m_spi->set_speed(m_spi_speed);
    m_spi->set_mode(m_spi_mode);
    // TODO: implement this. Does nothing at the moment

    m_update = false;
}

LoadCellADC::LoadCellADC() {}

void LoadCellADC::setup() {
    uint8_t buf[2] = {};

    m_spi->set_speed(m_spi_speed);
    m_spi->set_mode(m_spi_mode);
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_RESET;
    m_cs.reset();
    m_spi->transfer(buf, 1);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
	buf[1] = ADS1120_MUX_P1_N2 | ADS1120_PGA_GAIN_128; // set mux, plus lotsa gain
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
    buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_1 | ADS1120_CM_CONT; // 45 SPS, continuous conversion
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
	buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
    m_cs.reset();
    m_spi->transfer(buf, 2);
    m_cs.set();
    ros::Duration(0.001).sleep();

    m_is_setup = true;
}

void LoadCellADC::update() {
    // does nothing for now
    m_spi->set_speed(m_spi_speed);
    m_spi->set_mode(m_spi_mode);

    m_update = false;
}