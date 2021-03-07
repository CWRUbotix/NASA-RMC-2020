#pragma once

#include "interface/gpio.h"
#include "interface/spi.h"

#include "sensor.h"

#define T_HIGH_CELCIUS 40.0
#define T_CRIT_CELCIUS 55.0

#define ADT7310_LSB_13_BIT ((float)0.0625)
#define ADT7310_LSB_16_BIT ((float)0.00781)

#define T_HIGH_16_BIT ((int16_t)(T_HIGH_CELCIUS / ADT7310_LSB_16_BIT))
#define T_CRIT_16_BIT ((int16_t)(T_CRIT_CELCIUS / ADT7310_LSB_16_BIT))

#define ADT7310_SPI_MODE SPI_MODE_3
#define ADT7310_SPI_SPEED 1000000

#define ADT7310_MFG_ID_MASK (0x1F << 3)
#define ADT7310_MFG_ID (0x18 << 3)

/* COMMAND BYTE */
#define ADT7310_CMD_READ_REG (1 << 6)
#define ADT7310_CMD_CNT_READ (1 << 2)

/* REGISTERS */
#define ADT7310_REG_STATUS 0x00
#define ADT7310_REG_CONFIG 0x01
#define ADT7310_REG_TEMP_VAL 0x02
#define ADT7310_REG_ID 0x03
#define ADT7310_REG_T_CRIT 0x04
#define ADT7310_REG_T_HYST 0x05
#define ADT7310_REG_T_HIGH 0x06
#define ADT7310_REG_T_LOW 0x07

/* STATUS REG 0x00 */
#define ADT7310_T_LOW_BIT (1 << 4)
#define ADT7310_T_HIGH_BIT (1 << 5)
#define ADT7310_T_CRIT_BIT (1 << 6)
#define ADT7310_RDY (1 << 7)

/* CONFIGURATION REG 0x01 */
#define ADT7310_FAULTS_1 (0x00)
#define ADT7310_FAULTS_2 (0x01)
#define ADT7310_FAULTS_3 (0x02)
#define ADT7310_FAULTS_4 (0x03)

#define ADT7310_CT_ACTIVE_HIGH (1 << 2)
#define ADT7310_INT_ACTIVE_HIGH (1 << 3)
#define ADT7310_COMP_MODE (1 << 4)
#define ADT7310_CONT_CONV (0x00 << 5)
#define ADT7310_ONE_SHOT (0x01 << 5)
#define ADT7310_1_SPS (0x02 << 5)
#define ADT7310_SHUTDOWN (0x03 << 5)
#define ADT7310_RES_16_BIT (1 << 7)

class EbayTempSensor : public SpiSensor<hwctrl::SensorData> {
 public:
  EbayTempSensor(ros::NodeHandle nh, boost::shared_ptr<Spi> spi, std::unique_ptr<Gpio> cs_pin, SensorConfig const& config);
  virtual ~EbayTempSensor() = default;

  virtual void setup() override final;
  virtual void update() override final;
};
