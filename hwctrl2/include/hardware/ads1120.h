#pragma once 

#include "interface/gpio.h"
#include "interface/spi.h"

#include "sensor.h"

#define ADS1120_SPI_MODE  SPI_MODE_1
#define ADS1120_SPI_SPEED 4000000

/* ADS1120 Commands */
#define ADS1120_CMD_RESET       0x06
#define ADS1120_CMD_START_SYNC  0x08
#define ADS1120_CMD_POWERDOWN   0x02
#define ADS1120_CMD_RDATA       0x10
#define ADS1120_CMD_RREG        (0x02 << 4)
#define ADS1120_CMD_WREG        (0x04 << 4)

/* CONFIG REGISTER 0x00 */
#define ADS1120_MUX_P0_N1         (0x00 << 4)
#define ADS1120_MUX_P0_N2         (0x01 << 4)
#define ADS1120_MUX_P0_N3         (0x02 << 4)
#define ADS1120_MUX_P1_N2         (0x03 << 4)
#define ADS1120_MUX_P1_N3         (0x04 << 4)
#define ADS1120_MUX_P2_N3         (0x05 << 4)
#define ADS1120_MUX_P1_N0         (0x06 << 4)
#define ADS1120_MUX_P3_N2         (0x07 << 4)
#define ADS1120_MUX_P0_NVSS       (0x08 << 4)
#define ADS1120_MUX_P1_NVSS       (0x09 << 4)
#define ADS1120_MUX_P2_NVSS       (0x0A << 4)
#define ADS1120_MUX_P3_NVSS       (0x0B << 4)
#define ADS1120_MUX_VREF_DIV_4    (0x0C << 4)
#define ADS1120_MUX_VDD_DIV_4     (0x0D << 4)
#define ADS1120_MUX_VDD_VSS_DIV_2 (0x0E << 4)

#define ADS1120_PGA_GAIN_1      0x00
#define ADS1120_PGA_GAIN_2      ((ADS1120_PGA_GAIN_1 + 1) << 1)
#define ADS1120_PGA_GAIN_4      ((ADS1120_PGA_GAIN_2 + 1) << 1)
#define ADS1120_PGA_GAIN_8      ((ADS1120_PGA_GAIN_4 + 1) << 1)
#define ADS1120_PGA_GAIN_16     ((ADS1120_PGA_GAIN_8 + 1) << 1)
#define ADS1120_PGA_GAIN_32     ((ADS1120_PGA_GAIN_16 + 1) << 1)
#define ADS1120_PGA_GAIN_64     ((ADS1120_PGA_GAIN_32 + 1) << 1)
#define ADS1120_PGA_GAIN_128    ((ADS1120_PGA_GAIN_64 + 1) << 1)

#define ADS1120_PGA_EN          0x00
#define ADS1120_PGA_BYPASS      0x01

/* CONFIG REGISTER 0x01 */
#define ADS1120_DR_0            (0x00)
#define ADS1120_DR_1            (0x01 << 5)
#define ADS1120_DR_2            (0x02 << 5)
#define ADS1120_DR_3            (0x03 << 5)
#define ADS1120_DR_4            (0x04 << 5)
#define ADS1120_DR_5            (0x05 << 5)
#define ADS1120_DR_6            (0x06 << 5)

#define ADS1120_MODE_NORMAL     (0x00 << 3)
#define ADS1120_MODE_DUTY_CYCLE (0x01 << 3)
#define ADS1120_MODE_TURBO      (0x02 << 3)

#define ADS1120_CM_CONT         (0x01 << 2)

#define ADS1120_TS_ENABLE       (0x01 << 1)

#define ADS1120_BCS_ON          (0x01)

/* CONFIG REGISTER 0x02 */
#define ADS1120_VREF_2_048      (0x00 << 6)
#define ADS1120_VREF_EXT        (0x01 << 6)
#define ADS1120_VREF_AIN0_AIN3  (0x02 << 6)
#define ADS1120_VREF_AVDD_AVSS  (0x03 << 6)

#define ADS1120_FIR_50_60       (0x01 << 4)
#define ADS1120_FIR_50          (0x02 << 4)
#define ADS1120_FIR_60          (0x03 << 4)

#define ADS1120_LOW_SIDE_PSW_EN (1 << 3)

#define ADS1120_IDAC_50UA       (0x02)
#define ADS1120_IDAC_100UA      (0x03)
#define ADS1120_IDAC_250UA      (0x04)
#define ADS1120_IDAC_500UA      (0x05)
#define ADS1120_IDAC_1000UA     (0x06)
#define ADS1120_IDAC_1500UA     (0x07)

/* CONFIG REGISTER 0x03 */
#define ADS1120_IDAC_MUX_AIN0       (0x01 << 5)
#define ADS1120_IDAC_MUX_AIN1       (0x02 << 5)
#define ADS1120_IDAC_MUX_AIN2       (0x03 << 5)
#define ADS1120_IDAC_MUX_AIN3       (0x04 << 5)
#define ADS1120_IDAC_MUX_REFP0      (0x05 << 5)
#define ADS1120_IDAC_MUX_REFN0      (0x06 << 5)

#define ADS1120_DRDYM_BOTH          (1 << 1)


class PotentiometerADC : public SpiSensor<hwctrl2::SensorData> {
public:
    PotentiometerADC(
        ros::NodeHandle nh, std::string name, uint32_t id, std::string topic,
        uint32_t size, ros::Duration update_period, boost::shared_ptr<Spi> spi, boost::shared_ptr<Gpio> cs_pin
    );
    virtual ~PotentiometerADC() = default;

    virtual void setup()  override final;
    virtual void update() override final;
};

class LoadCellADC : public SpiSensor<hwctrl2::SensorData> {
public:
    LoadCellADC(
        ros::NodeHandle nh, std::string name, uint32_t id, std::string topic,
        uint32_t size, ros::Duration update_period, boost::shared_ptr<Spi> spi, boost::shared_ptr<Gpio> cs_pin
    );
    virtual ~LoadCellADC() = default;

    virtual void setup()  override final;
    virtual void update() override final;
};
