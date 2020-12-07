#pragma once

#include "interface/gpio.h"
#include "interface/spi.h"

#include "sensor.h"

#include <boost/array.hpp>
#include <boost/circular_buffer.hpp>

#include <linux/types.h>
#include <cinttypes>
#include <cmath>

//#define LSM6DS3_X_VARIANCE

#define IMU_SAMPLES 5

#define LSM6DS3_SPI_SPEED 5000000
#define LSM6DS3_SPI_MODE SPI_MODE_3
#define LSM6DS3_WHO_AM_I_ID 0x69
#define LSM6DS3_SET_READ_MODE(b) b |= (1 << 7)
#define LSM6DS3_SET_WRITE_MODE(b) b &= ~(1 << 7)

/* CTRL1_XL */
#define LSM6DS3_ODR_104_HZ (0x04 << 4)
#define LSM6DS3_ODR_208_HZ (0x05 << 4)
#define LSM6DS3_ODR_416_HZ (0x06 << 4)
#define LSM6DS3_FS_XL_2G (0x00 << 2)
#define LSM6DS3_FS_XL_4G (0x02 << 2)
#define LSM6DS3_FS_XL_8G (0x03 << 2)
#define LSM6DS3_FS_XL_16G (0x01 << 2)
#define LSM6DS3_BW_XL_400_HZ 0x00
#define LSM6DS3_BW_XL_200_HZ 0x01
#define LSM6DS3_BW_XL_100_HZ 0x02
#define LSM6DS3_BW_XL_50_HZ 0x03

/* CTRL2_G */
#define LSM6DS3_FS_G_250_DPS (0x00 << 2)
#define LSM6DS3_FS_G_500_DPS (0x01 << 2)
#define LSM6DS3_FS_G_1000_DPS (0x02 << 2)
#define LSM6DS3_FS_G_2000_DPS (0x03 << 2)

/* CTRL3_C */
#define LSM6DS3_BOOT (1 << 7)
#define LSM6DS3_SW_RESET 0x01

/* CTRL9_XL */
#define LSM6DS3_XL_Z_EN

enum ImuReg : uint8_t {
  FUNC_CFG_ACCESS = 0x01,
  SENSOR_SYNC_TIME_FRAME = 0x04,
  FIFO_CTRL_1 = 0x06,
  FIFO_CTRL_2 = 0x07,
  FIFO_CTRL_3 = 0x08,
  FIFO_CTRL_4 = 0x09,
  FIFO_CTRL_5 = 0x0A,
  ORIENT_CFG_G = 0x0B,
  INT1_CTRL = 0x0D,
  INT2_CTRL = 0x0E,
  WHO_AM_I = 0x0F,
  CTRL1_XL = 0x10,
  CTRL2_G = 0x11,
  CTRL3_C = 0x12,
  CTRL4_C = 0x13,
  CTRL5_C = 0x14,
  CTRL6_C = 0x15,
  CTRL7_G = 0x16,
  CTRL8_XL = 0x17,
  CTRL9_XL = 0x18,
  CTRL10_C = 0x19,
  MASTER_CONFIG = 0x1A,
  WAKE_UP_SRC = 0x1B,
  TAP_SRC = 0x1C,
  D6D_SRC = 0x1D,
  STATUS_REG = 0x1E,
  OUT_TEMP_L = 0x20,
  OUT_TEMP_H = 0x21,
  OUTX_L_G = 0x22,
  OUTX_H_G = 0x23,
  OUTY_L_G = 0x24,
  OUTY_H_G = 0x25,
  OUTZ_L_G = 0x26,
  OUTZ_H_G = 0x27,
  OUTX_L_XL = 0x28,
  OUTX_H_XL = 0x29,
  OUTY_L_XL = 0x2A,
  OUTY_H_XL = 0x2B,
  OUTZ_L_XL = 0x2C,
  OUTZ_H_XL = 0x2D,
  SENSORHUB1_REG = 0x2E,
  SENSORHUB2_REG = 0x2F,
  SENSORHUB3_REG = 0x30,
  SENSORHUB4_REG = 0x31,
  SENSORHUB5_REG = 0x32,
  SENSORHUB6_REG = 0x33,
  SENSORHUB7_REG = 0x34,
  SENSORHUB8_REG = 0x35,
  SENSORHUB9_REG = 0x36,
  SENSORHUB10_REG = 0x37,
  SENSORHUB11_REG = 0x38,
  SENSORHUB12_REG = 0x39,
  FIFO_STATUS1 = 0x3A,
  FIFO_STATUS2 = 0x3B,
  FIFO_STATUS3 = 0x3C,
  FIFO_STATUS4 = 0x3D,
  FIFO_DATA_OUT_L = 0x3E,
  FIFO_DATA_OUT_H = 0x3F,
  TIMESTAMP0_REG = 0x40,
  TIMESTAMP1_REG = 0x41,
  TIMESTAMP2_REG = 0x42,
  STEP_TIMESTAMP_L = 0x4B,
  STEP_TIMESTAMP_H = 0x4C,
  SENSORHUB13_REG = 0x4D,
  SENSORHUB14_REG = 0x4E,
  SENSORHUB15_REG = 0x4F,
  SENSORHUB16_REG = 0x50,
  SENSORHUB17_REG = 0x51,
  SENSORHUB18_REG = 0x52,
  FUNC_SRC = 0x53,
  TAP_CFG = 0x58,
  TAP_THS_6D = 0x59,
  INT_DUR2 = 0x5A,
  WAKE_UP_THS = 0x5B,
  WAKE_UP_DUR = 0x5C,
  FREE_FALL = 0x5D,
  MD1_CFG = 0x5E,
  MD2_CFG = 0x5F,
  OUT_MAG_RAW_X_L = 0x66,
  OUT_MAG_RAW_X_H = 0x67,
  OUT_MAG_RAW_Y_L = 0x68,
  OUT_MAG_RAW_Y_H = 0x69,
  OUT_MAG_RAW_Z_L = 0x6A,
  OUT_MAG_RAW_Z_H = 0x6B
};

constexpr double degrees_to_radians(double degrees) {
  return degrees * (M_PI / 180.0);
}

constexpr double g_fs = 250.0;
constexpr double xl_fs = 9.81 * 2.0;

constexpr double g_rms_noise = 0.140;
constexpr double xl_rms_noise = 0.0017;

constexpr double gyro_var =
    (double)(degrees_to_radians(g_rms_noise) * degrees_to_radians(g_rms_noise));
constexpr double xl_var =
    (double)(xl_rms_noise * xl_fs) * (xl_rms_noise * xl_fs);
// nh, name, type, id, topic, topic_size, update_period, spi_handle, spi_speed,
// spi_mode, cs_pin
class Lsm6ds3 : public SpiSensor<sensor_msgs::Imu> {
 public:
  enum class Axis : uint8_t { X = 0x01, Y = 0x02, Z = 0x03 };

  using SampleBuffer = boost::circular_buffer<float>;
  using SampleBufferIter = SampleBuffer::iterator;
  using SampleBufferConstIter = SampleBuffer::const_iterator;

  // these have to be boost arrays for ROS messages :(
  using DataArray = boost::array<double, 3>;
  using OffsetArray = boost::array<float, 3>;

  // change this to an actual matrix???
  using VarianceMatrix = boost::array<double, 9>;

 public:
  Lsm6ds3(ros::NodeHandle nh, const std::string& name, uint32_t id,
          const std::string& topic, uint32_t topic_size,
          ros::Duration update_period, boost::shared_ptr<Spi> spi,
          boost::movelib::unique_ptr<Gpio> cs, uint32_t samples = 5);
  virtual ~Lsm6ds3();

  virtual void setup() override final;
  virtual void update() override final;
  virtual void calibrate(std::vector<Calibration>& cals) override final;

 private:
  void power_on_accel(uint8_t config_byte);
  void power_on_gyro(uint8_t config_byte);
  void power_off();

  float read_accel(Axis axis);
  float read_gyro(Axis axis);

  void read_all_data(DataArray& xl_data, DataArray& gyro_data);

  void soft_reset();

 private:
  OffsetArray m_xl_offset;
  OffsetArray m_gyro_offset;

  SampleBuffer m_sample_buf1;
  SampleBuffer m_sample_buf2;
  SampleBuffer m_sample_buf3;
  SampleBuffer m_sample_buf4;
  SampleBuffer m_sample_buf5;
  SampleBuffer m_sample_buf6;

  float m_rm1;  //...
  float m_rm2;
  float m_rm3;
  float m_rm4;
  float m_rm5;
  float m_rm6;

  VarianceMatrix m_cov_xl = {xl_var, 0.0, 0.0, 0.0,   xl_var,
                             0.0,    0.0, 0.0, xl_var};
  VarianceMatrix m_cov_g = {gyro_var, 0.0, 0.0, 0.0,     gyro_var,
                            0.0,      0.0, 0.0, gyro_var};
};
