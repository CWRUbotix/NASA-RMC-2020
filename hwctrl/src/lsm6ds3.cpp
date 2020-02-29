#include <lsm6ds3.h>

// power on the accelerometer
void lsm6ds3_xl_power_on(int spi_fd, int gpio_fd, uint8_t config_byte){
  // Accelerometer, CTRL1_XL
  uint8_t buf[2] = {};
  buf[0] = CTRL1_XL; // register to write to
  buf[1] = config_byte;
  LSM6DS3_SET_WRITE_MODE(buf[0]);

  gpio_reset(gpio_fd);
  write(spi_fd, buf, 2);
  gpio_set(gpio_fd);

}

// power on the gyroscope
void lsm6ds3_g_power_on(int spi_fd, int gpio_fd, uint8_t config_byte){
  // Gyroscope, CTRL2_G
  uint8_t buf[2] = {};
  buf[0] = CTRL2_G; // register to write to
  buf[1] = config_byte;
  LSM6DS3_SET_WRITE_MODE(buf[0]);

  gpio_reset(gpio_fd);
  write(spi_fd, buf, 2);
  gpio_set(gpio_fd);
}


/**
 * read acceleration in specified axis
 * @return acceleration in m/s
 */
float read_accel(int spi_fd, int gpio_fd, int axis, float fs){
  uint8_t buf[2] = {};
  switch(axis){
    case LSM6DS3_X_AXIS:{
      buf[0] = OUTX_L_XL;
      break;
    }case LSM6DS3_Y_AXIS:{
      buf[0] = OUTY_L_XL;
      break;
    }case LSM6DS3_Z_AXIS:{
      buf[0] = OUTY_L_XL;
      break;
    }
    default:
      return 0.0;
  }
  LSM6DS3_SET_READ_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_cmd(spi_fd, buf[0], buf, 2); // send cmd byte, read 2 bytes
  gpio_set(gpio_fd);

  int16_t val = buf[0] | (buf[1] << 8);
  return (9.81 * fs * ((float)val))/32767.0;
}

/**
 *
 */
float read_gyro(int spi_fd, int gpio_fd, int axis, float fs){
  uint8_t buf[2] = {};
  switch(axis){
    case LSM6DS3_X_AXIS:{
      buf[0] = OUTX_L_G;
      break;
    }case LSM6DS3_Y_AXIS:{
      buf[0] = OUTY_L_G;
      break;
    }case LSM6DS3_Z_AXIS:{
      buf[0] = OUTY_L_G;
      break;
    }
    default:
      return 0.0;
  }
  LSM6DS3_SET_READ_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_cmd(spi_fd, buf[0], buf, 2); // send cmd byte, read 2 bytes
  gpio_set(gpio_fd);

  int16_t val = buf[0] | (buf[1] << 8);
  return (fs * ((float)val))/32767.0;
}
