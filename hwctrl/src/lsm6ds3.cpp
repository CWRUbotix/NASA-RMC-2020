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
void lsm5ds3_g_power_on(int spi_fd, int gpio_fd, uint8_t config_byte){
  // Gyroscope, CTRL2_G
  buf[0] = CTRL2_G; // register to write to
  buf[1] = config_byte;
  LSM6DS3_SET_WRITE_MODE(buf[0]);

  gpio_reset(gpio_fd);
  write(spi_fd, buf, 2);
  gpio_set(gpio_fd);
}


/**
 * read acceleration in specified axis
 */
float read_accel(int spi_fd, int gpio_fd, int axis){
  switch(axis){
    case LSM6DS3_X_AXIS:{
      break;
    }case LSM6DS3_Y_AXIS:{
      break;
    }case LSM6DS3_Z_AXIS:{
      break;
    }
    default:
      break;
  }
}


/**
 * read just the accleration in x
 */
float read_accel_x(int spi_fd, int gpio_fd){

}

/**
 *
 */
void read_gyro(int spi_fd, int gpio_fd, float* vals){

}
