#include <lsm6ds3.h>

// power on the accelerometer
void lsm6ds3_xl_power_on(int spi_fd, int gpio_fd, uint8_t config_byte){
  // Accelerometer, CTRL1_XL
  uint8_t buf[2] = {};
  buf[0] = CTRL1_XL; // register to write to
  buf[1] = config_byte;
  //LSM6DS3_SET_WRITE_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 2);
  gpio_set(gpio_fd);

}

// power on the gyroscope
void lsm6ds3_g_power_on(int spi_fd, int gpio_fd, uint8_t config_byte){
  // Gyroscope, CTRL2_G
  uint8_t buf[2] = {};
  buf[0] = CTRL2_G; // register to write to
  buf[1] = config_byte;
  //LSM6DS3_SET_WRITE_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 2);
  gpio_set(gpio_fd);
}


/**
 * read acceleration in specified axis
 * @return acceleration in m/s
 */
float read_accel(int spi_fd, int gpio_fd, int axis, float fs){
  uint8_t buf[3] = {};
  switch(axis){
    case LSM6DS3_X_AXIS:{
      buf[0] = OUTX_L_XL;
      break;
    }case LSM6DS3_Y_AXIS:{
      buf[0] = OUTY_L_XL;
      break;
    }case LSM6DS3_Z_AXIS:{
      buf[0] = OUTZ_L_XL;
      break;
    }
    default:
      return 0.0;
  }
  LSM6DS3_SET_READ_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 3); // send cmd byte, read 2 bytes
  gpio_set(gpio_fd);

  int16_t val = buf[1] | (buf[2] << 8);
  return (fs * ((float)val))/32767.0;
}

/**
 *
 */
float read_gyro(int spi_fd, int gpio_fd, int axis, float fs){
  uint8_t buf[3] = {};
  switch(axis){
    case LSM6DS3_X_AXIS:{
      buf[0] = OUTX_L_G;
      break;
    }case LSM6DS3_Y_AXIS:{
      buf[0] = OUTY_L_G;
      break;
    }case LSM6DS3_Z_AXIS:{
      buf[0] = OUTZ_L_G;
      break;
    }
    default:
      return 0.0;
  }
  LSM6DS3_SET_READ_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 3); // send cmd byte, read 2 bytes
  gpio_set(gpio_fd);

  int16_t val = buf[1] | (buf[2] << 8);
  return DEG_TO_RAD((fs * ((float)val))/32767.0);
}

/**
 * read all gyroscope and accelerometer data in one go
 * gyroscope data will be in rad/s
 * accelerometer data will be in m/s^2
 */
void lsm6ds3_read_all_data(int spi_fd, int gpio_fd, double* xl_data, double* gyro_data){
  uint8_t buf[13];
  memset(buf, 0, 13);
  buf[0] = OUTX_L_G;
  LSM6DS3_SET_READ_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 13);
  gpio_set(gpio_fd);

  int16_t temp;
  int ind = 1;
  temp = buf[ind++];
  temp |= buf[ind++] << 8;
  gyro_data[0] = (double)(LSM6DS3_G_FS * temp)/32767.0; // x axis dps
  DEG_TO_RAD(gyro_data[0]);

  temp = buf[ind++];
  temp |= buf[ind++] << 8;
  gyro_data[1] = (double)(LSM6DS3_G_FS * temp)/32767.0; // y axis dps
  DEG_TO_RAD(gyro_data[1]);

  temp = buf[ind++];
  temp |= buf[ind++] << 8;
  gyro_data[2] = (double)(LSM6DS3_G_FS * temp)/32767.0; // z axis dps
  DEG_TO_RAD(gyro_data[2]);

  temp = buf[ind++];
  temp |= buf[ind++] << 8;
  xl_data[0] = (double)(LSM6DS3_XL_FS * temp)/32767.0; // x axis dps

  temp = buf[ind++];
  temp |= buf[ind++] << 8;
  xl_data[1] = (double)(LSM6DS3_XL_FS * temp)/32767.0; // y axis dps

  temp = buf[ind++];
  temp |= buf[ind++] << 8;
  xl_data[2] = (double)(LSM6DS3_XL_FS * temp)/32767.0; // z axis dps
}

/**
 * initiate a software reset of the LSM6DS3
 */
void lsm6ds3_soft_reset(int spi_fd, int gpio_fd){
  uint8_t buf[2];
  memset(buf, 0, 2);
  buf[0] = CTRL3_C;
  LSM6DS3_SET_READ_MODE(buf[0]);

  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 2);
  gpio_set(gpio_fd);
  
  uint8_t og_val = buf[1];
  buf[1] |= LSM6DS3_BOOT; // initiate software reset and reboot memory content
  LSM6DS3_SET_WRITE_MODE(buf[0]); // write to the same register, now with modification
  int n = 0;
  for(int i = 0; i < 10000; i++){
    n ++;
    // spin briefly
  }
  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 2);
  gpio_set(gpio_fd);

  n = 0;
  for(int i = 0; i < 10000; i++){
    n ++;
    // spin briefly
  }

  buf[0] = CTRL3_C;
  buf[1] = og_val;
  gpio_reset(gpio_fd);
  spi_transfer(spi_fd, buf, 2);
  gpio_set(gpio_fd);

}
