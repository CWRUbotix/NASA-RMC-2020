#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#include <spi.h>
#include <gpio.h>

#define LSM6DS3_X_AXIS 0x01
#define LSM6DS3_Y_AXIS 0x02
#define LSM6DS3_Z_AXIS 0x03

#define LSM6DS3_SPI_SPEED 			2500000
#define LSM6DS3_SPI_MODE 			SPI_MODE_3
#define LSM6DS3_WHO_AM_I_ID 		0x69
#define LSM6DS3_SET_READ_MODE(b) 	b |= (1 << 7)
#define LSM6DS3_SET_WRITE_MODE(b) 	b &= ~(1 << 7)

#define GYRO_PWR_DOWN_MODE
#define GYRO_LOW_PWR_MODE
#define GYRO_NORMAL_MODE
#define GYRO_HIGH_PERF_MODE

/* CTRL1_XL */
#define LSM6DS3_ODR_104_HZ 	(0x04 << 4)
#define LSM6DS3_ODR_208_HZ 	(0x05 << 4)
#define LSM6DS3_ODR_416_HZ 	(0x06 << 4)
#define LSM6DS3_FS_XL_2G 				(0x00 << 2)
#define LSM6DS3_FS_XL_4G 				(0x02 << 2)
#define LSM6DS3_FS_XL_8G 				(0x03 << 2)
#define LSM6DS3_FS_XL_16G 			(0x01 << 2)
#define LSM6DS3_BW_XL_400_HZ 		0x00
#define LSM6DS3_BW_XL_200_HZ 		0x01
#define LSM6DS3_BW_XL_100_HZ 		0x02
#define LSM6DS3_BW_XL_50_HZ 		0x03

/* CTRL2_G */
#define LSM6DS3_FS_G_250_DPS 		(0x00 << 2)
#define LSM6DS3_FS_G_500_DPS 		(0x01 << 2)
#define LSM6DS3_FS_G_1000_DPS 	(0x02 << 2)
#define LSM6DS3_FS_G_2000_DPS 	(0x03 << 2)

typedef enum {
	FUNC_CFG_ACCESS 	= 0x01,
	SENSOR_SYNC_TIME_FRAME = 0x04,
	FIFO_CTRL_1 		= 0x06,
	FIFO_CTRL_2 		= 0x07,
	FIFO_CTRL_3 		= 0x08,
	FIFO_CTRL_4 		= 0x09,
	FIFO_CTRL_5 		= 0x0A,
	ORIENT_CFG_G 		= 0x0B,
	INT1_CTRL 			= 0x0D,
	INT2_CTRL 			= 0x0E,
	WHO_AM_I 			= 0x0F,
	CTRL1_XL 			= 0x10,
	CTRL2_G 			= 0x11,
	CTRL3_C 			= 0x12,
	CTRL4_C 			= 0x13,
	CTRL5_C 			= 0x14,
	CTRL6_C 			= 0x15,
	CTRL7_G 			= 0x16,
	CTRL8_XL 			= 0x17,
	CTRL9_XL 			= 0x18,
	CTRL10_C 			= 0x19,
	MASTER_CONFIG 		= 0x1A,
	WAKE_UP_SRC 		= 0x1B,
	TAP_SRC 			= 0x1C,
	D6D_SRC 			= 0x1D,
	STATUS_REG 			= 0x1E,
	OUT_TEMP_L 			= 0x20,
	OUT_TEMP_H 			= 0x21,
	OUTX_L_G 			= 0x22,
	OUTX_H_G 			= 0x23,
	OUTY_L_G 			= 0x24,
	OUTY_H_G 			= 0x25,
	OUTZ_L_G 			= 0x26,
	OUTZ_H_G 			= 0x27,
	OUTX_L_XL 			= 0x28,
	OUTX_H_XL 			= 0x29,
	OUTY_L_XL 			= 0x2A,
	OUTY_H_XL 			= 0x2B,
	OUTZ_L_XL 			= 0x2C,
	OUTZ_H_XL 			= 0x2D,
	SENSORHUB1_REG 		= 0x2E,
	SENSORHUB2_REG 		= 0x2F,
	SENSORHUB3_REG 		= 0x30,
	SENSORHUB4_REG 		= 0x31,
	SENSORHUB5_REG 		= 0x32,
	SENSORHUB6_REG 		= 0x33,
	SENSORHUB7_REG 		= 0x34,
	SENSORHUB8_REG 		= 0x35,
	SENSORHUB9_REG 		= 0x36,
	SENSORHUB10_REG 	= 0x37,
	SENSORHUB11_REG 	= 0x38,
	SENSORHUB12_REG 	= 0x39,
	FIFO_STATUS1 		= 0x3A,
	FIFO_STATUS2 		= 0x3B,
	FIFO_STATUS3 		= 0x3C,
	FIFO_STATUS4 		= 0x3D,
	FIFO_DATA_OUT_L 	= 0x3E,
	FIFO_DATA_OUT_H 	= 0x3F,
	TIMESTAMP0_REG 		= 0x40,
	TIMESTAMP1_REG 		= 0x41,
	TIMESTAMP2_REG 		= 0x42,
	STEP_TIMESTAMP_L 	= 0x4B,
	STEP_TIMESTAMP_H 	= 0x4C,
	SENSORHUB13_REG 	= 0x4D,
	SENSORHUB14_REG 	= 0x4E,
	SENSORHUB15_REG 	= 0x4F,
	SENSORHUB16_REG 	= 0x50,
	SENSORHUB17_REG 	= 0x51,
	SENSORHUB18_REG 	= 0x52,
	FUNC_SRC 			= 0x53,
	TAP_CFG 			= 0x58,
	TAP_THS_6D 			= 0x59,
	INT_DUR2 			= 0x5A,
	WAKE_UP_THS 		= 0x5B,
	WAKE_UP_DUR 		= 0x5C,
	FREE_FALL 			= 0x5D,
	MD1_CFG 			= 0x5E,
	MD2_CFG 			= 0x5F,
	OUT_MAG_RAW_X_L 	= 0x66,
	OUT_MAG_RAW_X_H 	= 0x67,
	OUT_MAG_RAW_Y_L 	= 0x68,
	OUT_MAG_RAW_Y_H 	= 0x69,
	OUT_MAG_RAW_Z_L 	= 0x6A,
	OUT_MAG_RAW_Z_H 	= 0x6B

}ImuReg;

void lsm6ds3_xl_power_on(int spi_fd, int gpio_fd, uint8_t config_byte);
void lsm5ds3_g_power_on(int spi_fd, int gpio_fd, uint8_t config_byte);
float read_accel(int spi_fd, int gpio_fd, int axis);
float read_gyro(int spi_fd, int gpio_fd, int axis);

#endif
