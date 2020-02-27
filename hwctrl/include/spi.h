#ifndef SPI_H_
#define SPI_H_

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <cinttypes>

#define SPI_DEFAULT_MODE 	SPI_MODE_0
#define SPI_DEFAULT_SPEED 	2500000

#define SPI_ERR_OPEN_FAILED       (-1)
#define SPI_ERR_SET_MODE_FAILED   (-2)
#define SPI_ERR_SET_SPEED_FAILED  (-3)
#define SPI_ERR_WRITE_FAILED      (-4)
#define SPI_ERR_READ_FAILED       (-5)

/* get the file id for the chosen spi bus */
int spi_init(char* fname);

/* set max clock speed in Hz */
int spi_set_speed(int f, uint32_t speed);

/* set the SPI mode */
int spi_set_mode(int f, uint8_t mode);

/* send a command and read reply bytes */
int spi_cmd(int fd, uint8_t cmd, uint8_t * rpy, int rpy_len);

#endif
