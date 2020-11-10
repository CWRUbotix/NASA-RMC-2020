#pragma once

#include <cinttypes>

#define SPI_DEFAULT_MODE 	SPI_MODE_0
#define SPI_DEFAULT_SPEED 	2500000

#define SPI_ERR_OPEN_FAILED       (-1)
#define SPI_ERR_SET_MODE_FAILED   (-2)
#define SPI_ERR_SET_SPEED_FAILED  (-3)
#define SPI_ERR_WRITE_FAILED      (-4)
#define SPI_ERR_READ_FAILED       (-5)

namespace spi {
    /* get the file id for the chosen spi bus */
    int spi_init(const char* fname);

    /* set max clock speed in Hz */
    int spi_set_speed(int f, uint32_t speed);

    /* set the SPI mode */
    int spi_set_mode(int f, uint8_t mode);

    /* send a command and read reply bytes */
    int spi_cmd(int fd, unsigned char cmd, unsigned char * rpy, int rpy_len);

    /* transfer buf_len bytes from buf, store received bytes in buf */
    unsigned char* spi_transfer(int fd, unsigned char * buf, int buf_len);
}