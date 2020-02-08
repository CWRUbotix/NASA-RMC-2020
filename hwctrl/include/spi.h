#ifndef SPI_H_
#define SPI_H_

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <cinttypes>

#define SPI_DEFAULT_MODE 	SPI_MODE_0
#define SPI_DEFAULT_SPEED 	2500000

/* get the file id for the chosen spi bus */
int spi_init(char* fname);

/* set max clock speed in Hz */
int spi_set_speed(int f, uint32_t speed);

/* set the SPI mode */
int spi_set_mode(int f, uint8_t mode);

#endif