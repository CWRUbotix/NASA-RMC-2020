#include <spi.h>

int spi_init(char* fname){
	int file;
	ROS_INFO("Trying SPI on %s", fname);
    if ((file = open(fname,O_RDWR)) < 0){
	    ROS_ERROR("SPI open failed");
        return SPI_ERR_OPEN_FAILED;
    }

    //possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
    //multiple possibilities using |

    // set default spi mode
    if (spi_set_mode(file, SPI_DEFAULT_MODE) < 0){
	    ROS_ERROR("Set SPI mode failed");
        return SPI_ERR_SET_MODE_FAILED;
    }
    
    // make sure MSB is first
    uint8_t value = 0;
    if (ioctl(file, SPI_IOC_WR_LSB_FIRST, &value) != 0){
	    return SPI_ERR_SET_MODE_FAILED;
    }

    // set default spi clock speed
    if (spi_set_speed(file, SPI_DEFAULT_SPEED) < 0){
	    ROS_ERROR("Set SPI max speed failed");
        return SPI_ERR_SET_SPEED_FAILED;
    }
    return file;

}

int spi_set_speed(int f, uint32_t speed){
	// will return 0 if no error
	if(ioctl(f, SPI_IOC_WR_MAX_SPEED_HZ, &speed) != 0){
		return SPI_ERR_SET_SPEED_FAILED;
	}
	return 0;
}

int spi_set_mode(int f, uint8_t spi_mode){
	if(spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_1 && spi_mode != SPI_MODE_2 && spi_mode != SPI_MODE_3){
		spi_mode = SPI_MODE_0;
	}
	if(ioctl(f, SPI_IOC_WR_MODE, &spi_mode) != 0) {
		return SPI_ERR_SET_MODE_FAILED;
	}
	return 0;// 0 if success
}

/**
 * 
 */
int spi_cmd(int fd, uint8_t cmd, uint8_t * rpy, int rpy_len){
	write(fd, &cmd, 1);
	return read(fd, rpy, rpy_len);
}
