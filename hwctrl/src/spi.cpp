#include <spi.h>

int spi_init(char* fname){
	int file;
 
    if ((file = open(fname,O_RDWR)) < 0){
        ROS_INFO("Failed to open SPI bus.");
    	
        return -1;
    }

    //possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
    //multiple possibilities using |

    // set default spi mode
    if (spi_set_mode(file, SPI_DEFAULT_MODE) < 0){
        ROS_INFO("Failed to set spi mode.");
        return -1;
    }
    // set default spi clock speed
    if (spi_set_speed(file, SPI_DEFAULT_SPEED) < 0){
        ROS_INFO("Failed to set max speed hz");
        return -1;
    }
        
}

int spi_set_speed(int f, uint32_t speed){
	return ioctl(f, SPI_IOC_WR_MAX_SPEED_HZ, &speed); // will return 0 if no error
}

int spi_set_mode(int f, uint8_t spi_mode){
	if(spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_1 && spi_mode != SPI_MODE_2 && spi_mode != SPI_MODE_3){
		spi_mode = SPI_MODE_0;
	}
	return ioctl(f, SPI_IOC_WR_MODE, &spi_mode); // 0 if success
}