#include "interface/spi.h"

Spi::Spi(const std::string& fname) {
	if(init(fname) != 1) {
		ROS_ERROR("Could not init SPI interface on %s", fname.c_str());
	}
}

Spi::~Spi() {
	close(m_file);
}

int Spi::init(const std::string& fname) {
	ROS_INFO("Trying SPI on %s", fname.c_str());
    if ((m_file = open(fname.c_str(),O_RDWR)) < 0){
		m_error = true;
	    ROS_ERROR("SPI open failed");
        return SPI_ERR_OPEN_FAILED;
    }
    //possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
    //multiple possibilities using |

    // set default spi mode
    if (set_mode(SPI_DEFAULT_MODE) < 0){
		m_error = true;
	    ROS_ERROR("Set SPI mode failed");
        return SPI_ERR_SET_MODE_FAILED;
    }
    
    // make sure MSB is first
    uint8_t value = 0;
    if (ioctl(m_file, SPI_IOC_WR_LSB_FIRST, &value) != 0){
		m_error = true;
	    return SPI_ERR_SET_MODE_FAILED;
    }

    // set default spi clock speed
    if (set_speed(SPI_DEFAULT_SPEED) < 0){
		m_error = true;
	    ROS_ERROR("Set SPI max speed failed");
        return SPI_ERR_SET_SPEED_FAILED;
    }

	return 0;
}

int Spi::set_speed(uint32_t speed) {
	if(ioctl(m_file, SPI_IOC_WR_MAX_SPEED_HZ, &speed) != 0){
		m_error = true;
		return SPI_ERR_SET_SPEED_FAILED;
	}
	return 0;
}


int Spi::set_mode(uint32_t spi_mode) {
	if(spi_mode != SPI_MODE_0 && spi_mode != SPI_MODE_1 && spi_mode != SPI_MODE_2 && spi_mode != SPI_MODE_3){
		spi_mode = SPI_MODE_0;
	}
	if(ioctl(m_file, SPI_IOC_WR_MODE, &spi_mode) != 0) {
		m_error = true;
		return SPI_ERR_SET_MODE_FAILED;
	}
	return 0;// 0 if success
}

int Spi::cmd(uint8_t cmd, uint8_t* rpy, int rpy_len) {
	write(m_file, &cmd, 1);
	return read(m_file, rpy, rpy_len);
}

uint8_t* Spi::transfer(uint8_t* buf, int buf_len) {
	struct spi_ioc_transfer xfer[2];
	int status;
	memset(xfer, 0, sizeof xfer);
	xfer[0].tx_buf = (unsigned long)buf;
	xfer[0].rx_buf = (unsigned long)buf;
	xfer[0].len = buf_len;
	xfer[0].delay_usecs = 400;
	status = ioctl(m_file, SPI_IOC_MESSAGE(1), xfer);
	return buf;
}

