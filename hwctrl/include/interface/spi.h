#pragma once

#include <cinttypes>
#include <string>

#define SPI_DEFAULT_MODE 	SPI_MODE_0
#define SPI_DEFAULT_SPEED 	2500000

#define SPI_ERR_OPEN_FAILED       (-1)
#define SPI_ERR_SET_MODE_FAILED   (-2)
#define SPI_ERR_SET_SPEED_FAILED  (-3)
#define SPI_ERR_WRITE_FAILED      (-4)
#define SPI_ERR_READ_FAILED       (-5)

class Spi {
public:
    Spi(const std::string& fname);
    ~Spi();

    Spi(Spi const&) = delete;
    void operator=(Spi const&) = delete;

    int set_speed(uint32_t speed);
    int set_mode(uint32_t mode);
    int cmd(uint8_t cmd, uint8_t* rpy, int rpy_len);
    uint8_t* transfer(uint8_t* buf, int buf_len);

    inline bool has_error() {return m_error;}
    inline void clear_error() { m_error = false; }

private:
    int init(const std::string& fname);

private:
    int m_file;
    bool m_error;
};