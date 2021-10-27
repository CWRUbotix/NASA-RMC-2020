//
// Created by bscholar on 10/23/21.
//
#pragma once

#include <string>
#include <cinttypes>

#define SPI_DEFAULT_MODE SPI_MODE_0


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

private:
    int init(const std::string& fname);
private:
    int m_fd;
    bool m_error;
};