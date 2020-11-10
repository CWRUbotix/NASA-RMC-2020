#include "interface/gpio.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cinttypes>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>

constexpr auto k_in_str  = "in";
constexpr auto k_out_str = "out";

Gpio::Gpio(std::string path, Direction dir, State state)
: m_path(path)
{
    // heap allocation bad
    m_dir_path   = m_path + "direction";
    m_value_path = m_path + "value";
    init(path, dir, state);
}

void Gpio::init(std::string path, Direction dir, State state) {

}


void Gpio::set_direction(Direction dir)  {
    std::string dir_str = dir == Direction::Input ? k_in_str : k_out_str;
    std::ofstream file(m_dir_path, std::ios::out);
    file << dir_str;
    file.close();
}

Gpio::Direction Gpio::get_direction() const {
    std::string retval;
    std::ifstream file(m_dir_path, std::ios::in);
    std::getline(file, retval);
    file.close();
    return retval.compare(k_in_str) == 0 ? Gpio::Direction::Input : Gpio::Direction::Output;
}

int Gpio::get_handle() {
    Direction dir = get_direction();

    if(dir == Direction::Input)
        return open(m_value_path.c_str(), O_RDONLY);
    else
        return open(m_value_path.c_str(), O_RDWR);
}

void Gpio::set_state(State state) {
    if(m_handle > 0) {
        char b = state == State::Set ? '1' : '0';
        write(m_handle, &b, 1);
    } else {
        // ROS_WARN
    }
}

void Gpio::set() {
    set_state(State::Set);
}

void Gpio::reset() {
    set_state(State::Reset);
}

Gpio::State Gpio::read_state() const {
    if(m_handle > 0) {
        uint8_t buf = 0;
        int len = read(m_handle, &buf, 1);
    };
}