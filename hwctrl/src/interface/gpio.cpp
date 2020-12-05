#include "interface/gpio.h"

#include <ros/ros.h>

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
#include <istream>
#include <ostream>

const std::string in_str  = "in";
const std::string out_str = "out";
const std::string default_gpio_path =  "/sys/class/gpio";

Gpio::Gpio(uint32_t num, Direction dir, State state) {
    std::ostringstream s;
    s << default_gpio_path << "/gpio" << std::to_string(num) << "/";
    Gpio(s.str(), dir, state);
}

Gpio::Gpio(std::string path, Direction dir, State state)
: m_path(path)
{
    if(*path.end() != '/') {
        path = path.append("/");
    }
    
    m_dir_path =   m_path + "direction";
    m_value_path = m_path + "value";

    set_direction(dir);

    if(dir == Direction::Output) {
        set_state(state);
    }
}

void Gpio::set_direction(Direction dir) const  {
    const std::string dir_str = dir == Direction::Input ? in_str : out_str;
    std::ofstream f(m_dir_path);
    if(!f) {
        ROS_ERROR("");
    }
    f << dir_str;
    f.close();
}

Gpio::Direction Gpio::get_direction() const {
    char dir;
    std::ifstream f(m_dir_path);
    if(!f) {
        ROS_ERROR("");
    }
    f.read(&dir, 1);
    f.close();
    return (dir == 'i' ? Direction::Input : Direction::Output);
}

void Gpio::set_state(State state) const {
    const char b = (state == State::Set ? '1' : '0');
    std::ofstream f(m_value_path);
    f.write(&b, 1);
    f.close();
}

Gpio::State Gpio::read_state() const {
    char buf;
    std::ifstream f(m_value_path);
    f.read(&buf, 1);
    f.close();
    return (Gpio::State) buf;
}

