#pragma once

#include <string>


class Gpio {
public:
    enum State {
        Reset = 0, Set
    };

    enum Direction {
        Input = 0, Output
    };
public:
    Gpio(std::string, Direction dir = Direction::Input, State state = State::Reset);
    ~Gpio() = default;

    int init(std::string, Direction, State);
    

    void set_direction(Direction);
    Direction get_direction() const;

    State read_state() const;

    void set_state(State);
    void set();
    void reset();

    void release_handle();

private:
    int get_handle();

private:
    std::string m_path;
    std::string m_dir_path;
    std::string m_value_path;
    int m_handle;
};