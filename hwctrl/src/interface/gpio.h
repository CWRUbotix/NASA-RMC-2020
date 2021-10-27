//
// Created by bscholar on 10/23/21.
//
#pragma once

#include <cinttypes>

constexpr uint32_t MMAP_OFFSET = 0x44C00000;
constexpr uint32_t MMAP_SIZE = 0x481AEFFF - MMAP_OFFSET;

constexpr uint32_t gpio_register_size = 4;

constexpr uint32_t GPIO_BASE = 0x44e10800;
constexpr uint32_t GPIO0 = 0x44E07000;
constexpr uint32_t GPIO1 = 0x4804C000;
constexpr uint32_t GPIO2 = 0x481AC000;
constexpr uint32_t GPIO3 = 0x481AE000;

constexpr uint32_t GPIO_OE = 0x134;
constexpr uint32_t GPIO_DATAOUT = 0x13C;
constexpr uint32_t GPIO_DATAIN = 0x138;

enum class State {
    Low = 0, High = 1
};

enum class Direction {
    Input = 0, Output = 1
};

class Gpio {
public:
    Gpio();
    ~Gpio();

private:
    void open();

private:
    uint32_t
};

class InputGpio : public Gpio {
public:
    InputGpio(bool active_low = false);
    ~InputGpio() = default;

private:

};

class OutputGpio : public Gpio {

};