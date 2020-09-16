#ifndef _GPIO_H_
#define _GPIO_H_

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

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

#define GPIO_ERR_UNSPEC       -1
#define GPIO_ERR_BAD_HANDLE   GPIO_ERR_UNSPEC-1
#define GPIO_ERR_OPEN_FAILED  GPIO_ERR_UNSPEC-2
#define GPIO_ERR_WRITE_FAILED GPIO_ERR_UNSPEC-3
#define GPIO_ERR_READ_FAILED  GPIO_ERR_UNSPEC-4

int gpio_init(std::string path, int mode, int value);

int gpio_set_dir(std::string path, int mode);

std::string gpio_get_dir(std::string path);

int gpio_get_value_handle(std::string path);

int gpio_set(int handle);

int gpio_reset(int handle);

int gpio_read(int handle);

#endif
