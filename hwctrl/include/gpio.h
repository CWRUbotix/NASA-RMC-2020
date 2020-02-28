#ifndef _GPIO_H_
#define _GPIO_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <unistd.h>

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

int gpio_set_dir(std::string path, int mode);

int gpio_get_value_handle(std::string path);

int gpio_set(int handle);

int gpio_reset(int handle);

int gpio_read(int handle);

#endif
