#ifndef PARSE_CSV_H_
#define PARSE_CSV_H_

#include <ros/ros.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <vector>

std::vector<std::vector<std::string>> read_csv(std::string fpath);
bool write_csv(std::string fpath, std::vector<std::vector<std::string>> data);

#endif