#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <string>


namespace csv {
    std::vector<std::vector<std::string>> read_csv(std::string fpath);
    bool write_csv(std::string fpath, std::vector<std::vector<std::string>> data);
}


// for printing out size of class at compile time
template<int s> class SizeCheck;

#ifdef PRINT_SIZES
    #define SIZE(x) SizeCheck<sizeof(x)> __var;
#else
    #define SIZE(x)
#endif