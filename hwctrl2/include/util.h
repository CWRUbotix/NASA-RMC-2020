#pragma once

#include <iostream>

// for printing out size of class at compile time
template<int s> class SizeCheck;

#ifdef PRINT_SIZES
    #define SIZE(x) SizeCheck<sizeof(x)> __var;
#else
    #define SIZE(x)
#endif