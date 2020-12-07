#include <gtest/gtest.h>
#include <ros/ros.h>

#include "file_test.cpp"
#include "util_test.cpp"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}