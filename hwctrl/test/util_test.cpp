#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "../include/util.h"

constexpr double EPSILON = 0.00001;

TEST(UtilTest, mean_stddev) {
  std::vector<double> vec    = {1.0, 2.0, 3.0};
  auto                avg    = math::avg(vec.begin(), vec.end());
  auto                stddev = math::stddev(vec.begin(), vec.end());
  ASSERT_NEAR(avg, 2.0, EPSILON);
  ASSERT_NEAR(stddev, 1.0, EPSILON);

  vec.clear();
  for (int i = 0; i < 10; i++) vec.push_back(1.0);
  avg    = math::avg(vec.begin(), vec.end());
  stddev = math::stddev(vec.begin(), vec.end());
  ASSERT_NEAR(avg, 1.0, EPSILON);
  ASSERT_NEAR(stddev, 0.0, EPSILON);
}

TEST(UtilTest, empty_mean_stddev) {
  std::vector<double> vec = {};
  ASSERT_TRUE(std::isnan(math::avg(vec.begin(), vec.end())));
  ASSERT_TRUE(std::isnan(math::stddev(vec.begin(), vec.end())));

  vec.push_back(1.0);
  ASSERT_FALSE(std::isnan(math::avg(vec.begin(), vec.end())));
  ASSERT_TRUE(std::isnan(math::stddev(vec.begin(), vec.end())));

  vec.push_back(1.0);
  ASSERT_FALSE(std::isnan(math::avg(vec.begin(), vec.end())));
  ASSERT_FALSE(std::isnan(math::stddev(vec.begin(), vec.end())));
}

TEST(UtilTest, buffers) {
  uint8_t buf[128] = {};
  {
    // write
    int idx = 0;
    buffer::append_int32(buf, (int32_t)1, &idx);
    buffer::append_float32(buf, (float)2.5, 10.0, &idx);
  }

  {
    // read
    int idx = 0;
    ASSERT_EQ(buffer::get_int32(buf, &idx), 1);
    ASSERT_NEAR(buffer::get_float32(buf, 10.0, &idx), 2.5, EPSILON);
  }
}
