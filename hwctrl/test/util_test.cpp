#include <gtest/gtest.h>

#include <ros/ros.h>

TEST(Utils, Avg) {

}

int main(int argc, char** argv) {
    // make sure ros core is started before running these tests
    ros::init(argc, argv, "testing");
    testing::InitGoogleTest(argc, argv);

    return RUN_ALL_TESTS();
}