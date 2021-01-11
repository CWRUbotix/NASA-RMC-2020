#include <ros/ros.h>

#include <thread>

#include "threads/canbus_thread.h"
#include "threads/motor_thread.h"
#include "threads/sensor_thread.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "hwctrl");
  ros::NodeHandle nh;

  SensorThread sensor_thread(nh);
  CanbusThread canbus_thread(nh);
  MotorThread  motor_thread(nh);

  std::thread sensor_thread_obj(std::ref(sensor_thread));
  std::thread canbus_thread_obj(std::ref(canbus_thread));
  std::thread motor_thread_obj(std::ref(motor_thread));

  sensor_thread_obj.detach();
  canbus_thread_obj.detach();
  motor_thread_obj.detach();

  ros::waitForShutdown();
  return 0;
}