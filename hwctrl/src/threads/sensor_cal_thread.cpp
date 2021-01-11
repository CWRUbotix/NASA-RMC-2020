#include "threads/sensor_cal_thread.h"

#include <ros/spinner.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "hardware/sensor.h"
#include "hwctrl.h"

SensorCalThread::SensorCalThread(ros::NodeHandle nh) : SensorThread(nh, "sensor_cal_thread"){};

void SensorCalThread::setup() {
  std::cout << "Welcome to the Sensor Calibration App\r\n";
  std::string home_dir_path(std::getenv("HOME"));
  std::istringstream path_stream(home_dir_path);
  
  std::string home_path;
  std::getline(path_stream, home_path, ':');  // get the first path
  if (*(home_path.end()) != '/') {
    home_path.push_back('/');
  }
  m_cal_path = home_path.append(hwctrl::cal_file_default);
}

void SensorCalThread::update(ros::Time) {
  if(m_done) ros::shutdown();
  std::cout << ">>> ";
  std::getline(std::cin, m_line);
  if (m_line.compare("imu") == 0) {
    calibrate_sensor_with_name("IMU");
  } else if (m_line.compare("load_cell") == 0) {
    calibrate_sensor_with_name("LOAD_CELL");
  } else if (m_line.compare("help") == 0) {
    printf("Sensor Calibration App Commands:\r\n");
    printf("  - %s\t%s\r\n", "help", "display this menu");
    printf("  - %s\t%s\r\n", "save", "save calibrations to file");
    printf("  - %s\t%s\r\n", "load", "load calibrations from file");
    printf("  - %s\t%s\r\n", "quit",
           "quit the application (maybe save first)");
    printf("  - %s\t%s\r\n", "print", "print the existing calibration data");
    printf("  - %s\t%s\r\n", "imu ", "calibrate the IMU");
    printf("  - %s\t%s\r\n", "loadcell", "calibrate the load cell(s)");
  } else if (m_line.compare("save") == 0) {
    // write calibration to file
    std::cout << "Saving calibration to file " << m_cal_path<< "..."
              << std::endl;
    write_calibration(m_cal_path, m_cals);
    if (boost::filesystem::exists(m_cal_path.c_str()) &&
        boost::filesystem::file_size(m_cal_path.c_str()) > 0) {
      std::cout << "Success! File written." << std::endl;
    }
  } else if (m_line.compare("print") == 0) {
    if (m_cals.size() == 0) {
      std::cout << "No calibrations recorded yet" << std::endl;
    } else {
      for (auto cal = m_cals.begin(); cal != m_cals.end(); ++cal) {
        std::cout << print_calibration(*cal) << std::endl;
      }
    }
  } else if (m_line.compare("quit") == 0) {
    std::cout << "Quitting ..." << std::endl;
    m_done = true;
  } else {
    std::cout << "Unrecognized command" << std::endl;
  }
}

void SensorCalThread::shutdown() {}

void SensorCalThread::calibrate_sensor_with_name(boost::string_view name) {
  bool found = false;
  for (auto sensor : m_sensors) {
    if (name.compare(sensor->get_name()) == 0) {
      // this must be the imu
      found = true;
      sensor->calibrate(m_cals);
      break;
    }
  }

  if (found)
    std::cout << "Configured" << name << "!" << std::endl;
  else
    std::cout << "Could not find" << name << ": (" << std::endl;
}
