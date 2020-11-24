#include "pch.h"
#include "threads/sensor_cal_thread.h"

#include <ros/ros.h>
#include <ros/spinner.h>

#include "hwctrl.h"
#include "hardware/sensor.h"

SensorCalThread::SensorCalThread(ros::NodeHandle nh)
: SensorThread(nh) {};

void SensorCalThread::operator()() {
    ROS_DEBUG("Starting sensors_thread");
	ros::AsyncSpinner spinner(1, &m_cb_queue);
	spinner.start();
	bool done = false;

    std::string line;
	std::vector<Calibration> cals; // calibrations for different sensors
	std::cout << "Welcome to the Sensor Calibration App\r\n";

	// construct the calibration file path
	std::string home_dir_path(std::getenv("HOME"));
	std::istringstream path_stream(home_dir_path);

	std::string home_path;
	std::getline(path_stream, home_path, ':'); // get the first path
	if(*(home_path.end()) != '/'){
		home_path.push_back('/');
	}
	std::string cal_file_path = home_path.append(paths::cal_file_default);

    while(ros::ok() && !done) {
        // get command from user
        std::cout << ">>> ";
        std::getline(std::cin, line);

        if(line.compare("imu") == 0) {
            calibrate_sensor_with_name("IMU", cals);
        } else if (line.compare("load_cell") == 0) {
            calibrate_sensor_with_name("LOAD_CELL", cals);
        } else if(line.compare("help") == 0){
			printf("Sensor Calibration App Commands:\r\n");
			printf("  - %s\t%s\r\n", "help", "display this menu");
			printf("  - %s\t%s\r\n", "save", "save calibrations to file");
			printf("  - %s\t%s\r\n", "load", "load calibrations from file");
			printf("  - %s\t%s\r\n", "quit", "quit the application (maybe save first)");
			printf("  - %s\t%s\r\n", "print", "print the existing calibration data");
			printf("  - %s\t%s\r\n", "imu ", "calibrate the IMU");
			printf("  - %s\t%s\r\n", "loadcell", "calibrate the load cell(s)");
		}else if(line.compare("save") == 0){
			// write calibration to file
			std::cout << "Saving calibration to file " << cal_file_path << "..." << std::endl;
			write_calibration(cal_file_path, cals);
			if(boost::filesystem::exists(cal_file_path.c_str()) && boost::filesystem::file_size(cal_file_path.c_str()) > 0){
				std::cout << "Success! File written." << std::endl;
			}
		}else if(line.compare("print") == 0){
			if(cals.size() == 0){
				std::cout << "No calibrations recorded yet" << std::endl;
			}else{
				for(auto cal = cals.begin(); cal != cals.end(); ++cal){
					std::cout << print_calibration(*cal) << std::endl;
				}
			}
		}else if(line.compare("quit") == 0){
			std::cout << "Quitting ..." << std::endl;
			done = true;
		}else{
			std::cout << "Unrecognized command" << std::endl;
		}
    }
    ros::shutdown();
}

void SensorCalThread::calibrate_sensor_with_name(boost::string_view name, std::vector<Calibration>& cals) {
    bool found = false;
    for(auto sensor : m_sensors) {
        if(name.compare(sensor->get_name()) == 0) {
            // this must be the imu
            found = true;
            sensor->calibrate(cals);
            break;
        }
    }

    if(found)
        std::cout << "Configured" << name << "!" << std::endl;
    else
        std::cout << "Could not find" << name << ": (" << std::endl;
}


