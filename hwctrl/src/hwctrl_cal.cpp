#include <hwctrl.h>

/**
 * @return number of characters written to buf
 */
int progress_bar(char * buf, float progress){
	int hashes = (int)((progress * 50.0) + 0.5); // fraction of 50, rounded
	int retval = 0;
	for(int i = 0; i < 50; i++){
		if(i < hashes){
			buf[i] = '#';
		}else{
			buf[i] = '.';
		}
		retval ++;
	}
	retval += sprintf(buf+retval, " %d%%", (int)((progress*100.0)+0.5));
}

/**
 * thread to do sensor things
 */
void sensors_cal_thread(SensorIf* sensor_if){
	ROS_DEBUG("Starting sensors_thread");
	ros::AsyncSpinner spinner(1, &(sensor_if->cb_queue));
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
	std::string cal_file_path = home_path.append(cal_file_default);

	// begin main loop
	while(ros::ok() && !done){
		std::cout << ">>> ";
		std::getline(std::cin, line);
		if(line.compare("imu") == 0){
			std::cout << "Beginning IMU calibration\r\n";
			std::cout << "Number of samples: ";
			int sample_depth, start;
			std::cin >> sample_depth;
			if(sample_depth <= 0){
				sample_depth = 2500;
			}
			std::cout << "Will collect " << sample_depth << " samples" << endl;
			std::cout << "Press ENTER to start...";
			getchar();
			
			// locate the imu object
			bool imu_found = false;
			auto imu = sensor_if->sensors_vect.begin();
			while(!(imu_found = (imu->name.compare("IMU") == 0)) && imu != sensor_if->sensors_vect.end()){
				imu++;
			}
			
			if(imu_found){
				int spi_fd = sensor_if->spi_handle;
				int cs_fd = imu->spi_device->gpio_value_handle;
				char buf[128]; // for printing to the screen
				spi_set_mode(spi_fd, imu->spi_device->spi_mode);
				spi_set_speed(spi_fd, imu->spi_device->spi_max_speed);
				ros::Duration(0.0005).sleep();
				std::cout << "\r\nSampling IMU accelerometer..." << endl;
				float * x_xl_samples 	= (float*)calloc(sample_depth, sizeof(float));
				float * y_xl_samples 	= (float*)calloc(sample_depth, sizeof(float));
				float * z_xl_samples 	= (float*)calloc(sample_depth, sizeof(float));
				float * x_xl_smooth 	= (float*)calloc(sample_depth, sizeof(float));
				float * y_xl_smooth 	= (float*)calloc(sample_depth, sizeof(float));
				float * z_xl_smooth 	= (float*)calloc(sample_depth, sizeof(float));
				double xl_data[3];
				double gyro_data[3];
				for(int i = 0; i < sample_depth; i++){
					lsm6ds3_read_all_data(spi_fd, cs_fd, xl_data, gyro_data);
					x_xl_samples[i] = (float)xl_data[0];
					y_xl_samples[i] = (float)xl_data[1];
					z_xl_samples[i] = (float)xl_data[2];
					ros::Duration(0.005).sleep();
					progress_bar(buf, (float)(1.0*i/sample_depth));
					std::cout << '\r' << buf;
				}
				std::cout << endl << "Done sampling." << endl;
				smooth_data(x_xl_samples, x_xl_smooth, sample_depth, 9); // gaussian kernel size 9
				smooth_data(y_xl_samples, y_xl_smooth, sample_depth, 9); // gaussian kernel size 9
				smooth_data(z_xl_samples, z_xl_smooth, sample_depth, 9); // gaussian kernel size 9
				
				int z_max_ind = fmax(z_xl_smooth, sample_depth);
				float z_max = z_xl_smooth[z_max_ind];
				float y_offset = y_xl_smooth[z_max_ind];
				float x_offset = x_xl_smooth[z_max_ind];
				float z_offset = z_max - 9.81;
				printf("Max acceleration in Z: %.5g\r\n", z_max);
				printf("Inferred X offset: %.5g\r\n", x_offset);
				printf("Inferred Y offset: %.5g\r\n", y_offset);
				printf("Inferred Z offset: %.5g\r\n", z_offset);
				Calibration z_xl_cal = {
					.name = std::string("IMU_XL_Z"),
					.scale = 1.0,
					.offset = 0.0 - z_offset,
					.variance = LSM6DS3_XL_VAR
				};
				Calibration x_xl_cal = {
					.name = std::string("IMU_XL_X"),
					.scale = 1.0,
					.offset = 0.0 - x_offset,
					.variance = LSM6DS3_XL_VAR
				};
				Calibration y_xl_cal = {
					.name = std::string("IMU_XL_Y"),
					.scale = 1.0,
					.offset = 0.0 - y_offset,
					.variance = LSM6DS3_XL_VAR
				};

				for(auto cal = cals.begin(); cal != cals.end(); ++cal){
					if( (*cal).name.compare("IMU_XL_X") == 0 ||
						(*cal).name.compare("IMU_XL_Y") == 0 ||
						(*cal).name.compare("IMU_XL_Z") == 0)
					{
						cals.erase(cal);
					}
				}
				cals.push_back(z_xl_cal);
				cals.push_back(x_xl_cal);
				cals.push_back(y_xl_cal);

				// FREE MEMORY
				free(x_xl_samples);
				free(y_xl_samples);
				free(z_xl_samples);
				free(x_xl_smooth);
				free(y_xl_smooth);
				free(z_xl_smooth);

				std::cout << "Press ENTER to start Gyroscope calibration." << endl;
				getchar();
				std::cout << "Now DON'T MOVE THE BOARD!" << endl;
				
				float * x_g_samples 	= (float*)calloc(sample_depth, sizeof(float));
				float * y_g_samples 	= (float*)calloc(sample_depth, sizeof(float));
				float * z_g_samples 	= (float*)calloc(sample_depth, sizeof(float));
				float * x_g_smooth 		= (float*)calloc(sample_depth, sizeof(float));
				float * y_g_smooth 		= (float*)calloc(sample_depth, sizeof(float));
				float * z_g_smooth 		= (float*)calloc(sample_depth, sizeof(float));
				for(int i = 0; i < sample_depth; i++){
					lsm6ds3_read_all_data(spi_fd, cs_fd, xl_data, gyro_data);
					x_g_samples[i] = (float)gyro_data[0];
					y_g_samples[i] = (float)gyro_data[1];
					z_g_samples[i] = (float)gyro_data[2];
					ros::Duration(0.005).sleep();
					progress_bar(buf, (float)(1.0*i/sample_depth));
					std::cout << "Sampling " << buf << '\r';
				}
				std::cout << endl << "Done sampling." << endl;
				// smooth_data(x_g_samples, x_g_smooth, sample_depth, 9); // gaussian kernel size 9
				// smooth_data(y_g_samples, y_g_smooth, sample_depth, 9); // gaussian kernel size 9
				// smooth_data(z_g_samples, z_g_smooth, sample_depth, 9); // gaussian kernel size 9
				
				float x_g_var 		= pow(fstddev(x_g_samples, sample_depth), 2.0);
				float y_g_var 		= pow(fstddev(y_g_samples, sample_depth), 2.0);
				float z_g_var 		= pow(fstddev(z_g_samples, sample_depth), 2.0);
				float x_g_offset 	= favg(x_g_samples, sample_depth);
				float y_g_offset 	= favg(y_g_samples, sample_depth);
				float z_g_offset 	= favg(z_g_samples, sample_depth);
				printf("Name \tOffset\tVariance\r\n");
				printf("Gyro X:\t%.5g\t%.5g\r\n", x_g_offset, x_g_var);
				printf("Gyro Y:\t%.5g\t%.5g\r\n", y_g_offset, y_g_var);
				printf("Gyro Z:\t%.5g\t%.5g\r\n", z_g_offset, z_g_var);
				Calibration cal_g_x = {
					.name = std::string("IMU_G_X"),
					.scale = 1.0,
					.offset = 0.0 - x_g_offset,
					.variance = x_g_var
				};
				Calibration cal_g_y = {
					.name = std::string("IMU_G_Y"),
					.scale = 1.0,
					.offset = 0.0 - y_g_offset,
					.variance = y_g_var
				};
				Calibration cal_g_z = {
					.name = std::string("IMU_G_Z"),
					.scale = 1.0,
					.offset = 0.0 - z_g_offset,
					.variance = z_g_var
				};
				
				for(auto cal = cals.begin(); cal != cals.end(); ++cal){
					if( (*cal).name.compare("IMU_G_X") == 0 ||
						(*cal).name.compare("IMU_G_Y") == 0 ||
						(*cal).name.compare("IMU_G_Z") == 0)
					{
						cals.erase(cal);
					}
				}
				cals.push_back(cal_g_x);
				cals.push_back(cal_g_y);
				cals.push_back(cal_g_z);
				std::cout << "Done calibrating IMU. " << endl;
				std::cout << "If you're done calibrating stuff, run \"save\" to write calibration to disk." << endl;
				
			}else{
				std::cout << "Couldn't find the IMU object." << endl;
			}
			
		}else if(line.compare("loadcell") == 0){
			std::cout << "Empty the basket and press ENTER...";
			getchar();
			
		}else if(line.compare("load") == 0){
			//time to load the calibration from file
			printf("Reading calibration from file %s\r\n", cal_file_path);
			std::vector<std::vector<std::string>> csv_data = read_csv(cal_file_path);
			for(auto line : csv_data){
				Calibration cal;
				cal.name = line.at(0);
				cal.scale = std::stof(line.at(1));
				cal.offset = std::stof(line.at(2));
				cal.variance = std::stof(line.at(3));

				// if we have a calibration for this sensor already, erase it
				// so we can replace it with the new one
				for(auto c = cals.begin(); c != cals.end(); ++c){
					if( (*c).name.compare(cal.name) == 0 ){
						cals.erase(c);
					}
				}
				cals.push_back(cal); // place calibration into cals vector
			}
		}else if(line.compare("help") == 0){
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
			std::cout << "Saving calibration to file " << cal_file_path << "..." << endl;
			write_cal(cal_file_path, cals);
			if(file_exists(cal_file_path.c_str()) && file_size(cal_file_path.c_str()) > 0){
				std::cout << "Success! File written." << endl;
			}
		}else if(line.compare("print") == 0){
			if(cals.size() == 0){
				std::cout << "No calibrations recorded yet" << endl;
			}else{
				for(auto cal = cals.begin(); cal != cals.end(); ++cal){
					std::cout << print_cal(*cal) << endl;
				}
			}
		}else if(line.compare("quit") == 0){
			std::cout << "Quitting ..." << endl;
			done = true;
		}else{
			std::cout << "Unrecognized command" << endl;
		}
		
        
    }

	ros::shutdown();
}

int main(int argc, char** argv){
	ROS_INFO("Hardware Controller Calibration Node");
	ros::init(argc, argv, "hwctrl_cal");
	ros::NodeHandle n;

	// CREATE THE CanbusIf OBJECT
	CanbusIf canbus_if(n);

	// CREATE THE SensorIf OBJECT
	SensorIf sensor_if(n);

	// CREATE THE HwMotorIf OBJECT
	HwMotorIf motor_if(n);

	ROS_INFO("Node init success");

    std::thread sensor_thread_obj(sensors_cal_thread, &sensor_if);

	ros::waitForShutdown();
	return 0;
}