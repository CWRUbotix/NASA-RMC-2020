#include <hwctrl.h>
////////////////////////////////////////////////////////////////////////////////
// SENSOR STUFF
////////////////////////////////////////////////////////////////////////////////

/**
 * SensorIf constructor
 * creates publishers, subscribers, timers, opens interfaces, initializes GPIO, etc
 */
SensorIf::SensorIf(ros::NodeHandle n) :
	loop_rate(100),
	uwb_update_period(0.1)
{
	this->nh = n;

	this->nh.setCallbackQueue(&(this->cb_queue)); // have this copy use this callback queue

	this->can_rx_sub = this->nh.subscribe("can_frames_rx", 128, &SensorIf::can_rx_callback, this);

	this->can_tx_pub = this->nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
	this->sensor_data_pub = this->nh.advertise<hwctrl::SensorData>("sensor_data", 128);
	this->limit_sw_pub = this->nh.advertise<hwctrl::LimitSwState>("limit_sw_states", 128);
	this->uwb_data_pub = this->nh.advertise<hwctrl::UwbData>("localization_data", 1024);

	this->get_sensors_from_csv(); // create all our sensor info objects and whatever

	int gpio_state = this->setup_gpio();
	if(gpio_state != 0){
		ROS_ERROR("GPIO setup failed :(");
	}else{
		ROS_DEBUG("GPIO setup succeeded!");
	}

	this->spi_handle = spi_init("/dev/spidev1.0");
	if(this->spi_handle < 0){
		ROS_ERROR("SPI init failed :(");
	}else{
		ROS_DEBUG("SPI init presumed successful");
		this->setup_spi_devices();
	}

	// setup sensors
	SensorInfo* sensor;
	for(int i = 0; i < this->n_sensors; i++){
		sensor = &(this->sensors[i]);
		if(sensor->if_type == IF_SPI && !sensor->spi_device->is_setup){
			continue; // skip making a timer for this one
		}
		sensor->update_timer = this->nh.createTimer(sensor->update_pd, &SensorInfo::set_update_flag, sensor);
		ROS_DEBUG("Sensor %d | Update Period: %.3fs", sensor->sys_id, sensor->update_pd.toSec());
	}

	// create the timer that will request data from the next UWB node
	this->uwb_update_timer = this->nh.createTimer(this->uwb_update_period, &SensorIf::uwb_update_callback, this);
}

/**
 * necessary setup of GPIO's
 */
int SensorIf::setup_gpio(){
	return 0;
}


/**
 * thread to do sensor things
 */
void sensors_thread(SensorIf* sensor_if){
	ROS_DEBUG("Starting sensors_thread");
	ros::AsyncSpinner spinner(1, &(sensor_if->cb_queue));
	spinner.start();
	while(ros::ok()){
		// check which UWB nodes have anchor data ready. read and publish them
		/*for(auto node = sensor_if->uwb_nodes.begin(); node != sensor_if->uwb_nodes.end(); ++node){
			int n_msgs = 4;
			hwctrl::UwbData uwb_msgs[n_msgs] = {};
			n_msgs = node->get_msgs_from_anchors(uwb_msgs, n_msgs);
			ROS_INFO("We have %d UWB messages to publish", n_msgs);
			for(int i = 0; i < n_msgs; i++){
				sensor_if->uwb_data_pub.publish(uwb_msgs[i]);
			}
		}*/

		// poll limit switches
		// publish any limit switches that have occurred
		// SensorInfo* sensor;
		for(int n = 0; n < sensor_if->n_sensors; n++){
			SensorInfo* sensor = &(sensor_if->sensors[n]);
			if(sensor->update && sensor->is_setup){
				// do update things based on device type
				switch(sensor->dev_type){
					case DEVICE_ADT7310:{
						SpiDevice* dev = sensor->spi_device;
						spi_set_mode(sensor_if->spi_handle, dev->spi_mode);
						spi_set_speed(sensor_if->spi_handle, dev->spi_max_speed);
						uint8_t cmd = ADT7310_CMD_READ_REG | (ADT7310_REG_TEMP_VAL << 3);
						uint8_t rpy[2];
						if(spi_cmd(sensor_if->spi_handle, cmd, rpy, 2) == 2){
							// correct number of bytes read
							int16_t raw = (rpy[0] << 8) | rpy[1];
							sensor->value = raw * ADT7310_LSB_16_BIT; // convert to celcius
							hwctrl::SensorData msg;
							msg.sensorID = sensor->sys_id;
							msg.value = sensor->value;
							sensor_if->sensor_data_pub.publish(msg);
						}
						break;
					}case DEVICE_LSM6DS3:{
						ROS_DEBUG("***** UPDATE IMU *****");
						int spi_fd = sensor_if->spi_handle;
						int gpio_fd = sensor->spi_device->gpio_value_handle;
						if(sensor->descrip.compare("accel") == 0){
							sensor->value = read_accel(spi_fd, gpio_fd, sensor->axis);
						}else if(sensor->descrip.compare("gyro") == 0){
							sensor->value = read_gyro(spi_fd, gpio_fd, sensor->axis);
						}else{
							break;
						}
						sensor->timestamp = ros::Time::now();
						hwctrl::SensorData msg;
						msg.sensorID = sensor->sys_id;
						msg.value = sensor->value;
						sensor_if->sensor_data_pub.publish(msg);
						break;
					}default:{
						break;
					}
				}

				sensor->update = false; // reset this flag
			}
		}

		sensor_if->loop_rate.sleep();
	}

}

/**
 * do things with new CAN messages, really for UWB and quadrature encoder data
 */
void SensorIf::can_rx_callback(boost::shared_ptr<hwctrl::CanFrame> frame){
	uint32_t rx_id = (uint32_t)frame->can_id;
	uint32_t can_id = 0;
	if((can_id = (rx_id & ~0xFF)) == 0){
		// was NOT a VESC frame
		// UWB frame or quad encoder frame
		UwbNode* uwb_node = this->get_uwb_by_can_id(rx_id);
		if(uwb_node != NULL){
			ROS_DEBUG("UWB frame (node id %d)", rx_id);
			// uwb_node->add_can_data(frame->data.data(), frame->can_dlc);
			hwctrl::UwbData msg;
			msg.anchor_id = frame->data[1];
			float temp = 0.0;
			memcpy(&temp, frame->data.data() + 2, 4);
			msg.distance = temp;
			msg.node_id = rx_id;
			this->uwb_data_pub.publish(msg);
		}else{
			// no UWB node matching this CAN ID, so must be our quadrature encoder
			//this->quad_encoder.add_can_data(frame->data.data(), frame->can_dlc);
		}
	}
}

/**
 * returns a pointer to the UWB node object that has the given CAN ID
 */
UwbNode* SensorIf::get_uwb_by_can_id(int can_id){
	auto i = this->uwb_nodes.begin();
	UwbNode* node = &(*i);
	while(i != this->uwb_nodes.end() && i->id != can_id){
		++i; // increment to next UWB Node
		node = &(*i);
	}
	return node;
}

/**
 * will be called by a timer
 * when called, will send a remote request to the next UWB node to get ranging data
 */
void SensorIf::uwb_update_callback(const ros::TimerEvent& tim_event){
	// ROS_INFO("===== UWB UPDATE CALLBACK =====");
	if(this->uwb_nodes.size() <= 0){
		ROS_DEBUG("No UWB nodes");
		return;
	}
	//UwbNode* nodes = this->uwb_nodes.data();
	//UwbNode* uwb_node = &(nodes[this->uwb_ind]);
	UwbNode node = this->uwb_nodes.at(this->uwb_ind);
	boost::shared_ptr<hwctrl::CanFrame> can_msg(new hwctrl::CanFrame()); // empty can frame msg built as a shared pointer
	can_msg->can_id  = node.id | CAN_RTR_FLAG; // can ID + remote request & std ID flags
	can_msg->can_dlc = 8; // I think it needs to be 8 for the legacy fw? UWB_CAN_HDDR_SIZE + 4; // this won't really be used but whatever
	this->can_tx_pub.publish(can_msg);

	this->uwb_ind++;
	if(this->uwb_ind >= this->uwb_nodes.size()){
		this->uwb_ind = 0;
	}
}

/**
 * process the hw_config.csv file to figure our what sensors are specified
 */
void SensorIf::get_sensors_from_csv(){
	std::string ros_package_path(std::getenv("ROS_PACKAGE_PATH"));
	std::istringstream path_stream(ros_package_path);

	std::string src_dir_path;
	std::getline(path_stream, src_dir_path, ':'); // get the first path
	if(*(src_dir_path.end()) != '/'){
		src_dir_path.push_back('/');
	}
	std::string config_file_path = src_dir_path.append(config_file_fname);

	std::vector<std::vector<std::string>> csv = read_csv(config_file_path);
	int line_num = 0;
	int id_ind, name_ind, category_ind, device_type_ind, interface_ind, device_id_ind, aux_1_ind, aux_2_ind, aux_3_ind, aux_4_ind;
	int sensor_ind = 0;
	for(auto line = csv.begin(); line != csv.end(); ++line){
		if(line_num == 0){
			for(int i = 0; i < (*line).size(); i++){
				if((*line)[i].compare(id_hddr)==0){
					id_ind = i;
				}else if((*line)[i].compare(name_hddr)==0){
					name_ind = i;
				}else if((*line)[i].compare(category_hddr)==0){
					category_ind = i;
				}else if((*line)[i].compare(device_type_hddr)==0){
					device_type_ind = i;
				}else if((*line)[i].compare(interface_hddr)==0){
					interface_ind = i;
				}else if((*line)[i].compare(device_id_hddr)==0){
					device_id_ind = i;
				}else if((*line)[i].compare(aux_1_hddr)==0){
					aux_1_ind = i;
				}else if((*line)[i].compare(aux_2_hddr)==0){
					aux_2_ind = i;
				}else if((*line)[i].compare(aux_3_hddr)==0){
					aux_3_ind = i;
				}else if((*line)[i].compare(aux_4_hddr)==0){
					aux_4_ind = i;
				}else{
					// do nothing
				}
			}
		}else{
			if((*line)[category_ind].compare(category_sensor) == 0){
				SensorInfo info;
				info.name = (*line)[name_ind];
				info.sys_id = std::stoi((*line)[device_id_ind]);
				info.if_type = get_if_type((*line)[interface_ind]);
				info.dev_type = get_device_type((*line)[device_type_ind]);
				info.dev_id = std::stoi((*line)[device_id_ind]);
				float pd = std::stof((*line)[aux_4_ind]);
				info.update_pd = ros::Duration(pd);

				switch(info.dev_type){
					ROS_DEBUG("Sensor: %d, %s", info.sys_id, info.name);
					case DEVICE_UWB:{
						// this line is for an UWB node
						ROS_DEBUG("An UWB node, id: %d!", info.dev_id);
						UwbNode uwb_node((uint32_t)info.dev_id);
						this->uwb_nodes.push_back(uwb_node);
						break;
					}
					case DEVICE_LSM6DS3:{
						// this line is for an imu
						info.axis = ((*line)[aux_2_ind])[0];
						info.descrip = (*line)[aux_1_ind];
						if( (*line)[aux_1_ind].compare("accel") == 0){
							// it's an acclerometer
						}else if( (*line)[aux_1_ind].compare("gyro") == 0) {
							// it's a gyroscope
						}
						info.spi_device = &(this->spi_devices[IMU_IND]); // store the pointer to the spi device
						break;
					}
					case DEVICE_ADT7310:{
						// this line is for the ebay temperature sensor
						info.spi_device = &(this->spi_devices[TEMP_SENSOR_IND]); // store the pointer to the spi device
						break;
					}
					case DEVICE_POT:{
						info.spi_device = &(this->spi_devices[ADC_1_IND]);
						if((*line)[aux_1_ind].compare("port") == 0){
							// it's on the port side
						}else if((*line)[aux_1_ind].compare("starboard") == 0){
							// it's the one on the starboard side
						}
						break;
					}
					case DEVICE_LOAD_CELL:{
						info.spi_device = &(this->spi_devices[ADC_2_IND]);
						break;
					}
					case DEVICE_POWER_SENSE:{
						// 24V Present input
						// just a gpio that tells if the e-stop is energized or not
						info.gpio_path = sys_power_on;
						gpio_set_dir(info.gpio_path, GPIO_INPUT);
						if((info.gpio_value_fd = gpio_get_value_handle(info.gpio_path)) > 0){
							// we vibin'
						}else{
							ROS_ERROR("Failed to get file descriptor for %svalue", info.gpio_path.c_str());
						}
						break;
					}
					case DEVICE_LIMIT_SW:{
						// a limit switch, high if activated
						// deal with this later
					}
				}
				// store the sensor info struct
				if(sensor_ind < MAX_NUMBER_OF_SENSORS){
					this->sensors[sensor_ind ++] = info;
					this->n_sensors = sensor_ind;
				}
				else{
					ROS_WARN("Too many sensors, can't add any more");
				}
			}

		}
		line_num++;
	}
}

void SensorIf::setup_spi_devices(){
	// SETUP DATA FOR SPI DEVICES
	SpiDevice* dev = &(spi_devices[ADC_1_IND]);
	uint8_t buf[8] = {};
	//====== ADC 1, potentiometers =======
	dev->device_type   = DEVICE_ADS1120;
  dev->gpio_path 		 = adc_1_cs;
	dev->spi_mode 		 = ADS1120_SPI_MODE;
	dev->spi_max_speed = ADS1120_SPI_SPEED;
	gpio_set_dir(dev->gpio_path, GPIO_OUTPUT);
	if((dev->gpio_value_handle = gpio_get_value_handle(dev->gpio_path)) > 0){
		gpio_set(dev->gpio_value_handle); // make CS high to disable
		spi_set_speed(this->spi_handle, dev->spi_max_speed);
		spi_set_mode(this->spi_handle, dev->spi_mode);

		buf[0] = ADS1120_CMD_RESET;
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 1); // send the reset byte
		gpio_set(dev->gpio_value_handle);

		buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
		buf[1] = ADS1120_PGA_BYPASS; 			// we want to bypass the PGA
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
		buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_4 | ADS1120_CM_CONT; // 330 SPS, continuous conversion
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
		buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		// initial mux configuration
		buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1;
		buf[1] = ADS1120_MUX_P1_NVSS | ADS1120_PGA_BYPASS;
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		dev->is_setup = true;
	}else{
		ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
	}

	//======= ADC 2, LOAD CELL ADC =======
	dev = &(spi_devices[ADC_2_IND]);
	dev->device_type 		= DEVICE_ADS1120;
	dev->gpio_path 			= adc_2_cs;
	dev->spi_mode 			= ADS1120_SPI_MODE;
	dev->spi_max_speed 	= ADS1120_SPI_SPEED;
	gpio_set_dir(dev->gpio_path, GPIO_OUTPUT);
	if((dev->gpio_value_handle = gpio_get_value_handle(dev->gpio_path)) > 0){

		gpio_set(dev->gpio_value_handle); // make CS high to disable
		spi_set_speed(this->spi_handle, dev->spi_max_speed);
		spi_set_mode(this->spi_handle, dev->spi_mode);

		buf[0] = ADS1120_CMD_RESET;
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 1); // send the reset byte
		gpio_set(dev->gpio_value_handle);

		buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
		buf[1] = ADS1120_MUX_P1_N2 | ADS1120_PGA_GAIN_128; // set mux, plus lotsa gain
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
		buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_1 | ADS1120_CM_CONT; // 45 SPS, continuous conversion
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
		buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
		gpio_reset(dev->gpio_value_handle); // pull CS low
		write(this->spi_handle, buf, 2); // send the bytes
		gpio_set(dev->gpio_value_handle);

		dev->is_setup = true;
	}else{
		ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
	}

	// ======= TEMP SENSOR =======
	dev = &(spi_devices[TEMP_SENSOR_IND]);
	dev->device_type 	= DEVICE_ADT7310;
	dev->gpio_path 		= temp_sensor_cs;
	dev->spi_mode 		= ADT7310_SPI_MODE;
	dev->spi_max_speed= ADT7310_SPI_SPEED;
	gpio_set_dir(dev->gpio_path, GPIO_OUTPUT);
	if((dev->gpio_value_handle = gpio_get_value_handle(dev->gpio_path)) > 0){
		gpio_set(dev->gpio_value_handle); // make CS high to disable
		spi_set_speed(this->spi_handle, dev->spi_max_speed);
		spi_set_mode(this->spi_handle, dev->spi_mode);
		buf[0] = ADT7310_CMD_READ_REG | (ADT7310_REG_ID << 3); // read ID register

		gpio_reset(dev->gpio_value_handle);
		int read_len = spi_cmd(this->spi_handle, buf[0], buf, 1); // write command byte and read resp byte
		gpio_set(dev->gpio_value_handle);

		if(read_len > 0 && (buf[0] & ADT7310_MFG_ID_MASK) == ADT7310_MFG_ID){

			buf[0] = (ADT7310_REG_CONFIG << 3); // writing to config register
			buf[1] = ADT7310_FAULTS_1 | ADT7310_RES_16_BIT; // why not 16 bit?
			gpio_reset(dev->gpio_value_handle);
			write(this->spi_handle, buf, 2);
			gpio_set(dev->gpio_value_handle);

			int16_t temp;
			buf[0] = (ADT7310_REG_T_CRIT << 3); // writing to critical temperature register
			temp   = T_CRIT_16_BIT;
			buf[1] = (temp >> 8);
			buf[2] = temp & 0xFF;
			gpio_reset(dev->gpio_value_handle);
			write(this->spi_handle, buf, 3);
			gpio_set(dev->gpio_value_handle);

			buf[0] = (ADT7310_REG_T_HIGH << 3); // writing to critical temperature register
			temp   = T_HIGH_16_BIT;
			buf[1] = (temp >> 8);
			buf[2] = temp & 0xFF;
			gpio_reset(dev->gpio_value_handle);
			write(this->spi_handle, buf, 3);
			gpio_set(dev->gpio_value_handle);

			dev->is_setup = true;
		}
	}else{
		ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
	}



	// ======= IMU ========
	dev = &(spi_devices[IMU_IND]);
	dev->device_type 	= DEVICE_LSM6DS3;
	dev->gpio_path 		= imu_cs;
	dev->spi_mode 		= LSM6DS3_SPI_MODE;
	dev->spi_max_speed= LSM6DS3_SPI_SPEED;
	gpio_set_dir(dev->gpio_path, GPIO_OUTPUT);
	if((dev->gpio_value_handle = gpio_get_value_handle(dev->gpio_path)) > 0){
		gpio_set(dev->gpio_value_handle); // make CS high to disable
		spi_set_speed(this->spi_handle, dev->spi_max_speed);
		spi_set_mode(this->spi_handle, dev->spi_mode);
		buf[0] = WHO_AM_I;
		LSM6DS3_SET_READ_MODE(buf[0]);
		gpio_reset(dev->gpio_value_handle);
		spi_cmd(this->spi_handle, buf[0], buf, 1);
		gpio_set(dev->gpio_value_handle);

		if(buf[0] == LSM6DS3_WHO_AM_I_ID){
			lsm6ds3_xl_power_on(this->spi_handle, dev->gpio_value_handle, LSM6DS3_ODR_104_HZ | LSM6DS3_FS_XL_2G);
			lsm6ds3_g_power_on(this->spi_handle, dev->gpio_value_handle, LSM6DS3_ODR_104_HZ | LSM6DS3_FS_G_250_DPS);
			ROS_DEBUG("IMU setup success!");
			dev->is_setup = true;
		}else{
			ROS_ERROR("IMU setup failed :(");
		}

	}else{
		ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
	}


}

///////////////////////////////////////////////////////////////////////////////
//
//   SENSOR INFO STUFF
//
///////////////////////////////////////////////////////////////////////////////
void SensorInfo::set_update_flag(const ros::TimerEvent& event){
	this->update = true;
}
