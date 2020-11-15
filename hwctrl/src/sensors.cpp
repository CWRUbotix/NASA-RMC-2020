#include <hwctrl.h>
////////////////////////////////////////////////////////////////////////////////
// SENSOR STUFF
////////////////////////////////////////////////////////////////////////////////

/**
 * SensorIf constructor
 * creates publishers, subscribers, timers, opens interfaces, initializes GPIO, etc
 */
SensorIf::SensorIf(ros::NodeHandle n) :
    loop_rate(1000),
    uwb_update_period(0.1)
{
    this->nh = n;

    this->nh.setCallbackQueue(&(this->cb_queue)); // have this copy use this callback queue

    this->can_rx_sub = this->nh.subscribe("can_frames_rx", 128, &SensorIf::can_rx_callback, this);

    this->can_tx_pub = this->nh.advertise<hwctrl::CanFrame>("can_frames_tx", 128);
    this->sensor_data_pub = this->nh.advertise<std_msgs::Float32>("sensor_data", 128);

    this->limit_sw1_pub = this -> nh.advertise<std_msgs::Bool>("/dumper/dep_bottom_limit_switch", 128);
    this->limit_sw2_pub = this -> nh.advertise<std_msgs::Bool>("/dumper/top_limit_switch", 128);
    this->limit_sw3_pub = this -> nh.advertise<std_msgs::Bool>("/excavation/exc_lower_limit_switch", 128);
    this->limit_sw4_pub = this -> nh.advertise<std_msgs::Bool>("/excavation/exc_upper_limit_switch", 128);


    this->uwb_data_pub = this->nh.advertise<hwctrl::UwbData>("localization_data", 1024);


    this->imu_data_pub = this->nh.advertise<sensor_msgs::Imu>("/realsense/imu/data_raw", 128);

    this->ebay_temperature_pub = this->nh.advertise<std_msgs::Float32>("ebay_temperature_pub",128);

    // create all our sensor info objects and whatever
    // this step also links the spi devices to the correct SensorInfo objects
    // also gets file descriptor handle for GPIOI based sensors
    // this->get_sensors_from_csv(); 
    this->get_sensor_configs(); // reads from parameter server

    // ==== read in sensor calibrations ====
    // construct the calibration file path
    std::istringstream path_stream(std::string(std::getenv("HOME")));

    std::string cal_file_path;
    std::getline(path_stream, cal_file_path, ':'); // get the first path
    if(*(cal_file_path.end()) != '/'){
        cal_file_path.push_back('/');
    }
    cal_file_path.append(cal_file_default.c_str());

    if(file_exists(cal_file_path.c_str())){
        ROS_INFO("Loading sensor calibrations from %s...", cal_file_path.c_str());
        std::vector<Calibration> cals;
        read_cal(cal_file_path, cals);

        for(auto cal : cals){
            std::vector<SensorInfo>::iterator sensor = this->sensors_vect.begin();
            int pos = std::string::npos;
            // if sensor name exists within calibration name
            while( (pos = cal.name.find(sensor->name)) == std::string::npos && sensor != this->sensors_vect.end()){
                sensor++;
            }
            if(pos != std::string::npos){
                sensor->calibrations.push_back(cal);
            }
        }
    }

    // ==== initialize SPI bus ====
    ROS_DEBUG("Initializing SPI bus from %s...", spidev_path.c_str());
    this->spi_handle = spi_init(spidev_path.c_str());
    if(this->spi_handle < 0){
        ROS_ERROR("SPI init failed :(");
    }else{
        ROS_INFO("SPI bus initialization on %s presumed successful: %d", spidev_path.c_str(), this->spi_handle);
        this->setup_spi_devices();
    }

    // additional sensor setup, including making update timers
    for(std::vector<SensorInfo>::iterator sensor = this->sensors_vect.begin(); sensor != this->sensors_vect.end(); sensor++){
        if(sensor->if_type == IF_SPI && !sensor->spi_device->is_setup){
            continue; // skip making a timer for this one, setup remains false
        }else if(sensor->if_type == IF_GPIO && sensor->gpio_value_fd <= 0){
            sensor->is_setup = false;
            ROS_WARN("Sensor %s not setup.", sensor->name.c_str());
            continue; // not good-to-go
        }
        if(sensor->dev_type == DEVICE_LSM6DS3){
            sensor->imu = new ImuData;
        }
        if(sensor->update_pd.toSec() > 0.0){
            sensor->update_timer = this->nh.createTimer(sensor->update_pd, &SensorInfo::set_update_flag, &(*sensor));
        }
        sensor->is_setup = true;
        ROS_INFO("Sensor %s | Update Period: %.3fs", sensor->name.c_str(), sensor->update_pd.toSec());
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
    ROS_INFO("Starting sensors_thread");
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
        //for(int n = 0; n < sensor_if->n_sensors; n++){
        for(std::vector<SensorInfo>::iterator sensor = sensor_if->sensors_vect.begin(); sensor != sensor_if->sensors_vect.end(); sensor++){
            //SensorInfo* sensor = &(sensor_if->sensors[n]);
            if(sensor->is_setup && sensor->update){
                // do update things based on device type
                switch(sensor->dev_type){
                    case DEVICE_ADT7310:{
                        SpiDevice* dev = sensor->spi_device;
                        spi_set_mode(sensor_if->spi_handle, dev->spi_mode);
                        spi_set_speed(sensor_if->spi_handle, dev->spi_max_speed);
                        uint8_t spi_buf[3];
                        memset(spi_buf, 0, 3);
                        spi_buf[0] = ADT7310_CMD_READ_REG | (ADT7310_REG_TEMP_VAL << 3);
                        gpio_reset(dev->gpio_value_handle);
                        spi_transfer(sensor_if->spi_handle, spi_buf, 3);
                        gpio_set(dev->gpio_value_handle);
                        int16_t raw = (spi_buf[1] << 8) | spi_buf[2];
                        sensor->value = (float)raw * ADT7310_LSB_16_BIT; // convert to celcius
                        hwctrl::SensorData msg;
                        msg.sensor_id = sensor->sys_id;
                        msg.value = sensor->value;
                        msg.name = sensor->name;
                        sensor_if->sensor_data_pub.publish(msg);
                        break;
                    }case DEVICE_LSM6DS3:{
                        // ROS_INFO("***** UPDATE IMU *****");
                        SpiDevice* dev = sensor->spi_device;
                        int spi_fd = sensor_if->spi_handle;
                        int gpio_fd = dev->gpio_value_handle;
                        double xl_data[3];
                        double gyro_data[3];
                        spi_set_mode(sensor_if->spi_handle, dev->spi_mode);
                        spi_set_speed(sensor_if->spi_handle, dev->spi_max_speed);
                        ros::Duration(0.0005).sleep();
                        lsm6ds3_read_all_data(spi_fd, gpio_fd, xl_data, gyro_data);
                        sensor_msgs::Imu msg;
                        msg.header.seq  = sensor->seq++; // just a counter, could help track missed messages?
                        msg.header.stamp = ros::Time::now();
                        msg.header.frame_id = sensor->name;
                        msg.orientation_covariance[0] = -1.0; // indicates that we don't provide orientation data (yet)
                        if(sensor->calibrations.size() >= 6){
                            Calibration& cal_xl_x = get_cal_by_name("IMU_XL_X", sensor->calibrations);
                            Calibration& cal_xl_y = get_cal_by_name("IMU_XL_Y", sensor->calibrations);
                            Calibration& cal_xl_z = get_cal_by_name("IMU_XL_Z", sensor->calibrations);
                            Calibration& cal_g_x = get_cal_by_name("IMU_G_X", sensor->calibrations);
                            Calibration& cal_g_y = get_cal_by_name("IMU_G_Y", sensor->calibrations);
                            Calibration& cal_g_z = get_cal_by_name("IMU_G_Z", sensor->calibrations);
                            msg.linear_acceleration.x = cal_xl_x.scale*xl_data[0] + cal_xl_x.offset;
                            msg.linear_acceleration.y = cal_xl_y.scale*xl_data[1] + cal_xl_y.offset;
                            msg.linear_acceleration.z = cal_xl_z.scale*xl_data[2] + cal_xl_z.offset;
                            msg.angular_velocity.x = cal_g_x.scale*gyro_data[0] + cal_g_x.offset;
                            msg.angular_velocity.y = cal_g_y.scale*gyro_data[1] + cal_g_y.offset;
                            msg.angular_velocity.z = cal_g_z.scale*gyro_data[2] + cal_g_z.offset;
                        }else{
                            msg.linear_acceleration.x = xl_data[0];
                            msg.linear_acceleration.y = xl_data[1];
                            msg.linear_acceleration.z = xl_data[2];
                            msg.angular_velocity.x = gyro_data[0];
                            msg.angular_velocity.y = gyro_data[1];
                            msg.angular_velocity.z = gyro_data[2];
                        }
                        
                        msg.linear_acceleration_covariance = sensor->imu->cov_xl;
                        msg.angular_velocity_covariance  = sensor->imu->cov_g;

                        sensor->imu->sample_buf_1[sensor->imu->sample_ind] = (float)xl_data[0];
                        sensor->imu->sample_buf_2[sensor->imu->sample_ind] = (float)xl_data[1];
                        sensor->imu->sample_buf_3[sensor->imu->sample_ind] = (float)xl_data[2];
                        sensor->imu->sample_buf_4[sensor->imu->sample_ind] = (float)gyro_data[0];
                        sensor->imu->sample_buf_5[sensor->imu->sample_ind] = (float)gyro_data[1];
                        sensor->imu->sample_buf_6[sensor->imu->sample_ind] = (float)gyro_data[2];

                        if(++sensor->imu->sample_ind >= IMU_SAMPLES){
                            sensor->imu->sample_ind = 0;

                            sensor->imu->value_rm_1 = get_running_mean(sensor->imu->sample_buf_1, IMU_SAMPLES);
                            sensor->imu->value_rm_2 = get_running_mean(sensor->imu->sample_buf_2, IMU_SAMPLES);
                            sensor->imu->value_rm_3 = get_running_mean(sensor->imu->sample_buf_3, IMU_SAMPLES);
                            sensor->imu->value_rm_4 = get_running_mean(sensor->imu->sample_buf_4, IMU_SAMPLES);
                            sensor->imu->value_rm_5 = get_running_mean(sensor->imu->sample_buf_5, IMU_SAMPLES);
                            sensor->imu->value_rm_6 = get_running_mean(sensor->imu->sample_buf_6, IMU_SAMPLES);

                            // update covariance matrix

                        }
                        sensor_if->imu_data_pub.publish(msg);
                        break;
                    }case DEVICE_POWER_SENSE:{
                        int val = gpio_read(sensor->gpio_value_fd);
                        hwctrl::SensorData msg;
                        msg.sensor_id = sensor->sys_id;
                        msg.name = sensor->name;
                        msg.value = (float)val;
                        sensor_if->sensor_data_pub.publish(msg);
                        break;
                    }case DEVICE_LIMIT_SW:{
                        int val = gpio_read(sensor->gpio_value_fd);
                        boost::shared_ptr<hwctrl::LimitSwState> msg(new hwctrl::LimitSwState());
                        msg->id = sensor->sys_id;
                        msg->state = (val == 1);
                        msg->timestamp = ros::Time::now();
                        sensor_if->limit_sw1_pub.publish(msg);  // TODO this needs to be seperate topics for each switch
                        break;
                    }default:{
                        break;
                    }
                } // end switch
                sensor->update = false;
            } // end if is_setup and mtx.try_lock()
        } // end for each sensor

        sensor_if->loop_rate.sleep();
    }
    sensor_if->shutdown();

}

/**
 * do things with new CAN messages, really for UWB and quadrature encoder data
 */
void SensorIf::can_rx_callback(boost::shared_ptr<hwctrl::CanFrame> frame){
    uint32_t rx_id = (uint32_t)frame->can_id;
    uint32_t can_id = (rx_id & 0xFF);
    UwbNode& uwb_node = this->get_uwb_by_can_id(can_id);
    if(uwb_node.id == can_id){
        ROS_DEBUG("UWB frame (node id %d)", can_id);
        // uwb_node->add_can_data(frame->data.data(), frame->can_dlc);
        hwctrl::UwbData msg;
        msg.anchor_id = frame->data[1];
        float temp = 0.0;
        memcpy(&temp, frame->data.data() + 2, 4);
        msg.distance = temp;
        msg.node_id = can_id;
        this->uwb_data_pub.publish(msg);
    }else{
        // no UWB node matching this CAN ID, so could our quadrature encoder
        //this->quad_encoder.add_can_data(frame->data.data(), frame->can_dlc);
    }
    
}

/**
 * returns a reference to the UWB node object that has the given CAN ID
 * if not found, returns last element by default
 */
UwbNode& SensorIf::get_uwb_by_can_id(int can_id){
    int i = 0;
    int len = this->uwb_nodes.size();
    while(i < len && this->uwb_nodes.at(i).id != can_id){
        i++; // increment to next UWB Node
    }
    if(i < len){
        return this->uwb_nodes.at(i);
    }else{
        // we hit the end with no luck
        return this->uwb_nodes.at(len-1);
    }
}

/**
 * returns a reference to the SensorInfo object with the given name
 * returns last element by default if not found
 */
SensorInfo& SensorIf::get_sensor_by_name(std::string name){
    int i = 0;
    int len = this->sensors_vect.size();
    while(i < len && this->sensors_vect.at(i).name.compare(name) != 0){
        i++;
    }
    if(i < len){
        return this->sensors_vect.at(i);
    }else{
        // we hit the end with no luck, default to returning last element
        return this->sensors_vect.at(len-1);
    }
    
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
    UwbNode& node = this->uwb_nodes.at(this->uwb_ind);
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
 * @brief setup sensor configurations accessed from parameter server
 */
void SensorIf::get_sensor_configs(){
    ROS_INFO("Reading sensor configs from param server...");
    std::string base = param_base + "/sensor";
    for(auto name = sensor_param_names.begin(); name != sensor_param_names.end(); ++name){
        std::string full_name = base + "/" + *name;
        ROS_INFO("Checking for parameters under %s/...", full_name.c_str());
        std::string name_param = full_name + "/name";
        std::string type_param = full_name + "/type";
        std::string can_id_param = full_name + "/can_id";
        std::string interface_param = full_name + "/interface";
        std::string period_param = full_name + "/update_period";
        std::string gpio_param = full_name + "/gpio";

        SensorInfo sensor;
        if(this->nh.hasParam(name_param)){
            this->nh.getParam(name_param, sensor.name);
            ROS_INFO(" - Found sensor name: %s", sensor.name.c_str());
        }
        if(this->nh.hasParam(type_param)){
            std::string type_str;
            this->nh.getParam(type_param, type_str);
            ROS_INFO(" - Found sensor type: %s", type_str.c_str());
            sensor.dev_type = get_device_type(type_str);
        }
        if(this->nh.hasParam(interface_param)){
            std::string if_str;
            this->nh.getParam(interface_param, if_str);
            ROS_INFO(" - Found sensor interface: %s", if_str.c_str());
            sensor.if_type = get_if_type(if_str);
        }
        if(this->nh.hasParam(period_param)){
            double update_pd;
            this->nh.getParam(period_param, update_pd);
            ROS_INFO(" - Found sensor update period: %.3fs", update_pd);
            sensor.update_pd = ros::Duration(update_pd);
        }
        if(this->nh.hasParam(can_id_param)){
            this->nh.getParam(can_id_param, sensor.dev_id);
            ROS_INFO(" - Found CAN id: %d", sensor.dev_id);
        }
        if(this->nh.hasParam(gpio_param)){
            this->nh.getParam(gpio_param, sensor.gpio_path);
            ROS_INFO(" - Found gpio file path: %s", sensor.gpio_path.c_str());
        }

        switch(sensor.dev_type){
            case DEVICE_UWB:{
                // this sensor is an UWB node
                ROS_DEBUG("An UWB node, id: %d!", sensor.dev_id);
                UwbNode uwb_node((uint32_t)sensor.dev_id);
                this->uwb_nodes.push_back(uwb_node);
                break;
            }
            case DEVICE_LSM6DS3:{
                sensor.spi_device = &(this->spi_devices[IMU_IND]); // store the pointer to the spi device
                break;
            }
            case DEVICE_ADT7310:{
                // this line is for the ebay temperature sensor
                sensor.spi_device = &(this->spi_devices[TEMP_SENSOR_IND]); // store the pointer to the spi device
                break;
            }
            case DEVICE_POT:{
                sensor.spi_device = &(this->spi_devices[ADC_1_IND]);
                break;
            }
            case DEVICE_LOAD_CELL:{
                sensor.spi_device = &(this->spi_devices[ADC_2_IND]);
                break;
            }
            case DEVICE_POWER_SENSE:{
                // 24V Present input
                // just a gpio that tells if the e-stop is energized or not
                ROS_INFO("System power sense on GPIO %s", sensor.gpio_path.c_str());
                if((sensor.gpio_value_fd = gpio_init(sensor.gpio_path, GPIO_INPUT, 0)) <= 0){
                    ROS_WARN("Failed to get file descriptor for %svalue", sensor.gpio_path.c_str());
                }
                break;
            }
            case DEVICE_LIMIT_SW:{
                // a limit switch, high if activated
                ROS_INFO("Limit switch on GPIO %s", sensor.gpio_path.c_str());
                if((sensor.gpio_value_fd = gpio_init(sensor.gpio_path, GPIO_INPUT, 0)) <= 0){
                    ROS_WARN("Failed to get file descriptor for %svalue", sensor.gpio_path.c_str());
                }
                break;
            }
        }

        this->sensors_vect.push_back(sensor);
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
                info.sys_id = std::stoi((*line)[id_ind]);
                info.if_type = get_if_type((*line)[interface_ind]);
                info.dev_type = get_device_type((*line)[device_type_ind]);
                info.update_pd = ros::Duration(std::stof((*line)[aux_4_ind]));

                switch(info.dev_type){
                    ROS_DEBUG("Sensor: %d, %s", info.sys_id, info.name);
                    case DEVICE_UWB:{
                        // this line is for an UWB node
                        info.dev_id = std::stoi((*line)[device_id_ind]);
                        ROS_DEBUG("An UWB node, id: %d!", info.dev_id);
                        UwbNode uwb_node((uint32_t)info.dev_id);
                        this->uwb_nodes.push_back(uwb_node);
                        break;
                    }
                    case DEVICE_LSM6DS3:{
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
                        ROS_INFO("System power sense on GPIO %s", info.gpio_path.c_str());
                        if((info.gpio_value_fd = gpio_init(info.gpio_path, GPIO_INPUT, 0)) <= 0){
                            ROS_WARN("Failed to get file descriptor for %svalue", info.gpio_path.c_str());
                        }
                        break;
                    }
                    case DEVICE_LIMIT_SW:{
                        // a limit switch, high if activated
                        info.gpio_path = sys_gpio_base + (*line)[device_id_ind];
                        ROS_INFO("Limit switch on GPIO %s", info.gpio_path.c_str());
                        if((info.gpio_value_fd = gpio_init(info.gpio_path, GPIO_INPUT, 0)) <= 0){
                            ROS_WARN("Failed to get file descriptor for %svalue", info.gpio_path.c_str());
                        }
                        break;
                    }
                }
                // store the sensor info struct
                this->sensors_vect.push_back(info);
            }

        }
        line_num++;
    }
}

/**
 * goes through the array of spi devices and sets them up depending on their type
 * here's where lots of behavior and configs get hard-coded in :)
 */
void SensorIf::setup_spi_devices(){
    // SET CS LINES HIGH
    for(int i = 0; i < NUMBER_OF_SPI_DEVICES; i++){
        SpiDevice* dev = &(spi_devices[i]);
        if(i == ADC_1_IND){
            dev->gpio_path = adc_1_cs;
            dev->device_type   = DEVICE_ADS1120;
            dev->spi_mode        = ADS1120_SPI_MODE;
            dev->spi_max_speed = ADS1120_SPI_SPEED;
        }else if(i == ADC_2_IND){
            dev->gpio_path = adc_2_cs;
            dev->device_type        = DEVICE_ADS1120;
            dev->spi_mode           = ADS1120_SPI_MODE;
            dev->spi_max_speed  = ADS1120_SPI_SPEED;
        }else if(i == TEMP_SENSOR_IND){
            dev->gpio_path = temp_sensor_cs;
            dev->device_type    = DEVICE_ADT7310;
            dev->spi_mode       = ADT7310_SPI_MODE;
            dev->spi_max_speed= ADT7310_SPI_SPEED;
        }else if(i == IMU_IND){
            dev->gpio_path = imu_cs;
            dev->device_type    = DEVICE_LSM6DS3;
            dev->spi_mode       = LSM6DS3_SPI_MODE;
            dev->spi_max_speed  = LSM6DS3_SPI_SPEED;
        }else{
            continue;
        }
        dev->gpio_value_handle = gpio_init(dev->gpio_path, GPIO_OUTPUT, 1);
        // dev->gpio_value_handle = gpio_get_value_handle(dev->gpio_path);
        // gpio_set(dev->gpio_value_handle);
        // gpio_set_dir(dev->gpio_path, GPIO_OUTPUT);
    }

    // SETUP DATA FOR SPI DEVICES
    SpiDevice* dev = &(spi_devices[ADC_1_IND]);
    uint8_t buf[8] = {};
    //====== ADC 1, potentiometers =======
    if((dev->gpio_value_handle) > 0){
        spi_set_speed(this->spi_handle, dev->spi_max_speed);
        spi_set_mode(this->spi_handle, dev->spi_mode);

        buf[0] = ADS1120_CMD_RESET;
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 1); // send the reset byte
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
        buf[1] = ADS1120_PGA_BYPASS;            // we want to bypass the PGA
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
        buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_4 | ADS1120_CM_CONT; // 330 SPS, continuous conversion
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
        buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        // initial mux configuration
        buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1;
        buf[1] = ADS1120_MUX_P1_NVSS | ADS1120_PGA_BYPASS;
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        dev->is_setup = true;
    }else{
        ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
    }

    //======= ADC 2, LOAD CELL ADC =======
    dev = &(spi_devices[ADC_2_IND]);
    if((dev->gpio_value_handle) > 0){
        ROS_INFO("Setting up the load cell ADC");
        spi_set_speed(this->spi_handle, dev->spi_max_speed);
        spi_set_mode(this->spi_handle, dev->spi_mode);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_RESET;
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 1); // send the reset byte
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_WREG | 1; // writing to register 0x00, 1 byte
        buf[1] = ADS1120_MUX_P1_N2 | ADS1120_PGA_GAIN_128; // set mux, plus lotsa gain
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_WREG | (0x01 << 2) | 1; // register 0x01, 1 byte
        buf[1] = ADS1120_MODE_NORMAL | ADS1120_DR_1 | ADS1120_CM_CONT; // 45 SPS, continuous conversion
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        buf[0] = ADS1120_CMD_WREG | (0x02 << 2) | 1; // reg 0x02, 1 byte
        buf[1] = ADS1120_VREF_AIN0_AIN3; // set reference to AIN0 and AIN3
        gpio_reset(dev->gpio_value_handle); // pull CS low
        spi_transfer(this->spi_handle, buf, 2); // send the bytes
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.001).sleep();

        dev->is_setup = true;
    }else{
        ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
    }

    // ======= TEMP SENSOR =======
    dev = &(spi_devices[TEMP_SENSOR_IND]);
    if((dev->gpio_value_handle) > 0){
        ROS_INFO("Setting up the ADT7310");
        ros::Duration(0.001).sleep();
        spi_set_speed(this->spi_handle, dev->spi_max_speed);
        spi_set_mode(this->spi_handle, dev->spi_mode);
        buf[0] = ADT7310_CMD_READ_REG | (ADT7310_REG_ID << 3); // read ID register
        buf[1] = 0x00;

        gpio_reset(dev->gpio_value_handle);
        // ros::Duration(0.005).sleep();
        spi_transfer(this->spi_handle, buf, 2); // write command byte and read resp byte
        gpio_set(dev->gpio_value_handle);

        if((buf[1] & ADT7310_MFG_ID_MASK) == ADT7310_MFG_ID){
            ROS_INFO("Great, ADT7310 Mfg ID read correct!");
            buf[0] = (ADT7310_REG_CONFIG << 3); // writing to config register
            buf[1] = ADT7310_FAULTS_1 | ADT7310_RES_16_BIT; // why not 16 bit?
            gpio_reset(dev->gpio_value_handle);
            spi_transfer(this->spi_handle, buf, 2);
            gpio_set(dev->gpio_value_handle);
            ros::Duration(0.001).sleep();

            int16_t temp;
            buf[0] = (ADT7310_REG_T_CRIT << 3); // writing to critical temperature register
            temp   = T_CRIT_16_BIT;
            buf[1] = (temp >> 8);
            buf[2] = temp & 0xFF;
            gpio_reset(dev->gpio_value_handle);
            spi_transfer(this->spi_handle, buf, 3);
            gpio_set(dev->gpio_value_handle);
            ros::Duration(0.001).sleep();

            buf[0] = (ADT7310_REG_T_HIGH << 3); // writing to critical temperature register
            temp   = T_HIGH_16_BIT;
            buf[1] = (temp >> 8);
            buf[2] = temp & 0xFF;
            gpio_reset(dev->gpio_value_handle);
            spi_transfer(this->spi_handle, buf, 3);
            gpio_set(dev->gpio_value_handle);
            ros::Duration(0.001).sleep();

            dev->is_setup = true;
        }else{
            ROS_INFO("Read %x from ADT7310", buf[1]);
        }
    }else{
        ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
    }



    // ======= IMU ========
    dev = &(spi_devices[IMU_IND]);
    if((dev->gpio_value_handle) > 0){
        spi_set_speed(this->spi_handle, dev->spi_max_speed);
        spi_set_mode(this->spi_handle, dev->spi_mode);
        ros::Duration(0.002).sleep();
        lsm6ds3_soft_reset(this->spi_handle, dev->gpio_value_handle);
        ros::Duration(0.1).sleep();

        buf[0] = WHO_AM_I;
        buf[1] = 0x00;
        LSM6DS3_SET_READ_MODE(buf[0]);
        ROS_INFO("Trying the IMU");
        gpio_reset(dev->gpio_value_handle);
        spi_transfer(this->spi_handle, buf, 2);
        gpio_set(dev->gpio_value_handle);
        ros::Duration(0.002).sleep();

        if(buf[1] == LSM6DS3_WHO_AM_I_ID){
            lsm6ds3_xl_power_on(this->spi_handle, dev->gpio_value_handle, LSM6DS3_ODR_208_HZ | LSM6DS3_FS_XL_2G);
            ros::Duration(0.002).sleep();
            lsm6ds3_g_power_on(this->spi_handle, dev->gpio_value_handle, LSM6DS3_ODR_208_HZ | LSM6DS3_FS_G_250_DPS);
            ros::Duration(0.002).sleep();
            ROS_INFO("IMU setup success!");
            dev->is_setup = true;
        }else{
            ROS_ERROR("IMU setup failed :(, read [%x , %x]...", buf[0], buf[1]);
        }

    }else{
        ROS_ERROR("Failed to get file descriptor for %svalue", dev->gpio_path.c_str());
    }


}

void SensorIf::shutdown(){
    for(int i = 0; i < this->n_sensors; i++){
        SensorInfo * sensor = &(this->sensors[i]);
        if(sensor->gpio_value_fd > 0){
            close(sensor->gpio_value_fd);
        }
        if(sensor->if_type == IF_SPI && sensor->spi_device->gpio_value_handle > 0){
            close(sensor->spi_device->gpio_value_handle);
        }
    }
    close(this->spi_handle);
}

///////////////////////////////////////////////////////////////////////////////
//
//   SENSOR INFO STUFF
//
///////////////////////////////////////////////////////////////////////////////
SensorInfo::SensorInfo(void)
{

}
void SensorInfo::set_update_flag(const ros::TimerEvent& event){
    this->update = true;
    // this->update_mtx->unlock(); // will allow thread to lock mutex and update sensor
}
