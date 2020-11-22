#include "threads/motor_thread.h"


MotorThread::MotorThread(ros::NodeHandle nh) : m_nh(nh), m_loop_rate(1000) {
    m_motors.reserve(16);
    // m_cb_queue = ros::CallbackQueue();

    m_nh.setCallbackQueue(&m_cb_queue);

    // m_motor_data_pub = m_nh.advertise<hwctrl::MotorData>("motor_data", 128);

    m_motor_set_sub   = m_nh.subscribe("motor_setpoints", 128, &MotorThread::set_motor_callback, this);
    m_sensor_data_sub = m_nh.subscribe("sensor_data", 128, &MotorThread::sensor_data_callback, this);
    // m_limit_sw_sub  = m_nh.subscribe("");

    read_from_server();
}

void MotorThread::read_from_server() {
    ROS_INFO("Reading motor configs from param server...");
	const std::string base = param_base + "/motor";
    uint32_t sys_id_idx = 0;
    for(auto name = motor_param_names.begin(); name != motor_param_names.end(); ++name){
        const std::string full_name = base + "/" + *name;
		ROS_INFO("Checking for parameters under %s/...", full_name.c_str());
		const std::string name_param       = full_name + "/name";
		const std::string type_param       = full_name + "/type";
		const std::string can_id_param     = full_name + "/can_id";
		const std::string gear_reduc_param = full_name + "/gear_reduction";
		const std::string max_rpm_param    = full_name + "/max_rpm";
		const std::string max_accel_param  = full_name + "/max_acceleration";
		const std::string period_param     = full_name + "/update_period";

        std::string    name_str;
        MotorType      type;
        ControlType    c_type;
        int            can_id;
        double         gearing;
        double         max_rpm;
        double         max_acc;
        ros::Duration  update_pd;
        ros::Duration  timeout = ros::Duration(2.0);
    
        boost::shared_ptr<Motor> motor;
        int found = 0; 

        if(m_nh.hasParam(name_param)){
			m_nh.getParam(name_param, name_str);
			ROS_INFO(" - Found motor name: %s", name_str.c_str());
			found++;
		}
		if(m_nh.hasParam(type_param)){
			std::string type_str;
			m_nh.getParam(type_param, type_str);
			ROS_INFO(" - Found motor type: %s", type_str.c_str());
			type = get_motor_type(type_str);
			found++;
		}
		if(m_nh.hasParam(can_id_param)){
			m_nh.getParam(can_id_param, can_id);
			ROS_INFO(" - Found CAN id: %d", can_id);
			found++;
		}
		if(m_nh.hasParam(gear_reduc_param)){
			m_nh.getParam(gear_reduc_param, gearing);
			ROS_INFO(" - Found gear reduction: %.2f", gearing);
			found++;
		}
		if(m_nh.hasParam(max_rpm_param)){
			m_nh.getParam(max_rpm_param, max_rpm);
			ROS_INFO(" - Found max RPM: %.2f", max_rpm);
			found++;
		}
		if(m_nh.hasParam(max_accel_param)){
			m_nh.getParam(max_accel_param, max_acc);
			ROS_INFO(" - Found max acceleration: %.2f", max_acc);
			found++;
		}
		if(m_nh.hasParam(period_param)){
			double pd;
			m_nh.getParam(period_param, pd);
			update_pd = ros::Duration(pd);
			ROS_INFO(" - Found motor update period: %.3fs", pd);
		}

        // ros::NodeHandle nh, const std::string& name, uint32_t id,  uint32_t can_id,
        // ros::Duration update_pd = ros::Duration(MOTOR_LOOP_PERIOD), float accel_setpoint = DEFAULT_MAX_ACCEL, float max_accel = DEFAULT_MAX_ACCEL, 
        // float max_rpm = DEFAULT_MAX_RPM, float gear_reduc = 1.0f, ros::Duration timeout = ros::Duration(2.0)
    

        switch(type) {
            case MotorType::Vesc: {
                auto temp = boost::make_shared<VescMotor>(m_nh, name_str, sys_id_idx++, (uint32_t) can_id, update_pd, (float) max_acc, (float) max_acc, (float) max_rpm, (float) gearing, timeout);
                motor = temp;
                break;
            }
            case MotorType::BMC: {
                auto temp = boost::make_shared<BMCMotor> (m_nh, name_str, sys_id_idx++, (uint32_t) can_id, update_pd, (float) max_acc, (float) max_acc, (float) max_rpm, (float) gearing, timeout);
                break;
            }
            case MotorType::Sabertooth:
                // I wont implement this for now
                motor = nullptr;
            default:
                break;
        }

        if(found > 0 && motor != nullptr)
            m_motors.push_back(motor);
    }
}

void MotorThread::set_motor_callback(boost::shared_ptr<hwctrl::MotorCmd> msg) {
    m_motors.at(msg->id) -> set_setpoint(ros::Time::now(), msg -> setpoint, msg -> acceleration);
}

void MotorThread::sensor_data_callback(boost::shared_ptr<hwctrl::SensorData> msg) {
    // determine if this sensor is useful to us;
    if (msg->name.compare("SYSTEM 24V STATE") == 0) {
        if (msg->value == 1.0f) {
            if (!m_sys_power_on) {
                setup_motors(); // sys power has just come on, so initialize motors
            }
            m_sys_power_on = true;
        } else {
            m_sys_power_on = false;
        }
    }
}

void MotorThread::setup_motors() {
    for(auto motor : m_motors) {
        motor->setup();
    }
}

void MotorThread::update_motors() {
    auto time = ros::Time::now();
    for(auto motor : m_motors) {
        if(motor->ready_to_update()) {
            motor->update(time);
        }
    }
}

void MotorThread::shutdown() {
    // stop motors maybe?
}

void MotorThread::sleep() {
    m_loop_rate.sleep();
}

void MotorThread::operator()() {
    ros::AsyncSpinner spinner(1, &m_cb_queue);
    spinner.start();
    // setup_motors();
    while(ros::ok()) {
        update_motors();
        sleep();
    }
    shutdown();
}