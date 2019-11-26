#include <hwctrl.h>

void HwMotorIf::maintain_motors(){
	while(ros::ok()){
		this->maintain_next_motor();
		ros::Duration(MOTOR_LOOP_PERIOD/this->motors.size()).sleep();
	}
}

void maintain_motors_thread(HwMotorIf motor_if){
	while(ros::ok()){
		motor_if.maintain_next_motor();
		ros::Duration(MOTOR_LOOP_PERIOD/motor_if.get_num_motors()).sleep();
	}
}

void HwMotorIf::maintain_next_motor(){
	HwMotor* motor = &(*(this->motor_it));
	ROS_INFO("Maintaining motor %s", motor->name.c_str());
	switch(motor->motor_type){
		case(MOTOR_NONE):break;
		case(MOTOR_VESC):{

			if(motor->online){
				// figure out next velocity setpoint based on acceleration & last sent RPM
				float delta 	=  motor->setpoint - motor->last_setpoint;
				float dt 		= (float)(ros::Time::now().toSec() - motor->update_t.toSec());
				float accel 	= fabs(delta/dt); // proposed acceleration

				// if proposed acceleration is higher than requested accleration, accelerate only at requested acceleration.
				// else the delta we calculated is ok
				if(accel > fabs(motor->accel_setpoint)){
					if(delta > 0){
						delta = fabs(motor->accel_setpoint) * dt; // new delta (is positive)
					}else{
						delta = -1.0 * fabs(motor->accel_setpoint) * dt; // new delta (needs to be negative)
					}
				}
				motor->last_setpoint = motor->last_setpoint + delta; // the new setpoint based on delta
			}
			
			canbus::SetVescCmd cmd;
			cmd.request.can_id 	= motor->device_id; // device_id should be the can_id
			cmd.request.e_rpm 	= motor->rpm_coef * motor->last_setpoint;

			if(this->vesc_client.call(cmd)){
				// presumed success
				if(cmd.response.status == 0){
					motor->update_t = cmd.response.timestamp;
					motor->online = true;
				}
			}else{
				ROS_INFO("No dice");
				motor->online = false;
			}
			break;}
		case(MOTOR_BRUSHED):{

			break;}
	}

	this->motor_it ++;
	if(this->motor_it == this->motors.end()){
		this->motor_it = this->motors.begin();
	}
}


bool HwMotorIf::set_motor_callback(hwctrl::SetMotor::Request& request, hwctrl::SetMotor::Response& response){
	if(request.id >= this->motors.size()){
		return false;
	}

	ROS_INFO("Setting motor %d", request.id);
	HwMotor* motor = this->motors.data() + request.id; // pointer to our motor struct

	motor->setpoint = request.setpoint;

	if(motor->ctrl_type == CTRL_RPM){
		if(fabs(request.acceleration) > motor->max_accel || fabs(request.acceleration) == 0.0){
			motor->accel_setpoint = motor->max_accel;
		}else {
			motor->accel_setpoint = request.acceleration;
		}
		response.actual_accel = motor->accel_setpoint;
	}
	response.status = 0; // need to change later to be meaningful
	
	return true;
}

void HwMotorIf::add_motor(HwMotor mtr){
	this->motors.push_back(mtr);
}

void HwMotorIf::get_motors_from_csv(std::string fname){
	std::vector<std::vector<std::string>> csv = read_csv(fname);
	int line_num = 0;
	int id_ind, name_ind, category_ind, device_type_ind, interface_ind, device_id_ind, aux_1_ind, aux_2_ind, aux_3_ind, aux_4_ind;
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
			if((*line)[category_ind].compare(category_motor) == 0){
				// make a motor struct and populate with data
				HwMotor motor;
				motor.id 			= std::stoi((*line)[id_ind]);
				motor.name 			= (*line)[name_ind];
				motor.device_id 	= std::stoi((*line)[device_id_ind]);
				motor.motor_type 	= get_device_type((*line)[device_type_ind]);
				motor.if_type 		= get_if_type((*line)[interface_ind]);
				motor.rpm_coef 		= std::stof((*line)[aux_2_ind]); // this will be our gear reduction
				motor.max_rpm 		= std::stof((*line)[aux_3_ind]); // our max rpm
				motor.max_accel		= std::stof((*line)[aux_4_ind]); // our max acceleration

				if(motor.motor_type == DEVICE_VESC){
					motor.ctrl_type = CTRL_RPM;
				}else{
					motor.ctrl_type = CTRL_POSITION;
				}
				this->add_motor(motor);
			}

		}
		line_num++;
	}
	this->motor_it = this->motors.begin();
}

int HwMotorIf::get_num_motors(){
	return this->motors.size();
}

InterfaceType get_if_type(std::string type_str){
	if(type_str.compare("can") == 0){
		return IF_CANBUS;
	}else if(type_str.compare("uart") == 0){
		return IF_UART;
	}else{
		return IF_NONE;
	}
}


DeviceType get_device_type(std::string type_str){
	if      (type_str.compare("vesc") 		== 0){
		return DEVICE_VESC;
	}else if(type_str.compare("sabertooth") == 0){
		return DEVICE_SABERTOOTH;
	}else if(type_str.compare("uwb") 		== 0){
		return DEVICE_UWB;
	}else if(type_str.compare("quad_enc") 	== 0){
		return DEVICE_QUAD_ENC;
	}else{
		return DEVICE_NONE;
	}
}

std::string HwMotor::to_string(){
	sprintf(this->scratch_buf, 
		"\n==========\nMotor\t: %s\n\rID\t: %d\n", 
		this->name.c_str(), this->id
		);
	return std::string(this->scratch_buf);
}

std::string HwMotorIf::list_motors(){
	std::string retval;
	for(auto mtr = this->motors.begin(); mtr != this->motors.end(); ++mtr){
		retval.append((*mtr).to_string());
	}
	return retval;
}

// LIMIT SWITCH MONITORING THREAD
void limit_switch_thread(ros::Publisher pub){
	while(ros::ok()){
		ROS_INFO("Checking limit switches");
		ros::Duration(2).sleep();
		// ros::spinOnce();
	}
}

// callback for the VescData topic subscriber
void HwMotorIf::vesc_data_callback(const canbus::VescData& msg){
	FILE * pFile;
	pFile = fopen(vesc_log_path.c_str(), "a");
	// ID, TIME(s), VOLTAGE, CURRENT
	ros::Time raw_time 	= msg.timestamp;
	int id 				= msg.can_id;
	float time_s 		= 1.0*raw_time.sec + 1.0*raw_time.nsec/10e9;
	float volts 		= msg.v_in;
	float current 		= msg.current_in;
	fprintf(pFile, "%d,%f.3,%f.3,%f.3", id, time_s, volts, current);

	// hwctrl::MotorData motor_data;
	// HwMotor * motor;
	
	// motor_data.id = 0;
	// motor_data.timestamp = msg.timestamp;
	// motor_data.setpoint 	= 0.0;
}