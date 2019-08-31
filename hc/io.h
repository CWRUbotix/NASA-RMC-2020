#ifndef IO_H_
#define IO_H_

#include "values_and_types.h"
#include <math.h>
#include <inttypes.h>

void update_sensors(void);
void maintain_motors(void);
void package_DAC_voltage(float v, uint8_t* data);
void debug(String s);
float get_rot_encoder_value(SensorInfo* encoder);
void Encoder_index_ISR(void);
void Encoder_A_ISR(void);

void update_sensors(){
	for(int i = 0; i < NUM_SENSORS; i++){
		SensorInfo* sensor 	= &sensor_infos[i];
		Device* device 		= sensor->device;
		switch(sensor->type){
			case SENS_NONE: break;
			case SENS_DIGITAL_INPUT:{
				sensor->value = (digitalRead(sensor->pin) == HIGH ? 1.0 : 0.0);
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LIMIT:{
				sensor->value = (digitalRead(sensor->pin) == HIGH ? 1.0 : 0.0);
				sensor->t_stamp = TIME_STAMP;
				break;}
			case SENS_LOAD_CELL:{
				if(sensor->device != NULL && sensor->device->is_setup){
	        		uint16_t temp 		= 0;
	        		int16_t signed_temp = 0;
	        		float ratio 		= 0.0;
	        		float f_temp 		= 0.0;
	        		sensor->value_good = sensor->device->adc->read_channel(sensor->adc_channel_config, &temp);
					if(sensor->value_good){
						memcpy(&signed_temp, &temp, 2);
						ratio 			= signed_temp/32767.0; 	// ratio of full scale
						f_temp 			= ratio*5.0; 			// voltage
						sensor->value 	= sensor->slope*f_temp + sensor->offset;	// apply linear correction
						sensor->t_stamp = TIME_STAMP;
						sensor->prev_values[sensor->val_ind] = sensor->value;
						sensor->val_ind++;
						if(sensor->val_ind >= NUM_PREV_VALUES){
							sensor->val_ind = 0;
							// Use this opportunity to re-calculate our baseline
							float new_baseline 		= 0.5*faverage(sensor->prev_values, NUM_PREV_VALUES) + 0.3*sensor->baseline + 0.2*sensor->last_baseline;
							sensor->last_baseline 	= sensor->baseline;
							sensor->baseline 		= new_baseline;
						}
						sensor->peak_value = max_in_array(sensor->prev_values, NUM_PREV_VALUES);
						if(i == EXC_LOAD_CELL){
							debug(String(sensor->name)+":"+
								String(f_temp, 3)+":"+
								String(sensor->peak_value, 3)+":"+
								String(ratio, 2));
						}
					}else{
						debug(String(sensor->name)+":"+"ERROR");
					}
				}
				break;}
			case SENS_GYRO:{
				if(sensor->device != NULL && sensor->device->is_setup){

					SPI.beginTransaction(IMU_SPI_settings);
					switch(sensor->imu_axis){
						case 'X': {sensor->value = device->imu->readFloatGyroX(); break;}
						case 'Y': {sensor->value = device->imu->readFloatGyroY(); break;}
						case 'Z': {sensor->value = device->imu->readFloatGyroZ(); break;}
					}
					SPI.endTransaction();
					sensor->t_stamp = TIME_STAMP;
					sensor->value 	= sensor->slope * sensor->value + sensor->offset;
					// sensor->prev_values[sensor->val_ind] = sensor->value;
					// sensor->val_ind++;
					// if(sensor->val_ind >= NUM_PREV_VALUES){
					// 	sensor->val_ind = 0;}
				}
				break;}
			case SENS_ACCEL:{
				if(sensor->device != NULL && sensor->device->is_setup){
					SPI.beginTransaction(IMU_SPI_settings);
					switch(sensor->imu_axis){
						case 'X': {sensor->value = device->imu->readFloatAccelX(); break;}
						case 'Y': {sensor->value = device->imu->readFloatAccelY(); break;}
						case 'Z': {sensor->value = device->imu->readFloatAccelZ(); break;}
					}
					SPI.endTransaction();
					sensor->t_stamp = TIME_STAMP;
					sensor->value = sensor->slope * sensor->value + sensor->offset; 	// convert g's to m/s^2
					// sensor->prev_values[sensor->val_ind] = sensor->value;
					// sensor->val_ind++;
					// if(sensor->val_ind >= NUM_PREV_VALUES){
					// 	sensor->val_ind = 0;}
				}
				break;}
			case SENS_ROT_ENC:{
				float delta 		= (encoder_A_pulses/ROT_ENC_PPR) * EXC_MM_PER_ROT;  // how much have we moved in this loop
				encoder_A_pulses 	= 0; // reset this
				long new_time 		= TIME_STAMP;
				float speed 		= delta / ((new_time - sensor->t_stamp)/1000000.0); // in mm/s
				sensor->value 		+= delta; 					// change value by this much
				sensor->t_stamp 	= new_time;
				debug("EXC TRANS SPD:"+String(speed,3));
				break;}
			case SENS_BLDC_ENC:{
				if(sensor->device != NULL && sensor->device->is_setup){
					// sensor->device->vesc->request_mc_values(); 				// should have been called during maintain motors
					sensor->device->vesc->update_mc_values(); 					// read the available packet
					sensor->value = 1.0*sensor->device->vesc->get_rpm();		// get the motor RPM
					sensor->value 	= sensor->value / sensor->motor->rpm_factor; 	// convert to output RPM
					sensor->t_stamp = TIME_STAMP;
					float current 	= sensor->device->vesc->get_current_in();
					float temp_1 	= sensor->device->vesc->get_temperature_1();
					// float temp_2 	= sensor->device->vesc->get_temperature_2();

					debug(String(sensor->motor->name)+" : "+String(current,3)+" : "+String(temp_1,3)); 
				}
				break;}
			case SENS_POT_ENC:{
				if(sensor->device != NULL && sensor->device->is_setup){
					uint16_t raw 		= 0;
	        		sensor->value_good 	= sensor->device->adc->read_channel(sensor->adc_channel_config, &raw);
					if(sensor->value_good){
						sensor->t_stamp = TIME_STAMP;
						float temp 		= fmap(1.0*raw, sensor->min, sensor->max, 0.0, 32767.0); // map analog value between 0 and FS
						temp 			= temp/32767.0;
						// convert fraction of travel to angle
						temp 			= -((float)pow(4.0*temp + 9.4132, 2) - 170.7218)/102.7197;
						temp 			= 1.6917 - (float)acos( temp ); // angle in radians
						sensor->value 	= temp * (180/3.14159); 		// convert to degrees
						// debug("Pot. raw val:\t"+String(raw, DEC));
					}else{
						debug("Pot. read ERROR");
					}
				}
				break;}
			case SENS_LOOKY_ENC:{
				//to get status:
				//XYZrobotServoStatus status = servo.readStatus();
				// sensor->device->servo_status = sensor->device->servo->readStatus();
				sensor->value 	= Herkulex.getAngle(sensor->device->id);
				sensor->t_stamp = TIME_STAMP;
				break;}
		}
	}
}

void maintain_motors(){
	for(int i = 0; i < NUM_MOTORS; i++){
		MotorInfo* motor = &motor_infos[i];

		switch(motor->type){
			case MTR_NONE: break;
			case MTR_VESC:{
				float dt 		= (TIME_STAMP - motor->t_stamp) / 1000000.0; 	// delta-T in sec
				float proposed_delta = (motor->setpt - motor->last_rpm)/dt; // calculate proposed delta-RPM
				int sign 		= (int)(proposed_delta/fabs(proposed_delta));
				if(fabs(proposed_delta) > motor->max_delta){ 					// check if the proposed delta is allowed
					motor->last_rpm += (sign * motor->max_delta * dt);
				}else{ 														// TOO MUCH DELTA
					motor->last_rpm = motor->setpt; // increment by max, and consider the sign
				}
				int rpm = (int)(motor->last_rpm * motor->rpm_factor); 		// convert to "eRPM" and cast to int

				int dir 		= get_sign(rpm);
				if(motor->limit_1 != NULL && motor->limit_1->value > 0.0){
					if(dir != motor->limit_1->allowed_dir){
						rpm = 0;
					}
				}else if(motor->limit_2 != NULL && motor->limit_2->value > 0.0){
					if(dir != motor->limit_2->allowed_dir){
						rpm = 0;
					}
				}

				// debug("Updating VESC: "+String(rpm, DEC));
				motor->device->vesc->set_rpm(rpm); 							// actually send the rpm
				motor->device->vesc->request_mc_values(); 					// resonse packet should be ready when we call update sensors
				motor->t_stamp = TIME_STAMP;

				break;}
			case MTR_SABERTOOTH:{
				
				if(i == EXC_ROT_PORT){
					MotorInfo* stbd 	= &motor_infos[EXC_ROT_STBD];
					stbd->setpt 		= motor->setpt;
					float diff 			= motor->sensor->value - stbd->sensor->value;
					
					if(diff > LIN_ACT_ERR_MARGIN){
						stbd->kp = LIN_ACT_KP + LIN_ACT_KP_INC;
						motor->kp = LIN_ACT_KP - LIN_ACT_KP_INC;
					}else if(diff < -LIN_ACT_ERR_MARGIN){
						stbd->kp = LIN_ACT_KP - LIN_ACT_KP_INC;
						motor->kp = LIN_ACT_KP + LIN_ACT_KP_INC;
					}else{
						stbd->kp = LIN_ACT_KP;
						motor->kp = LIN_ACT_KP;
					}
				}else if(i == EXC_ROT_STBD){
					// motor_infos[EXC_ROT_PORT].setpt = motor->setpt;
				}
				
				float err 		= motor->setpt - motor->sensor->value; 			// error in degrees
				// float dt 		= (TIME_STAMP - motor->t_stamp) / 1000000.0; // how much time since last update in SECONDS
				// motor->integ 	+= err*dt;
				// if(motor->integ != motor->integ){
				// 	motor->integ = 0.0;
				// }
				// motor->integ 	= fconstrain(motor->integ, -motor->max_integ, motor->max_integ);
				// debug(String(motor->integ));
				float voltage 	= 0.0;
				if(fabs(err) < motor->err_margin){ 	// if err_margin is set to 0.0, then we'll always do PI(D)
					voltage = 0.0; // stop the motor
				}else{
					//PID controls:	
					voltage = motor->kp*err; // + motor->ki*motor->integ; // should range from -2.5 to +2.5 (or min_power to max_power)
				}
				voltage = fconstrain(voltage, motor->min_power, motor->max_power);

				// FOR DEBUGGING PURPOSES
				// voltage = motor->setpt;
				// DELETE LATER TO ENABLE PI(D) CONTROL

				int dir 		= get_sign(voltage);
				if(motor->limit_1->value > 0.0){
					if(dir != motor->limit_1->allowed_dir){
						voltage = 0.0;
					}
				}else if(motor->limit_2->value > 0.0){
					if(dir != motor->limit_2->allowed_dir){
						voltage = 0.0;
					}
				}
				dir = get_sign(voltage); 	// in case something changed

				// ACCOUNT FOR DEADBAND
				if(dir > 0){
					voltage = fmap(voltage, 0, motor->max_power, motor->deadband, motor->max_power);
				}else if(dir < 0){
					voltage = fmap(voltage, motor->min_power, 0, motor->min_power, -motor->deadband);
				}else{}


				// debug(String(i, DEC) + " Sabertooth,\tError: " + String(err, 3) + " deg,\tV: "+ String(voltage, 3));
				motor->power = voltage; 	// for other things to reference it
				if(motor->device->interface == DIGITAL_IO){
					int pwm = (int)(voltage + 4095.0/2 + 0.5);
					// debug("PIN: " + String(motor->device->spi_cs));
					// debug("pwm: " + String(pwm, DEC));
					analogWrite(motor->device->spi_cs, pwm);
				}else{
					// voltage = voltage + MOTOR_ANLG_CENTER_V; 	// shift bipolar voltage into the correct range
					// uint8_t data[3] = {};
					// package_DAC_voltage(voltage, data); 	// for debugging

					// SPI.beginTransaction(*(motor->device->spi_settings));
					// digitalWrite(motor->device->spi_cs, LOW);
					// PAUSE_SHORT;
					// SPI.transfer(data, 3);
					// digitalWrite(motor->device->spi_cs, HIGH);
					// SPI.endTransaction();
				}
				motor->t_stamp = TIME_STAMP; 			// record the update time

				break;}
			case MTR_SABERTOOTH_RC:{
				float power 	= 0.0;//motor->setpt; // FOR DEBUGGING
				float dt 		= (TIME_STAMP - motor->t_stamp) / 1000000.0;
				float err 		= motor->setpt - motor->sensor->value; 	// error in mm
				
				
				if(fabs(err) > motor->err_margin){
					motor->integ 	+= err*dt; 			// only integrate when outside the error margin
					if(motor->integ != motor->integ){
						motor->integ = 0.0;
					}
					motor->integ = fconstrain(motor->integ, -motor->max_integ, motor->max_integ);

					power = motor->kp*err + motor->ki*motor->integ;
				}else{
					power = 0.0; 						// STOP
				}

				// CHANGE MAX-POWER BASED ON SENSOR READINGS
				SensorInfo* sensor = &(sensor_infos[EXC_LOAD_CELL]);
				if(sensor->peak_value > sensor->baseline + 0.1){ 	// if too much load is detected
					motor->max_power -= 1.5; 						// begin to decrement max_power
					if(motor->max_power < 25.0){
						motor->max_power = 25; 						// make sure it doesn't get too small
					}
				}else{ 												// if we're not under heavy load
					motor->max_power += 0.1; 						// increment motor->max_power until we're back to normal
					if(motor->max_power > EXC_NORMAL_POWER){
						motor->max_power = EXC_NORMAL_POWER;
					}
				}

				// ACCOUNT FOR DEADBAND
				int dir = get_sign(power); 				
				if(dir > 0){
					power = fmap(power, 0, motor->max_power, motor->deadband, motor->max_power);
				}else if(dir < 0){
					power = fmap(power, motor->min_power, 0, motor->min_power, -motor->deadband);
				}else{}

				// FINAL CONSTRAIN in-case something happened
				power = fconstrain(power, motor->min_power, motor->max_power);

				dir 		= get_sign(power);
				if(motor->limit_1->value > 0.0){
					if(dir != motor->limit_1->allowed_dir){
						power = 0.0; 								// STOP
					}
					motor->sensor->value = motor->limit_1->offset;  // what is the position at this limit
				}else if(motor->limit_2->value > 0.0){
					if(dir != motor->limit_2->allowed_dir){
						power = 0.0; 								// STOP
					}
					motor->sensor->value = motor->limit_2->offset;  // what is the position at this limit
				}

				motor->power = power; 
				debug(String(motor->name)+":"+String(power,1));
				int pulse_width = power + 1500.0 + 0.5;
				digitalWrite(motor->device->spi_cs, HIGH);
				delayMicroseconds(pulse_width);
				digitalWrite(motor->device->spi_cs, LOW);
				motor->t_stamp = TIME_STAMP;
				break;}
			case MTR_LOOKY:{
				if(motor->setpt != motor->last_setpt){
					// Herkulex.moveOneAngle(servoID, angle, time_ms, iLed??)
					int span = (int)(1000.0*fabs(motor->sensor->value - motor->setpt)/motor->max_power); 	// 1000*degs / (degs-per-sec) = ms
					Herkulex.moveOneAngle(motor->device->id, motor->setpt, span, 2);
				}
				// debug("Updating Looky");
				break;}
		}
	}
}

void debug(String s){
	DEBUG.println(s);

	DEBUG.flush();
	// delay((int)(s.length()*0.1));
	// s.concat('\n');
	// int len = s.length();
	// unsigned char data[len] = {};
	// s.getBytes(data, len);

	// SPI.beginTransaction(Debug_SPI_settings);
	// digitalWrite(DEBUG_SELECT_PIN, LOW);
	// PAUSE_SHORT;
	// SPI.transfer(data, len);
	// digitalWrite(DEBUG_SELECT_PIN, HIGH);
	// SPI.endTransaction();
}


void package_DAC_voltage(float v, uint8_t* data){
	// set the mode to normal
	data[0] &= 0x00;

	uint16_t temp = (uint16_t)(((v/5.0)*65535)+0.5);
	// debug("DAC voltage: " + String(temp));
	data[1] = (temp >> 8) & 0xFF;
	data[2] = (temp) & 0xFF;
}

void Encoder_index_ISR(){
	// if A is HIGH (and B is LOW), dir is CCW
	// else, dir is CW
	uint8_t dir = digitalRead(ENCODER_A_PIN);
	if(dir == HIGH){
		encoder_rots += 1; 	// increment number of rotations
	}else{
		encoder_rots += -1; // decrement number of rotations
	}
	// sensor_infos[EXC_TRANS_ENC].value = sensor_infos[EXC_TRANS_ENC].offset + sensor_infos[EXC_TRANS_ENC].rots * EXC_MM_PER_ROT;
	// sensor_infos[EXC_TRANS_ENC].t_stamp = TIME_STAMP;
}

// detect rising edge of the A Quadrature signal
void Encoder_A_ISR(){
	// if A is HIGH (and B is LOW), dir is CCW (+)
	// else, dir is CW (-)
	int b_state = digitalRead(ENCODER_B_PIN);
	// float delta = (1.0/ROT_ENC_PPR) * EXC_MM_PER_ROT; 			// how much change in distance due to this pulse
	if(b_state == LOW){
		encoder_A_pulses += 1;
		// sensor_infos[EXC_TRANS_ENC].value += delta; // CCW rotation
	}else{
		encoder_A_pulses += -1;
		// sensor_infos[EXC_TRANS_ENC].value -= delta; // CW rotation
	}
	// sensor_infos[EXC_TRANS_ENC].t_stamp = TIME_STAMP;
}

// @return fraction of rotation
float get_rot_encoder_value(SensorInfo* encoder){
	int n = 0;
	uint8_t rpy = 0;
	uint16_t value = 0;
	SPI.beginTransaction(*encoder->device->spi_settings);
	digitalWrite(encoder->device->spi_cs, LOW);
	PAUSE_SHORT;

	rpy = SPI.transfer(ROT_ENC_RD_POS); 	// send the command

	while(rpy != ROT_ENC_RD_POS && n < 10){ // wait for a response or timeout
		rpy = SPI.transfer(0x00);
		n++;
	}
	if(rpy == ROT_ENC_RD_POS){
		value = ((SPI.transfer(0x00) & 0x0F) << 8 ); 	// get the high-order byte
		value |= (SPI.transfer(0x00) & 0xFF); 			// add the low-order byte
	}

	digitalWrite(encoder->device->spi_cs, HIGH);
	SPI.endTransaction();

	float retval = (value/4095.0); 	// fraction of a full rotation
	return retval;
}

#endif
