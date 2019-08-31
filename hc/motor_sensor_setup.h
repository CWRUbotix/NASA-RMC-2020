#ifndef MOTOR_SENSOR_SETUP_H_
#define MOTOR_SENSOR_SETUP_H_

#include "values_and_types.h"
#include <inttypes.h>

void init_device_comms(void);
void init_motors(void);
void init_sensors(void);


void init_motors(){
	for(int i = 0; i < NUM_MOTORS; i++){
		MotorInfo* motor = &motor_infos[i];
		
		switch(motor->type){
			case MTR_NONE:{break;}
			case MTR_VESC:{
				if(motor->device != NULL && !motor->device->is_setup){
					motor->device->vesc->begin();
					motor->device->vesc->set_rpm(0);
					motor->device->vesc->request_mc_values();
					motor->device->is_setup = true;
				}
				// delay(10);
				break;}
			case MTR_SABERTOOTH:{
				if(motor->device->interface == SPI_BUS && motor->device->spi_cs != 0){
					pinMode(motor->device->spi_cs, OUTPUT);
					digitalWrite(motor->device->spi_cs, HIGH);
				
					// set the motor to not move
					// remember that 0V makes the motor go full speed in reverse
					uint8_t data[3] = {};
					package_DAC_voltage(MOTOR_ANLG_CENTER_V, data); 	// make sure motor is stopped

					SPI.beginTransaction(*(motor->device->spi_settings));
					digitalWrite(motor->device->spi_cs, LOW);
					PAUSE_SHORT;
					SPI.transfer(data, 3);
					digitalWrite(motor->device->spi_cs, HIGH);
					SPI.endTransaction();
				}else if(motor->device->interface == DIGITAL_IO){
					pinMode(motor->device->spi_cs, OUTPUT);
					analogWrite(motor->device->spi_cs, 2048); 	// center value
				}

				motor->setpt 	= motor->sensor->value; 	// make the set-point equal to the current position
				break;}
			case MTR_SABERTOOTH_RC:{
				pinMode(motor->device->spi_cs, OUTPUT);
				motor->setpt = motor->sensor->value;
				digitalWrite(motor->device->spi_cs, HIGH);
				delayMicroseconds(1500);
				digitalWrite(motor->device->spi_cs, LOW);
				break;}
			case MTR_LOOKY:{
				if(motor->device != NULL && !motor->device->is_setup){
					Herkulex.moveOneAngle(motor->device->id, 0.0, 1000, 2);
					motor->device->is_setup = true;
				}
				break;}
		}
	}
}

void init_sensors(){
	// do configurations as necessary
	SensorInfo* sensor = NULL;
	Device* device = NULL;
	for(int i = 0; i < NUM_SENSORS; i++){
		sensor = &sensor_infos[i];
		device = sensor->device;
		debug(String(sensor->name));
		switch(sensor->type){
			case SENS_NONE: break;
			case SENS_DIGITAL_INPUT:{
				if(sensor->pin != 0){
					pinMode(sensor->pin, INPUT);
				}
				break;}
			case SENS_LIMIT: {
				if(sensor->pin != 0){
					pinMode(sensor->pin, INPUT);
				}
				break;}
			case SENS_LOAD_CELL:{
				if(device != NULL && !device->is_setup){
					// configure ADC
					debug("setting up load cell ADC");
					device->adc->setup(CONFIG_FOR_LOAD_CELL);
					device->adc->set_mux_input(diff_1_2);
					device->adc->set_gain(7); 	// 7 is the max gain
					for(int i = 0; i<NUM_PREV_VALUES; i++){
		        		uint16_t temp 		= 0;
		        		int16_t signed_temp = 0;
		        		float f_temp 		= 0.0;
						sensor->value_good = sensor->device->adc->read_channel(sensor->adc_channel_config, &temp);
						if(sensor->value_good){
							memcpy(&signed_temp, &temp, 2);
							f_temp 			= (signed_temp/32767.0)*5.0; 				// voltage
							sensor->value 	= sensor->slope*f_temp + sensor->offset;	// apply linear correction
							sensor->prev_values[i] = sensor->value;
							sensor->baseline += sensor->value/NUM_PREV_VALUES; 			// for generating an average value
						}
						delay(5);
					}
					device->is_setup = true;
				}
				break;}
			case SENS_GYRO: {
				if(device != NULL && !device->is_setup){
					// configure device
					debug("begin IMU");
					SPI.beginTransaction(IMU_SPI_settings);
					int result = device->imu->begin();
					sensor->value = (float)result;
					if(result != 0){
						debug("IMU setup failed "+String(result, DEC));
						device->is_setup = false;
					}else{
						debug("IMU setup success");
						device->is_setup = true;
					}
					SPI.endTransaction();
					// device->is_setup = true;
				}
				break;}
			case SENS_ACCEL: {
				if(device != NULL && !device->is_setup){
					debug("begin IMU");
					SPI.beginTransaction(IMU_SPI_settings);
					int result = device->imu->begin();
					sensor->value = (float)result;
					if(result != 0){
						debug("IMU setup failed "+String(result, DEC));
						sensor->device->is_setup = false;
					}else{
						debug("IMU setup success");
						sensor->device->is_setup = true;
					}
					SPI.endTransaction();
					// device->is_setup = true;
				}
				break;}
			case SENS_ROT_ENC: {
				// configure device as per this sensor
				if(device != NULL && !device->is_setup){
					device->is_setup = true;
				}
				sensor->value = sensor->offset; 	// assumed initial position
				pinMode(ENCODER_A_PIN, INPUT);
				pinMode(ENCODER_B_PIN, INPUT);
				pinMode(ENCODER_INDEX_PIN, INPUT);
				break;}
			case SENS_BLDC_ENC: {
				// let this get configured in init_motors
				break;}
			case SENS_POT_ENC:{
				if(device != NULL && !device->is_setup){
					// configure ADC 
					debug("setting up pot. ADC");
					device->adc->setup(CONFIG_FOR_POT);
					device->adc->set_mux_input(sensor->adc_channel_config);
					device->adc->set_gain(0);
					device->is_setup = true;
				}
				break;}
		}
	}

}

// configure more critical pins such as SPI chip-select pins
//  basic low-level IO, prevent bus collisions, etc.
void init_device_comms(){
	for(int i = 0; i<NUM_DEVICES; i++){
		Device* device = &device_infos[i];

		switch(device->interface){
			case NONE: {break;}
			case DIGITAL_IO:{ break;}
			case VESC_UART: { 
				device->vesc->begin();
				break;}
			case LOOKY_UART: {
				Herkulex.begin(device->serial, 115200);
				Herkulex.initialize();
				break;}
			case SPI_BUS: {
				if(device->spi_cs != 0){
					pinMode(device->spi_cs, OUTPUT);
					digitalWrite(device->spi_cs, HIGH);
				}
				break;}
		}
		
		
	}
}

#endif
