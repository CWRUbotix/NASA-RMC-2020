////////////////////////////////////////////////////////////////////////////////
// 	MAIN
////////////////////////////////////////////////////////////////////////////////

#include "VESC/VESC.h"
#include "SparkFunLSM6DS3.h"
#include "ADS1120.h"
#include "Herkulex.h"
#include "values_and_types.h"
#include "data_setup.h"
#include "io.h"
#include "motor_sensor_setup.h"
#include "client_comms.h"

void setup(){
	DEBUG.begin(115200);
	SPI.begin();
	analogWriteResolution(12);
	analogWriteFrequency(ENCODER_CS_PIN, 14648);
	analogWriteFrequency(DAC_2_CS_PIN, 14648);
	analogWrite(ENCODER_CS_PIN, 2048);
	analogWrite(DAC_2_CS_PIN, 2048);
	
	debug("CWRUbotix Hardware Controller");
	
	debug("setup devices");
	setup_devices();

	debug("setup sensors");
	setup_sensors(); 		// sets up sensor data structures
	
	debug("setup motors");
	setup_motors(); 		// sets up motor data structures
	
	debug("init device comms");
	init_device_comms(); 	// configures pins according to motor & sensor data
	
	debug("init sensors");
	init_sensors(); 		// initializes the sensors, this happens as long as the re

	attachInterrupt(ENCODER_INDEX_PIN, Encoder_index_ISR, RISING);
	attachInterrupt(ENCODER_A_PIN, Encoder_A_ISR, RISING);
	
	// Initialize the DACs so the motors don't start to move when the E-Stop energizes
	// uint8_t data[3] = {};
	// SPI.beginTransaction(DAC_SPI_settings);
	// package_DAC_voltage(MOTOR_ANLG_CENTER_V, data); 	
	// digitalWrite(DAC_0_CS_PIN, LOW);
	// PAUSE_SHORT;
	// SPI.transfer(data, 3);
	// digitalWrite(DAC_0_CS_PIN, HIGH);
	// package_DAC_voltage(MOTOR_ANLG_CENTER_V, data); 
	// digitalWrite(DAC_1_CS_PIN, LOW);
	// PAUSE_SHORT;
	// SPI.transfer(data, 3);
	// digitalWrite(DAC_1_CS_PIN, HIGH);
	// package_DAC_voltage(MOTOR_ANLG_CENTER_V, data); 
	// digitalWrite(DAC_2_CS_PIN, LOW);
	// PAUSE_SHORT;
	// SPI.transfer(data, 3);
	// digitalWrite(DAC_2_CS_PIN, HIGH);
	// SPI.endTransaction();
	
	debug("check estop pin");
	if(digitalRead(sensor_infos[ESTOP_SENSE_INDEX].pin) == HIGH){
		// delay(500); 		// wait for motor controllers to boot up
		debug("init motors");
		init_motors();
	}else{
		debug("wait to init motors");
	}
	// wait on initializing motors
	// for(int i = 0; i<254; i++){
	// 	if( i != 0x01){
	// 		Herkulex.set_ID(i, 0x02);
	// 	}
	// }
	
	debug("setup complete");
}

void loop(){
	int start_time = micros();
	estop_state = digitalRead(sensor_infos[ESTOP_SENSE_INDEX].pin);
	sensor_infos[ESTOP_SENSE_INDEX].t_stamp = TIME_STAMP;

	if(estop_state==HIGH && estop_state!=estop_state_last){
		// delay(500); 			// wait for motor controllers to boot up
		debug("init motors");
		init_motors();
	} estop_state_last = estop_state;

	FAULT_T read_fault = NO_FAULT;
	if(CLIENT.available()){
		// do the client receive stuff
		debug("reading from client");
		read_fault = read_from_client();

		// answer the client
		debug("answering client");
		reply_to_client(read_fault);
	}

	update_sensors();

	if(estop_state == HIGH){
		// do motor things
		debug("maintaining motors");
		maintain_motors();
	}
	float loop_t = (micros() - start_time)/1000;
	if(loop_t > 0.0009){
		// String time_str = String("Loop-time in microsec: ");
		// time_str.concat(String(loop_t, 3));
		// debug(time_str);
	}
	debug("TIME : "+String(TIME_STAMP));


	// debug("\nIMU 0 Bus Errors Reported:");
	// debug(" All '1's = " + String(imu0.allOnesCounter));
	// debug(" Non-success = " + String(imu0.nonSuccessCounter));
	
}