#include <VESC.h>

#define MAX_RPM 	10000
#define MIN_RPM 	-10000

VESC vesc(&Serial1);

void setup(){
	vesc.begin();
	Serial.begin(115200);
}

void loop(){
	int i = 0;
	for(i = 0; i <= MAX_RPM; i+=250){
		vesc.request_mc_values();
		delay(250);
		vesc.update_mc_values();
		Serial.print("RPM \t: "); Serial.println(vesc.get_rpm());
		vesc.set_rpm(i);
		delay(1);
	}
	for(int n = 0; n < 50; n++){
		vesc.set_rpm(i);
		delay(1);
		vesc.request_mc_values();
		delay(10);
		vesc.update_mc_values();
		Serial.println(vesc.get_rpm());
		delay(50);
	}
	for(i = MAX_RPM; i >= MIN_RPM; i-=250){
		vesc.request_mc_values();
		delay(250);
		vesc.update_mc_values();
		Serial.print("RPM \t: "); Serial.println(vesc.get_rpm());
		vesc.set_rpm(i);
		delay(1);
		
	}

	for(int n = 0; n < 50; n++){
		vesc.set_rpm(i);
		delay(1);
		vesc.request_mc_values();
		delay(10);
		vesc.update_mc_values();
		Serial.println(vesc.get_rpm());
		delay(50);
	}

	for(i = MIN_RPM; i <= 0; i+=250){
		vesc.request_mc_values();
		delay(250);
		vesc.update_mc_values();
		Serial.print("RPM \t: "); Serial.println(vesc.get_rpm());
		vesc.set_rpm(i);
		delay(1);
		
	}
}