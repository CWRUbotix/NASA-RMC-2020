#include <HardwareControlInterface.h>

map<uint8_t, float> motorValues;
ros::Publisher sensorPublisher;
ros::Subscriber motorSubscriber;
ros::Subscriber driveSubscriber;

serial::Serial hcSerial;
unsigned long baud = 115200; 
string hcDescription = "Teensyduino USB Serial 4822650"; //description of the HCb

uint8_t setOutputsByte = 0x51;
uint8_t readValuesByte = 0x52;
uint8_t testByte = 0x53;
uint8_t syncTimeByte = 0x54;

bool addMotorValue(int ID, float value){
    //we don't care if this overwrites an existing pair
    uint8_t smallerID = ID;
    motorValues.erase(smallerID);
    motorValues.insert(std::pair<uint8_t,float>(smallerID,value));
    return true;
}

void addMotorCallback(const hci::motorCommand& msg){
    addMotorValue(msg.motorID, msg.value);
}

void driveCommandCallback(const hci::driveCommand& msg){
    if(msg.direction == 0){
    	//forward
    	addMotorValue(0, msg.value);
    	addMotorValue(1, msg.value);
    }
    else if(msg.direction == 1){
    	//backward
    	addMotorValue(0, -msg.value);
    	addMotorValue(1, -msg.value);
    }
    else if(msg.direction == 2){
    	//rightward
    	addMotorValue(0, msg.value);
    	addMotorValue(1, -msg.value);
    }
    else if(msg.direction == 3){
    	//leftward
    	addMotorValue(0, -msg.value);
    	addMotorValue(1, msg.value);
    }
}

void enumeratePorts(void){
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        ROS_INFO( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
    }
}

string findHardwareControllerPort(void){
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        ROS_INFO( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
        if(string(device.description.c_str()).compare(hcDescription) == 0){
            return device.port.c_str();
        }
    }

    ROS_WARN("Didn't find the Hardware Controller board!");
    return "0";
}


vector<uint8_t> generateMotorCommandMessage(void){
    vector<uint8_t> commandMessage;
    commandMessage.push_back(setOutputsByte);


    if(abs(motorValues[0]) != abs(motorValues[1])){
    	ROS_WARN("MISMATCH");
    }

    uint16_t checksum = 0;

    if(abs(motorValues[0]) != abs(motorValues[1])){
    	ROS_WARN("Sending motor command to one drive motor but not the other!");
    }

    for(map<uint8_t,float>::iterator it = motorValues.begin(); it != motorValues.end(); it++) {

        commandMessage.push_back(it->first);

        char value[sizeof(float)];
        float f = it->second;
        memcpy(value, &f, sizeof f);    //transfer the float value into a char array

        commandMessage.push_back(value[0]);
        commandMessage.push_back(value[1]);
        commandMessage.push_back(value[2]);
        commandMessage.push_back(value[3]);
        //ROS_INFO("%u, %u, %u, %u", value[0],value[1],value[2],value[3]);

        checksum += (uint16_t)(0x00ff & it->first);
        checksum += (uint16_t)(0x00ff & value[0]);
        checksum += (uint16_t)(0x00ff & value[1]);
        checksum += (uint16_t)(0x00ff & value[2]);
        checksum += (uint16_t)(0x00ff & value[3]);

        ROS_INFO("MotorCommand: motor %u to %f", it->first, f);
    }   

    vector<uint8_t>::iterator it = commandMessage.begin();
    it++; //now points to position 1
    uint16_t length = commandMessage.size() - 1;
    it = commandMessage.insert(it, checksum);
    it = commandMessage.insert(it, checksum >> 8);
    it = commandMessage.insert(it, length);
    it = commandMessage.insert(it, length >> 8);
    
    return commandMessage;
}

vector<uint8_t> generateSensorRequestMessage(void){
    vector<uint8_t> sensorMessage;
    sensorMessage.push_back(readValuesByte);
    uint16_t checksum = 0;
    for(uint8_t sensorID = 0 ; sensorID < 33; sensorID++) {

        sensorMessage.push_back(sensorID);

        checksum += (uint16_t)(0x00ff & sensorID);
    }   

    vector<uint8_t>::iterator it = sensorMessage.begin();
    it++; //now points to position 1
    uint16_t length = sensorMessage.size() - 1;
    it = sensorMessage.insert(it, checksum);
    it = sensorMessage.insert(it, checksum >> 8);
    it = sensorMessage.insert(it, length);
    it = sensorMessage.insert(it, length >> 8);
    
    return sensorMessage;

}

void parseSensorResponseMessage(vector<uint8_t> sensorResponse){

	//do checks to make sure everything is correct format
	//but not now 
	if(sensorResponse.size() < 6){
		//test sensorResponse:
		/*sensorResponse = vector<uint8_t>();
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)2);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)32);
		sensorResponse.push_back((uint8_t)65);
		sensorResponse.push_back((uint8_t)1);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);
		sensorResponse.push_back((uint8_t)0);*/
		return;
	}

	vector<uint8_t>::iterator it = sensorResponse.begin() + 5;
	for(it; it != sensorResponse.end(); it++) {

        hci::sensorValue sensorMessage;
        sensorMessage.sensorID = *it;
        it++; //points to the first byte of the data
 
        uint8_t data[4];

 		data[0] = *it++;
 		data[1] = *it++;
 		data[2] = *it++;
 		data[3] = *it++;
 		//ROS_INFO("%d, %u, %u, %u, %u", sensorMessage.sensorID, data[0],data[1],data[2],data[3]);
 		float val = *reinterpret_cast<float*>(&data[0]);
 
 		//ROS_INFO("sensorValue: %f", val);
 		sensorMessage.value = val;

 		it += 3;

 		sensorPublisher.publish(sensorMessage);
    }   
}




int main(int argc, char** argv) {

    ros::init(argc, argv, "hci");
    ros::NodeHandle n; 
    sensorPublisher = n.advertise<hci::sensorValue>("sensorValue", 32);
    motorSubscriber = n.subscribe("motorCommand",100,addMotorCallback); 
    driveSubscriber = n.subscribe("driveCommand",10,driveCommandCallback);

    string port = "0";
    while(port == "0"){
        port = findHardwareControllerPort();
        ros::Duration(1).sleep();
    } 
    ROS_INFO("%s\n ", port.c_str());

    ros::Rate loop_rate(200);
    
    while(ros::ok()){
        if(!hcSerial.isOpen()){
            while(!hcSerial.isOpen()){
                try
                {
                    hcSerial.setPort(port);
                    hcSerial.setBaudrate(baud);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    hcSerial.setTimeout(to);
                    hcSerial.open();
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR("Unable to open port ");
                    ROS_ERROR("%s", e.what());
                }
                ros::Duration(0.25).sleep();
                ros::spinOnce();
            }
            ROS_INFO("Serial port opened");
        }
        ROS_INFO("NEW MOTOR MESSAGE");
        vector<uint8_t> motorCommandMessage = generateMotorCommandMessage();
        
        //for (std::vector<uint8_t>::const_iterator i = motorCommandMessage.begin(); i != motorCommandMessage.end(); ++i){
        //    ROS_INFO("%u", *i);
        //}

        //ROS_INFO("LENGTH OF MOTOR COMMAND: %u", (motorCommandMessage[1] << 8) + motorCommandMessage[2]);


        hcSerial.write(motorCommandMessage);
        vector<uint8_t> motorCommandResponse;
        hcSerial.read(motorCommandResponse, motorCommandMessage.size());
        //assume for now that this works...


        //now move on to asking about the sensors

        vector<uint8_t> sensorRequestMessage = generateSensorRequestMessage();
        //ROS_INFO("NEW SENSOR MESSAGE");
        //for (std::vector<uint8_t>::const_iterator i = sensorRequestMessage.begin(); i != sensorRequestMessage.end(); ++i){
        //    ROS_INFO("%u", *i);
        //}

        hcSerial.write(sensorRequestMessage);
        vector<uint8_t> sensorRequestResponse;
        //jank cuz yolo
        //ROS_INFO("sensorRequest Size: %lu", sensorRequestMessage.size());
        uint16_t sensorRequestResponseLength = (((sensorRequestMessage.size() - 5) * 9) + 5);
        hcSerial.read(sensorRequestResponse, sensorRequestResponseLength);

        //ROS_INFO("SENSOR RESPONSE LENGTH: %lu" , sensorRequestResponse.size());

        parseSensorResponseMessage(sensorRequestResponse);
        
        //ros::Duration(.005).sleep();
        loop_rate.sleep();
        //ROS_INFO("test");
        ros::spinOnce();

    }
    
    ros::spin();

    return 0;
}
