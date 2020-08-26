#include <hwctrl.h>

////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
InterfaceType get_if_type(std::string type_str){
	if(type_str.compare("can") == 0){
		return IF_CANBUS;
	}else if(type_str.compare("uart") == 0){
		return IF_UART;
	}else if(type_str.compare("spi") == 0){
		return IF_SPI;
	}else if(type_str.compare("gpio") == 0){
		return IF_GPIO;
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
	}else if(type_str.compare("limit") 	== 0){
		return DEVICE_LIMIT_SW;
	}else if(type_str.compare("imu") 	== 0){
		return DEVICE_LSM6DS3;
	}else if(type_str.compare("temperature") 	== 0){
		return DEVICE_ADT7310;
	}else if(type_str.compare("adc") 	== 0){
		return DEVICE_ADS1120;
	}else if(type_str.compare("pot") 	== 0){
		return DEVICE_POT;
	}else if(type_str.compare("load_cell") 	== 0){
		return DEVICE_LOAD_CELL;
	}else if(type_str.compare("power_sense") 	== 0){
		return DEVICE_POWER_SENSE;
	}else{
		return DEVICE_NONE;
	}
}

/**
 * basic running average
 */
float get_running_mean(float* data, int size){
	float weight = 1.0/size;
	float retval = 0.0;
	for(int i = 0; i < size; i++){
		retval += data[i] * weight;
	}
	return retval;
}

/**
 * basic check to see if a file exists
 */
bool file_exists(const char * path){
	FILE * file;
	if(file = fopen(path, "r")){
		fclose(file);
		return true;
	}else{
		return false;
	}
}

/**
 * returns file size. If the file doesn't exist, returns -1
 */
unsigned long long int file_size(const char * path){
	if(!file_exists(path)){
		return -1;
	}
	FILE* file = fopen(path, "rb");
	fseek(file, 0, SEEK_END);
	unsigned long long int size = ftell(file);
	fclose(file);
	return size;
}

/**
 * write a std::vector of Calibration structs to the specified file path
 */
void write_cal(std::string path, std::vector<Calibration>& cals){
	std::vector<std::vector<std::string>> data;
	char word[256];
	for(auto cal : cals){
		std::vector<std::string> line;
		printf("Writing calibration for %s\r\n", cal.name.c_str());
		line.push_back(cal.name);
		sprintf(word, "%.6f",cal.scale);
		line.push_back(std::string(word));
		sprintf(word, "%.6f",cal.offset);
		line.push_back(std::string(word));
		sprintf(word, "%.6g",cal.variance);
		line.push_back(std::string(word));
		data.push_back(line); // add the line finally
	}
	write_csv(path, data);
}


std::string print_cal(Calibration& cal){
	char buf[256];
	sprintf(buf, "=== Name: %s ===\r\nScale:\t%.4g\r\nOffset:\t%.4g\r\nVariance:\t%.4g\r\n", 
		cal.name.c_str(),
		cal.scale,
		cal.offset,
		cal.variance
		);
	return std::string(buf);
}

/**
 * @param raw_data array of size "size" holding input data
 * @param smooth_data array of size "size" where smoothed data will be written
 * @param size the size of raw_data and smooth_data
 * @param a kernel size
 */
void smooth_data(float * raw_data, float * smooth_data, int size, int a){
	if(a >= size){
		a = size - 1;
	}
	// make "a" odd
	if(a % 2 == 0){
		a ++;
	}
	
	float kernel[a]; //
	double range = 2.5; // min and max z = -range to +range.
	double step = ((double)a-1.0)/(2.0 * range);
	double x = -range; // start at the negative of the range
	normal std_norm; // (default mean = zero, and standard deviation = unity)
	for (int i = 0; i < a; i++){
		kernel[i] = pdf(std_norm, x); // get's the probability at this point in the std normal distribution
		x += step;
	}
	int i;
	int offset = a/2; // get start index
	for(i = 0; i < size - offset; i++){
		for(int j = 0; j < a; j++){ // for each point in the kernel
			smooth_data[i] += kernel[j] * raw_data[abs((i - offset) + j)];
		}
	}
	for(i = size - offset; i < size; i++){ // copy end over, we'll do better later
		smooth_data[i] = raw_data[i];
	}
}

/**
 * @return index of maximum value in the given array
 */
int fmax(float * data, int size){
	int index = 0;
	float val = data[index];
	for(int i = 1; i < size; i++){
		if(data[i] > val){
			index = i;
			val = data[index];
		}
	}
	return index;
}

/**
 * @return average of data over size samples
 */
float favg(float * data, int size){
	float weight = 1.0/size;
	float retval = 0.0;
	for(int i = 0; i < size; i++){
		retval += weight * data[i];
	}
	return retval;
}

/**
 * @return stadnard deviation of data array
 */
float fstddev(float * data, int size){
	float m = favg(data, size);
	float sum = 0.0;
	for(int i = 0; i < size; i++){
		sum += pow(data[i] - m, 2.0);
	}

	return sum/(size - 1.0);
}

/**
 * 
 */
void decimate(float * in, float * out, int in_size, int decimation_factor){
	int j = 0;
	for(int i = 0; i < in_size - decimation_factor; i+= decimation_factor){
		out[j++] = favg(in + i, decimation_factor);
	}
}