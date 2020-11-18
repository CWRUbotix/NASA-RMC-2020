#include "util.h"

#include <fstream>
#include <sstream>
#include <numeric>
#include <cmath>


namespace csv {
    std::vector<std::vector<std::string>> read_csv(const std::string& fpath){
        std::ifstream config_file;
        config_file.open(fpath.c_str(), std::ifstream::in);
        
        std::string line;
        std::string word;

        std::vector<std::vector<std::string>> csv;

        int line_num = 0;
        int can_id_ind = -1;
        int device_type_ind = -1;
        if(config_file.is_open()){
            while(std::getline(config_file, line)){
                std::vector<std::string> words; 	// this will be our line
                line.push_back(','); // make sure we can read the last word
                std::istringstream line_stream(line);
                while(std::getline(line_stream, word, ',')){
                    words.push_back(word);
                }
                csv.push_back(words);
            }
        }

        config_file.close();

        return csv;
    }


    bool write_csv(const std::string& fpath, std::vector<std::vector<std::string>> data){
        std::ofstream file;
        file.open(fpath.c_str(), std::ofstream::out);
        if(file.is_open()){
            
            for(auto line : data){
                for(auto word : line){
                    file << word << ",";
                }
                file.seekp(-1, std::ios::cur); // go back one character to over-write the last comma
                file << "\n";
            }
            file.close();
            return true;
        }else{
            return false;
        }
    }
}

namespace buffer {
    void append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    append_int16(buffer, (int16_t)(number * scale), index);
}

void append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    append_int32(buffer, (int32_t)(number * scale), index);
}

int16_t get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

uint16_t get_uint16(const uint8_t *buffer, int32_t *index) {
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

int32_t get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

uint32_t get_uint32(const uint8_t *buffer, int32_t *index) {
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

float get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)get_int16(buffer, index) / scale;
}

float get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)get_int32(buffer, index) / scale;
}

}