#include "util.h"

#include <fstream>
#include <sstream>

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


    bool write_csv(std::string fpath, std::vector<std::vector<std::string>> data){
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