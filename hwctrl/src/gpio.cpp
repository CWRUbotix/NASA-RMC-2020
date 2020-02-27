#include <gpio.h>

int gpio_set_dir(std::string path, int mode){
  std::string fname = path + "direction";
  int fd = open(fname.c_str(), O_RDWR);
  if(f < 0){
    return -1; //fail
  }
  std::string dir_str = "";
  if(mode == GPIO_OUTPUT){
    dir_str = "out";
  }else{
    dir_str = "in";
  }
  if(write(fd, dir_str.c_str(), dir_str.length()) != 1){
    close(fd);
    return -1; // fail
  }

  close(fd);
}

int gpio_get_value_handle(std::string path){
  std::string fname = path + "value";
  return open(fname.c_str(), O_RDWR);
}

int gpio_set(int handle){
  if(handle > 0){
    char * buf = {'1'};
    return write(handle, buf, 1);
  }else{
    return -1;
  }
}

int gpio_reset(int handle){
  if(handle > 0){
    char * buf = {'0'};
    return write(handle, buf, 1);
  }else{
    return -1;
  }
}

int gpio_read(int handle){
  if(handle > 0){
    uint8_t buf[32] = {};
    int len = read(handle, buf, 32);
    if((char)buf[0] == '1'){
      return 1;
    }else if((char)buf[0] == '0'){
      return 0;
    }else{
      return -1; // what happened??
    }
  }else{
    return -1;
  }
}
