#include <gpio.h>

int gpio_init(std::string path, int mode, int value){
  if(mode == GPIO_INPUT){
    gpio_set_dir(path, mode);
    return gpio_get_value_handle(path);
  }else if(mode == GPIO_OUTPUT){
    std::string fname = path + "value";
    int fd = open(fname.c_str(), O_RDWR);
    if(value >= 1){
      gpio_set(fd);
    }else{
      gpio_reset(fd);
    }
    gpio_set_dir(path, GPIO_OUTPUT);
    return fd;
  }else{
    return -1;
  }
}

std::string gpio_get_dir(std::string path){
  std::string fname = path + "direction";
  std::ifstream file(fname, std::ios::in);
  std::string retval;
  std::getline(file, retval);
  file.close();
  return retval;
}

int gpio_set_dir(std::string path, int mode){
  std::string fname = path + "direction";
  int fd = open(fname.c_str(), O_RDWR);
  if(fd < 0){
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
  std::string dir = gpio_get_dir(path);
  if(dir.compare("in") == 0){
    return open(fname.c_str(), O_RDONLY);
  }else{
    return open(fname.c_str(), O_RDWR);
  }
}

int gpio_set(int handle){
  if(handle > 0){
    char b = '1';
    return write(handle, &b, 1);
  }else{
    return -1;
  }
}

int gpio_reset(int handle){
  if(handle > 0){
    char b = '0';
    return write(handle, &b, 1);
  }else{
    return -1;
  }
}

int gpio_read(int handle){
  if(handle > 0){
    uint8_t buf = 0x00;
    int len = read(handle, &buf, 1);
    if(buf == 1){
      return 1;
    }else if(buf == 0){
      return 0;
    }else{
      return GPIO_ERR_READ_FAILED; // what happened??
    }
  }else{
    return GPIO_ERR_BAD_HANDLE;
  }
}
