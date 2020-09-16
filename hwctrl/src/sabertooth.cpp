#include <sabertooth.h>

uint8_t sbth_chksum(uint8_t * packet){
	return (packet[0] + packet[1] + packet[2]) & 127;
}

int sbth_send_packet(std::string name, uint8_t * packet){
	// starting point      --> https://en.wikibooks.org/wiki/Serial_Programming/Serial_Linux#Unix_V7
	// this is really good --> https://en.wikibooks.org/wiki/Serial_Programming/termios
	struct termios tio_conf;

	int tty_fd;
	if((tty_fd = open(name.c_str(), O_RDWR | O_NDELAY | O_NONBLOCK)) < 0){
		ROS_INFO("Failed to open Sabertooth UART (%s)", name.c_str());
		return -1;
	}

	if(!isatty(tty_fd)){
		ROS_INFO("%s is not a TTY device.", name.c_str());
		return -1
	}

	 // get current config
	if(tcgetattr(tty_fd, &tio_conf) < 0){
		ROS_INFO("Failed to read config of Sabertooth UART (%s)", name.c_str());
		return -2;
	}
	// INPUT PROCESSING
	//  - convert break to null byte, no CR to NL translation,
	//  - no NL to CR translation, don't mark parity errors or breaks
	//  - no input parity check, don't strip high bit off,
	//  - no XON/XOFF software flow control
	tio_conf.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON); 

	// OUTPUT PROCESSING
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	tio_conf.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT | OFILL | OLCUC | OPOST);

	// CHARACTER PROCESSING
	// - clear current char size, no parity checking
	// - force 8-bit input
	tio_conf.c_cflag &= ~(CSIZE | PARENB);
	tio_conf.c_cflag |= CS8;

	// LINE PROCESSING
	// - echo off, echo newline off, canonical mode off
	// - extended input processing off, signal characters off
	tio_conf.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG); // line processing 

	tio_conf.c_cc[VMIN]  = 1; // one character to return from read
	tio_conf.c_cc[VTIME] = 5; // inter-character timer

	// output and input speed, 9600 baud 
	if(cfsetospeed(&tio_conf, B9600) < 0 || cfsetispeed(&tio_conf, B9600) < 0){
		ROS_INFO("Failed to set baud rate");
		return -3;
	}
	
	// apply the configuration
	if(tcsetattr(tty_fd, TCSANOW, &tio_conf) < 0){
		ROS_INFO("Failed to configure Sabertooth UART (%s)", name.c_str());
		return -4;
	} 	

	// SEND THE PACKET
	int write_status = write(tty_fd, packet, 4);

	if(write_status < 0){
		ROS_INFO("Failed to write to Sabertooth UART (%s)", name.c_str());
	}
	if(write_status < 4){
		ROS_INFO("Write to Sabertooth UART (%s) incomplete, only %d out of 4 written", name.c_str(), write_status);
	}

	close(tty_fd);

	return 0;
}