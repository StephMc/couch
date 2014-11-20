#include "Couch.h"
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
// For GPIO http://www.airspayce.com/mikem/bcm2835/
using namespace std;


uint8_t crc8_table[] = {
                    0x00, 0x3e, 0x7c, 0x42, 0xf8, 0xc6, 0x84, 0xba, 0x95, 0xab, 0xe9, 0xd7,
                    0x6d, 0x53, 0x11, 0x2f, 0x4f, 0x71, 0x33, 0x0d, 0xb7, 0x89, 0xcb, 0xf5,
                    0xda, 0xe4, 0xa6, 0x98, 0x22, 0x1c, 0x5e, 0x60, 0x9e, 0xa0, 0xe2, 0xdc,
                    0x66, 0x58, 0x1a, 0x24, 0x0b, 0x35, 0x77, 0x49, 0xf3, 0xcd, 0x8f, 0xb1,
                    0xd1, 0xef, 0xad, 0x93, 0x29, 0x17, 0x55, 0x6b, 0x44, 0x7a, 0x38, 0x06,
                    0xbc, 0x82, 0xc0, 0xfe, 0x59, 0x67, 0x25, 0x1b, 0xa1, 0x9f, 0xdd, 0xe3,
                    0xcc, 0xf2, 0xb0, 0x8e, 0x34, 0x0a, 0x48, 0x76, 0x16, 0x28, 0x6a, 0x54,
                    0xee, 0xd0, 0x92, 0xac, 0x83, 0xbd, 0xff, 0xc1, 0x7b, 0x45, 0x07, 0x39,
                    0xc7, 0xf9, 0xbb, 0x85, 0x3f, 0x01, 0x43, 0x7d, 0x52, 0x6c, 0x2e, 0x10,
                    0xaa, 0x94, 0xd6, 0xe8, 0x88, 0xb6, 0xf4, 0xca, 0x70, 0x4e, 0x0c, 0x32,
                    0x1d, 0x23, 0x61, 0x5f, 0xe5, 0xdb, 0x99, 0xa7, 0xb2, 0x8c, 0xce, 0xf0,
                    0x4a, 0x74, 0x36, 0x08, 0x27, 0x19, 0x5b, 0x65, 0xdf, 0xe1, 0xa3, 0x9d,
                    0xfd, 0xc3, 0x81, 0xbf, 0x05, 0x3b, 0x79, 0x47, 0x68, 0x56, 0x14, 0x2a,
                    0x90, 0xae, 0xec, 0xd2, 0x2c, 0x12, 0x50, 0x6e, 0xd4, 0xea, 0xa8, 0x96,
                    0xb9, 0x87, 0xc5, 0xfb, 0x41, 0x7f, 0x3d, 0x03, 0x63, 0x5d, 0x1f, 0x21,
                    0x9b, 0xa5, 0xe7, 0xd9, 0xf6, 0xc8, 0x8a, 0xb4, 0x0e, 0x30, 0x72, 0x4c,
                    0xeb, 0xd5, 0x97, 0xa9, 0x13, 0x2d, 0x6f, 0x51, 0x7e, 0x40, 0x02, 0x3c,
                    0x86, 0xb8, 0xfa, 0xc4, 0xa4, 0x9a, 0xd8, 0xe6, 0x5c, 0x62, 0x20, 0x1e,
                    0x31, 0x0f, 0x4d, 0x73, 0xc9, 0xf7, 0xb5, 0x8b, 0x75, 0x4b, 0x09, 0x37,
                    0x8d, 0xb3, 0xf1, 0xcf, 0xe0, 0xde, 0x9c, 0xa2, 0x18, 0x26, 0x64, 0x5a,
                    0x3a, 0x04, 0x46, 0x78, 0xc2, 0xfc, 0xbe, 0x80, 0xaf, 0x91, 0xd3, 0xed,
                    0x57, 0x69, 0x2b, 0x15};

// Host -> motor controller
struct set_motor_mesg {
	char cmd;
	int16_t fl_speed;
	int16_t fr_speed;
	int16_t bl_speed;
	int16_t br_speed;
	uint8_t crc;
} __attribute__((packed));

struct set_setting_mesg {
	char cmd;
	uint8_t setting_addr;
	int16_t value;
	uint8_t crc;
} __attribute__((packed));

struct read_setting_mesg {
	char cmd;
	uint8_t setting_addr;
	uint8_t crc;
} __attribute__((packed));

struct read_status_mesg {
	char cmd;
	uint8_t crc;
} __attribute__((packed));

struct reset_fault_mesg {
	char cmd;
	uint8_t crc;
} __attribute((packed));

struct read_charge_status_mesg {
	char cmd;
	uint8_t crc;
} __attribute((packed));

struct read_charge_chars_mesg {
	char cmd;
	uint8_t crc;
} __attribute((packed));

struct read_fault_status_mesg {
	char cmd;
	uint8_t crc;
} __attribute((packed));

struct read_system_state_mesg {
	char cmd;
	uint8_t crc;
} __attribute((packed));

// Motor controller -> host
struct ack_resp {
	char cmd;
	char sent_cmd;
	uint8_t crc;
} __attribute__((packed));

struct status_resp {
	char cmd;
	uint8_t fl_current;
	uint8_t fr_current;
	uint8_t bl_current;
	uint8_t br_current;
	uint8_t battery_voltage;
	uint8_t fl_temp;
	uint8_t fr_temp;
	uint8_t bl_temp;
	uint8_t br_temp;
	char fault_code;
	uint16_t padding;
	int32_t fl_encoder;
	int32_t fr_encoder;
	int32_t bl_encoder;
	int32_t br_encoder;
	char motor_ctrl_temp;
	uint8_t estopped;
	uint8_t crc;
} __attribute__((packed));

struct setting_value_resp {
	char cmd;
	uint8_t setting_addr;
	int16_t value;
	uint8_t crc;
} __attribute((packed));

struct fault_resp {
	char cmd;
	uint8_t fault_code;
	uint8_t fault_source;
	uint8_t max_value;
	uint8_t crc;
} __attribute((packed));

struct charge_status_resp {
	char cmd;
	uint8_t padding;
	uint16_t bat1_v;
	uint16_t bat2_v;
	uint16_t bat3_v;
	uint16_t bat4_v;
	uint8_t charge_flags;
	char temp;
	uint16_t curr_draw;
	uint8_t average_curr;
	uint8_t charge_progress;
	uint16_t pack_capacity;
	uint8_t bat1_z;
	uint8_t bat2_z;
	uint8_t bat3_z;
	uint8_t bat4_z;
	uint8_t charge_status;
	uint8_t crc;
} __attribute((packed));

struct charge_chars_resp {
	char cmd;
	uint8_t crc;
} __attribute((packed));

struct system_state_resp {
	char cmd;
	uint8_t state_flags;
	uint8_t crc;
} __attribute((packed));


Couch::Couch(const char* port) {
	
	// Open serial port
	port_fd = open(port, O_RDWR| O_NOCTTY);
	if (port_fd == -1) {
		cout << "Error in open uart port" << endl;
	} else {
		cout << "Opened uart port" << endl;
	}
	
	// Set parameters
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Error Handling 
	if (tcgetattr(Couch::port_fd, &tty) != 0) {
		cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
	}
	cout << "Setting up uart" << endl;

	// Set Baud Rate 
	cfsetospeed(&tty, (speed_t)B115200);
	cfsetispeed(&tty, (speed_t)B115200);

	// Setting other Port Stuff 
	tty.c_cflag &= ~PARENB; // Make 8n1
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	tty.c_cflag &= ~CRTSCTS; // no flow control
	tty.c_cc[VMIN] = 1; // read doesn't block
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
	tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
	cout << "About to make raw" << endl;

	// Make raw 
	cfmakeraw(&tty);

	// Flush Port, then applies attributes
	tcflush(port_fd, TCIFLUSH);
	if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
		cout << "Error " << errno << " from tcsetattr: " << strerror(errno) << endl;
	}
	
	/*crc8_table = { 
		    0x00, 0x3e, 0x7c, 0x42, 0xf8, 0xc6, 0x84, 0xba, 0x95, 0xab, 0xe9, 0xd7, 
		    0x6d, 0x53, 0x11, 0x2f, 0x4f, 0x71, 0x33, 0x0d, 0xb7, 0x89, 0xcb, 0xf5, 
		    0xda, 0xe4, 0xa6, 0x98, 0x22, 0x1c, 0x5e, 0x60, 0x9e, 0xa0, 0xe2, 0xdc, 
		    0x66, 0x58, 0x1a, 0x24, 0x0b, 0x35, 0x77, 0x49, 0xf3, 0xcd, 0x8f, 0xb1, 
		    0xd1, 0xef, 0xad, 0x93, 0x29, 0x17, 0x55, 0x6b, 0x44, 0x7a, 0x38, 0x06, 
		    0xbc, 0x82, 0xc0, 0xfe, 0x59, 0x67, 0x25, 0x1b, 0xa1, 0x9f, 0xdd, 0xe3, 
		    0xcc, 0xf2, 0xb0, 0x8e, 0x34, 0x0a, 0x48, 0x76, 0x16, 0x28, 0x6a, 0x54, 
		    0xee, 0xd0, 0x92, 0xac, 0x83, 0xbd, 0xff, 0xc1, 0x7b, 0x45, 0x07, 0x39, 
		    0xc7, 0xf9, 0xbb, 0x85, 0x3f, 0x01, 0x43, 0x7d, 0x52, 0x6c, 0x2e, 0x10, 
		    0xaa, 0x94, 0xd6, 0xe8, 0x88, 0xb6, 0xf4, 0xca, 0x70, 0x4e, 0x0c, 0x32, 
		    0x1d, 0x23, 0x61, 0x5f, 0xe5, 0xdb, 0x99, 0xa7, 0xb2, 0x8c, 0xce, 0xf0, 
		    0x4a, 0x74, 0x36, 0x08, 0x27, 0x19, 0x5b, 0x65, 0xdf, 0xe1, 0xa3, 0x9d, 
		    0xfd, 0xc3, 0x81, 0xbf, 0x05, 0x3b, 0x79, 0x47, 0x68, 0x56, 0x14, 0x2a, 
		    0x90, 0xae, 0xec, 0xd2, 0x2c, 0x12, 0x50, 0x6e, 0xd4, 0xea, 0xa8, 0x96, 
		    0xb9, 0x87, 0xc5, 0xfb, 0x41, 0x7f, 0x3d, 0x03, 0x63, 0x5d, 0x1f, 0x21, 
		    0x9b, 0xa5, 0xe7, 0xd9, 0xf6, 0xc8, 0x8a, 0xb4, 0x0e, 0x30, 0x72, 0x4c, 
		    0xeb, 0xd5, 0x97, 0xa9, 0x13, 0x2d, 0x6f, 0x51, 0x7e, 0x40, 0x02, 0x3c, 
		    0x86, 0xb8, 0xfa, 0xc4, 0xa4, 0x9a, 0xd8, 0xe6, 0x5c, 0x62, 0x20, 0x1e, 
		    0x31, 0x0f, 0x4d, 0x73, 0xc9, 0xf7, 0xb5, 0x8b, 0x75, 0x4b, 0x09, 0x37, 
		    0x8d, 0xb3, 0xf1, 0xcf, 0xe0, 0xde, 0x9c, 0xa2, 0x18, 0x26, 0x64, 0x5a, 
		    0x3a, 0x04, 0x46, 0x78, 0xc2, 0xfc, 0xbe, 0x80, 0xaf, 0x91, 0xd3, 0xed, 
		    0x57, 0x69, 0x2b, 0x15};*/
	cout << "Finished initalizing couch" << endl;	
}

Couch::~Couch(void) {
	close(Couch::port_fd);
}

uint8_t Couch::crc8(void *data, unsigned int length) {
	uint8_t crc = 0xff;
	uint8_t *d = (uint8_t*)data;
	for (int i = 0; i < length; i++) {
		crc = crc8_table[crc ^ d[i]];
	}
	return crc;
}

void Couch::comm(bool forBattery, void *message, int message_len, void *resp, int resp_len) {
	
	// Add crc - all messages should have a spare byte at end
	// so the crc can be added. Don't include the crc in the crc generation
	((char*)message)[message_len - 1] = crc8(message, message_len - 1);
//	cout << "Writing message: " << endl;
//	for (int i = 0; i < message_len; i++) {
//		printf("%d: %02x\n", i, *((char *) message + i));
//	}
	int written = 0;
	do {
		written += write(port_fd, message + written, message_len - written);
	} while (written < message_len);
//	cout << "Written packet, expecting to read " << resp_len << endl;
//	tcflush(port_fd, TCIOFLUSH); 
	/*int readCount = 0;
	do {
		int r = read(port_fd, resp + readCount, resp_len - readCount);
		//if (r < 0) {
		//	ROS_INFO("Couch comm error");
		//	break;
		//} else if (r == 0) {
		//	ROS_INFO("read nothing");
		//}
		readCount += r;
//		cout << "reading... " << readCount<< endl;
	} while (readCount < resp_len);
//	cout << "Read packet" << endl;
	// Check the crc is correct
	uint8_t expected = crc8(resp, resp_len - 1);
	if (((char*)resp)[resp_len - 1] != expected) {
		cout << "comm crc is incorrect" << endl;
	}*/
}

void Couch::led(bool turnOn) {
}

void Couch::setMotors(int16_t fl, int16_t fr, int16_t bl, int16_t br) {
	struct set_motor_mesg mesg;
	struct ack_resp resp;
	mesg.cmd = 'A';
	mesg.fl_speed = fl;
	mesg.fr_speed = fr;
	mesg.bl_speed = bl;
	mesg.br_speed = br;
	cout << "Writing " << sizeof(struct set_motor_mesg) << "bytes." << endl;
	comm(false, &mesg, sizeof(struct set_motor_mesg), &resp, sizeof(struct ack_resp));
	if (resp.cmd != 'A' || resp.sent_cmd != 'A') {
		 cout << "Set motors had a error" << endl;
	}
}

struct status* Couch::getStatus(void) {
	struct read_status_mesg mesg;
	struct status_resp resp;
	struct status *status = new struct status();

	mesg.cmd = 'D';
	Couch::comm(false, &mesg, sizeof(struct read_status_mesg), &resp, sizeof(struct status_resp));
	if (resp.cmd != 'E') {
		cout << "Get status had a error" << endl;
		return NULL;
	} else {
		status->fl_current = resp.fl_current;
		status->fr_current = resp.fr_current;
		status->bl_current = resp.bl_current;
		status->br_current = resp.br_current;
		status->battery_voltage = resp.battery_voltage;
		status->fl_temp = resp.fl_temp;
		status->fr_temp = resp.fr_temp;
		status->bl_temp = resp.bl_temp;
		status->br_temp = resp.br_temp;
		status->fault_code = resp.fault_code;
		status->fl_encoder = resp.fl_encoder;
		status->fr_encoder = resp.fr_encoder;
		status->bl_encoder = resp.bl_encoder;
		status->br_encoder = resp.br_encoder;
		status->motor_ctrl_temp = resp.motor_ctrl_temp;
		status->estopped = resp.estopped;
		//for (int i = 0; i < sizeof(struct status); i++) {
		//	printf("%d:  %x\n", i, *((char*)status + i));
		//}
		return status;
	}
}
	
void Couch::setSetting(uint8_t address, int16_t value) {
	struct set_setting_mesg mesg;
	struct ack_resp resp;
	mesg.cmd = 'B';
	mesg.setting_addr = address;
	mesg.value = value;
	comm(false, &mesg, sizeof(struct set_setting_mesg), &resp, 
		sizeof(struct ack_resp));
	if (resp.cmd != 'A') {
		cout << "Set setting command failed" << endl;
	} 	
}

void Couch::resetFault(void) {
	struct reset_fault_mesg mesg;
	struct ack_resp resp; 	
	mesg.cmd = 'E';
	Couch::comm(false, &mesg, sizeof(struct reset_fault_mesg), &resp, 
			sizeof(struct ack_resp));
	if (resp.cmd != 'A') {
		cout << "Reset fault command failed" << endl;	
	}
}

struct fault *Couch::getFault(void) {
	struct read_fault_status_mesg mesg;
	struct fault_resp resp;
	mesg.cmd = 'H';
	comm(false, &mesg, sizeof(struct read_fault_status_mesg), &resp,
			sizeof(struct fault_resp));
	if (resp.cmd != 'D') {
		cout << "Get fault command failed" << endl;
		return NULL;
	} else {
		struct fault *f = new struct fault();
		f->fault_code = resp.fault_code;
		f->fault_source = resp.fault_source;
		f->max_fault_val = resp.max_value;
		return f;
	}
}
