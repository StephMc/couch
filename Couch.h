#include <string>
#include <stdint.h>
#include <vector>

struct status {
	uint8_t fl_current;
	uint8_t fr_current;
	uint8_t bl_current;
	uint8_t br_current;
	uint8_t battery_voltage;
	int8_t fl_temp;
	int8_t fr_temp;
	int8_t bl_temp;
	int8_t br_temp;
	char fault_code;
	int32_t fl_encoder;
	int32_t fr_encoder;
	int32_t bl_encoder;
	int32_t br_encoder;
	int8_t motor_ctrl_temp;
	uint8_t estopped;
};

struct fault {
	uint8_t fault_code;
	uint8_t fault_source;
	uint8_t max_fault_val;
};
	
class Couch {
	private:
		void comm(bool forBattery, void *message, int message_len, void *resp, 
				int resp_len);
		uint8_t crc8(void *data, unsigned int length);
		static const int led_pin = 10; // compiler can't do non-static consts init in header...
		static const int e_stop_pin = 2;
		static const int bat_charge_pin = 3;
		static const int switch_pin = 23;
		int port_fd;
		//std::vector<uint8_t> crc8_table; 
	
	public:
		Couch(const char* port);
		~Couch();
		void led(bool turnOn);
		void setMotors(int16_t fl, int16_t fr, int16_t bl, int16_t br);
		struct status *getStatus(void);
		void setSetting(uint8_t address, int16_t value);
		void resetFault(void);
		struct fault *getFault(void);		
};

