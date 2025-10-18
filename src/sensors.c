#include "sensors.h"
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>    // open()
#include <unistd.h>   // close()
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
// namespace i2c{
// bool enumerate();

#define LIDAR_MUX_ADDR 0x70
#define LIDAR_DEF_ADDR 0x29
#define LIGHT_SEN_ADDR 0x48
#define DIST_SEN_ADDR  0x49
#define FLOOR_SEN_ADDR 0x4b

#define ADS7830_SINGLE       0x80
#define ADS7830_DIFFERENTIAL 0x00
#define ADS7830_CHANNEL0     0x00
#define ADS7830_CHANNEL1     0x10
#define ADS7830_CHANNEL2     0x20
#define ADS7830_CHANNEL3     0x30
#define ADS7830_CHANNEL4     0x40
#define ADS7830_CHANNEL5     0x50
#define ADS7830_CHANNEL6     0x60
#define ADS7830_CHANNEL7     0x70
#define ADS7830_POWER_DOWN   0x00
#define ADS7830_POWER_ADIR   0x0c
#define ADS7830_POWER_AD_ON  0x04
#define ADS7830_POWER_AD_OFF 0x00
#define ADS7830_POWER_IR_ON  0x08
#define ADS7830_POWER_IR_OFF 0x00

static int fd = 0;
static bool *i2c_slaves = NULL;
static uint8_t lidars[8];
static uint8_t lidar_count = 0;
static bool lidar_continuous = false;


static int i2c_open(const char* i2c_bus_path);
static bool i2c_rbyte(uint8_t addr, uint8_t* val);
static bool i2c_wbyte(uint8_t addr, uint8_t val);
static void enumerate_devices_in_range(bool slave_list[128],
	uint8_t first, // 0x08
	uint8_t last   // 0x77
);
static inline void enumerate_devices(bool slave_list[128]){
	enumerate_devices_in_range(slave_list, 0x08, 0x77);
}

static bool lidar_init(uint8_t new_addr);
static int lidar_rreg(uint8_t addr, uint16_t reg);
static void lidar_wreg(uint8_t addr, uint16_t reg, uint8_t val);
static int lidar_read(uint8_t addr);
static bool lidar_read_all(uint8_t data[8]);
static void lidar_start_cont(int period_ms); // Default 100ms
static void lidar_stop_cont();

static int adc_read(uint8_t addr, uint8_t ch);
static bool adc_read_all(uint8_t addr, uint8_t data[8]);
static inline uint8_t adc_ch2C(uint8_t ch);

static void swap(uint8_t* var, size_t i, size_t j);
static bool is_big_endian(void);



/* ** *****************************************************************
*
* High-level, interface function definitions
*
** ** ****************************************************************/

bool i2c_init(const char* i2c_bus_path){
	if(fd) return true;
	return i2c_open(i2c_bus_path) != -1;
}

void find_sensors(){
	if(!i2c_slaves) i2c_slaves = (bool*)calloc(128, sizeof(bool));
	enumerate_devices(i2c_slaves);
}

bool dist_sens_init(){
	if(!i2c_slaves) find_sensors();
	return true;
}

bool light_sens_init(){
	if(!i2c_slaves) find_sensors();
	return true;
}

bool floor_sens_init(){
	if(!i2c_slaves) find_sensors();
	return true;
}

uint8_t lidar_sens_init(){
	lidar_count = 0;
	uint8_t mux_mask = 0x00;
	if(!i2c_slaves) find_sensors();
	// 1. Check mux is available
	if(!i2c_slaves[LIDAR_MUX_ADDR]) return false;
	// 2. Initialize all LIDARs in MUX
	for(uint8_t i = 0, ch = 0x01; i < 8; ++i, ch<<=1){
		lidars[lidar_count] = 0;
		// if (i == 3) continue;
		// Skip initialized LIDARs (addr already set)
		if(i2c_slaves[0x50+i]){
			printf("LIDAR %02x already initialized. Skipping.\n");
			mux_mask |= ch;
			lidars[lidar_count++] = 0x50+i;
			continue;
		}
		// Shift MUX
		i2c_wbyte(LIDAR_MUX_ADDR, ch);
		usleep(1300);
		if (lidar_init(0x50 + i))
			lidars[lidar_count++] = 0x50+i;
			mux_mask |= ch;
	}
	i2c_wbyte(LIDAR_MUX_ADDR, mux_mask);
	usleep(1300);
	enumerate_devices_in_range(i2c_slaves, 50, 57);

	return lidar_count;
}


bool dist_sens_read(float data[6]){
	uint8_t raw[8];
	bool res = adc_read_all(DIST_SEN_ADDR, raw);
	for(uint8_t i = 0; i < 6; ++i)
		data[i] = raw[i]/255.0;
	return res;
}


bool dist_sens_readu(uint8_t data[6]){
	uint8_t raw[8];
	bool res = adc_read_all(DIST_SEN_ADDR, raw);
	for(uint8_t i = 0; i < 6; ++i)
		data[i] = raw[i];
	return res;
}



bool floor_sens_read(float data[4]){
	uint8_t raw[8];
	bool res = adc_read_all(FLOOR_SEN_ADDR, raw);
	data[0] = raw[3]/255.0f;
	data[1] = raw[2]/255.0f;
	data[2] = raw[1]/255.0f;
	data[3] = raw[0]/255.0f;
	return res;
}


bool light_sens_read(float data[8]){
	uint8_t raw[8];
	bool res = adc_read_all(LIGHT_SEN_ADDR, raw);
	for(uint8_t i = 0; i < 8; ++i) data[i] = raw[i]/255.0f;
	return res;
}


bool lidar_sens_readu(uint8_t data[8]){
	for(uint8_t i = 0; i < 8; ++i){
		if(!lidars[i]) break;
		int res = lidar_read(lidars[i]);
		data[i] = res;
	}
	return true;
}


bool lidar_sens_read(float data[8]){
	uint8_t raw[8];
	bool res = lidar_read_all(raw);
	for(uint8_t i = 0; i < 8; ++i)
		// data[i] = raw[i] != 0xff ? raw[i] * 0.001 : 0.0/0.0;
		data[i] = raw[i] != 0xff ? raw[i] * 0.001 : -1.0;

	return res;
}





/* ** *****************************************************************
*
* Low-level function definitions & local variables
*
** ** ****************************************************************/
int i2c_open(const char* i2c_bus_path){
	if(fd) return fd;
	int ret;
	// 1. Open the bus
	fd = open(i2c_bus_path, O_RDWR);
	if(fd <= 0){
		perror(i2c_bus_path);
		return -1;
	}
	return fd;
}



bool i2c_rbyte(uint8_t addr, uint8_t* val){
	struct i2c_msg messages[] = {
		{ addr, I2C_M_RD, 1, val }
	};
	struct i2c_rdwr_ioctl_data iodata = { messages, 1 };
	return ioctl(fd, I2C_RDWR, &iodata) != -1;
}



bool i2c_wbyte(uint8_t addr, uint8_t val){
	struct i2c_msg messages[] = {
		{ addr, 0, 1, &val }
	};
	struct i2c_rdwr_ioctl_data iodata = { messages, 1 };
	return ioctl(fd, I2C_RDWR, &iodata) != -1;
}



void enumerate_devices_in_range(bool slave_list[128], uint8_t first, uint8_t last){
	bool res;
	char cmd = 'r';
	uint8_t tmp;
	if(last > 127) last = 127;
	if(first >= last) first = last;
	for(uint8_t i = first; i <= last; ++i){
		// cmd = ((i >= 0x30 && i <= 0x37) || (i >= 0x50 && i <= 0x5f)) ? 'r' : 'w';
		slave_list[i] = false;
		// if((i < 0x08) || (i > 0x77))
		// 	continue;
		if( ioctl(fd, I2C_SLAVE, i) < 0 ){
			printf("Cannot access i2c addr %u\n", i);
			continue;
		}
		if(cmd == 'r') res = i2c_rbyte(i, &tmp);
		else           res = i2c_wbyte(i, 0);
		if(res) slave_list[i] = true;
	}

	printf("Slaves detected:");
	for(uint8_t i = 0; i < 128; ++i){
		if(slave_list[i]) printf(" %02x", i);
	}
	printf("\n");
}



bool lidar_init(uint8_t new_addr){
	if(!fd) return false;

	int ret;
	// 1. Check model id = 0xb4
	ret = lidar_rreg(LIDAR_DEF_ADDR, 0x00);
	if( ret != 0xb4 ){
		// fprintf(stderr, "Invalid sensor id %d\n", ret);
		return false;
	}

	// 2. Set hold (0x17) = 1 before doing changes to settings
	lidar_wreg(LIDAR_DEF_ADDR, 0x17, 0x01);
	// printf("0x0017 (0x01): %02x\n", lidar_rreg(LIDAR_DEF_ADDR, 0x17));

	// 3. Check reset? (reg 0x16 == 1 after fresh reset)
	ret = lidar_rreg(LIDAR_DEF_ADDR, 0x16);
	if( ret != 0x01 )
		printf("Device is not fresh out of refresh\n");

	// 4. Enable ALS and Range ready interrupts (reg 0x14 = 0x20 | 0x04)
	lidar_wreg(LIDAR_DEF_ADDR, 0x14, 0x04);
	// 5. Set measurement time to 10ms (reg 0x10a) for max accuracy
	// 10ms / 64.5us = 155 = 9b
	lidar_wreg(LIDAR_DEF_ADDR, 0x010a, 0x9b);
	//
	// printf("0x0014 (0x04): %02x\n", lidar_rreg(LIDAR_DEF_ADDR, 0x14));
	// 7. Clean RESET flag
	lidar_wreg(LIDAR_DEF_ADDR, 0x16, 0x00);
	// printf("0x0016 (0x00): %02x\n", lidar_rreg(LIDAR_DEF_ADDR, 0x16));
	// 8. Set new address
	lidar_wreg(LIDAR_DEF_ADDR, 0x0212, new_addr);
	// printf("0x0212 (0x5X): %02x\n", lidar_rreg(new_addr, 0x0212));
	// 9. Unhold
	lidar_wreg(new_addr, 0x17, 0x00);
	// printf("0x0017 (0x00): %02x\n", lidar_rreg(new_addr, 0x17));

	return true;
}


int lidar_rreg(uint8_t addr, uint16_t reg){
	uint8_t buffer[2];
	memcpy(buffer, &reg, 2);
	if( !is_big_endian() ) swap(buffer, 0, 1);

	struct i2c_msg messages[] = {
		{ addr,        0, 2, buffer }, // Write device and register addresses
		{ addr, I2C_M_RD, 1, buffer }, // Read register data
	};
	struct i2c_rdwr_ioctl_data iodata = { messages, 2 };
	int res = ioctl(fd, I2C_RDWR, &iodata);
	if (res != 2){
		// char serr[64];
		// sprintf(serr, "Failed to read register %04x", reg);
		// perror(serr);
		return -1;
	}
	return buffer[0];
}


void lidar_wreg(uint8_t addr, uint16_t reg, uint8_t val){
	uint8_t buffer[3];
	memcpy(buffer, &reg, 2);
	if( !is_big_endian() ) swap(buffer, 0, 1);
	buffer[2] = val;

	struct i2c_msg messages[] = {
		{ addr, 0, 3, buffer } // Write dev addr, reg addr, and data
	};
	struct i2c_rdwr_ioctl_data iodata = { messages, 1 };
	int res = ioctl(fd, I2C_RDWR, &iodata);
	if (res != 1){
		// char serr[64];
		// sprintf(serr, "Failed to write register %04x", reg);
		// perror(serr);
		// return -1;
	}
}


void lidar_start_cont(int period_ms){
	uint8_t period = period_ms / 10 > 255 ? 255 : period / 10;
	for(uint8_t i = 0; i < lidar_count; ++i){
		lidar_wreg(lidars[i], 0x18, 0x03); // Continuous read
		lidar_wreg(lidars[i], 0x1b, period); // Set period
	}
	lidar_continuous = true;
}


void lidar_stop_cont(){
	for(uint8_t i = 0; i < lidar_count; ++i)
		lidar_wreg(lidars[i], 0x18, 0x00); // Read off
	lidar_continuous = false;
}


int lidar_read(uint8_t addr){
	int8_t tries = 20;
	if(!lidar_continuous)
		lidar_wreg(addr, 0x18, 0x01); // One read
	while(tries--){ // Wait for interrrupt
		if( lidar_rreg(addr, 0x4f) & 0x04 ) break;
		usleep(10000);
	}
	if( tries < 0 ) return -1;
	int dist = lidar_rreg(addr, 0x62);
	lidar_wreg(addr, 0x15, 0x07); // Clear interrupt
	return dist;
}


bool lidar_read_all(uint8_t data[8]){
	int8_t tries = 20;
	 // Start read in all and clear output
	for(uint8_t i = 0; i < lidar_count; ++i){
		if(!lidar_continuous)
			lidar_wreg(lidars[i], 0x18, 0x01); // One read
		data[i] = 0xff;
	}
	// On continuous operation, just take the latest measurement
	if(lidar_continuous){
		for(uint8_t i = 0; i < lidar_count; ++i)
			data[i] = lidar_rreg(lidars[i], 0x52);
		return true;
	}
	// Wait for interrupt on the first, abort if not
	while(tries--){
		if( lidar_rreg(lidars[0], 0x4f) & 0x04 ) break;
		usleep(10000);
	}
	if(tries < 0) return false;
	// Read all and clear interrupt
	for(uint8_t i = 0; i < lidar_count; ++i){
		if( !(lidar_rreg(lidars[i], 0x4f) & 0x04) ) usleep(10000);
		data[i] = lidar_rreg(lidars[i], 0x62);
		lidar_wreg(lidars[i], 0x15, 0x07); // Clear interrupt
	}
	return true;
}



inline
uint8_t adc_ch2C(uint8_t ch){
	// From Datasheet page 14 with SD=1:
	// Ch C[2:0]    Ch C[2:0]
	//  0   000      1   100
	//  2   001      3   101
	//  4   010      5   110
	//  6   011      7   111
	//  => C = (ch%2 == 0) ? ch / 2 : ch / 2 | 0x04
	return ((ch%2 == 0) ? ch / 2 : ch / 2 | 0x04) << 4;
}

int adc_read(uint8_t addr, uint8_t ch){
	if((ch < 0) || (ch > 7)) return -1;
	uint8_t cmd = ADS7830_SINGLE | ADS7830_POWER_AD_ON | adc_ch2C(ch);
	uint8_t data = 0;

	struct i2c_msg messages[] = {
		{ addr,        0, 1, &cmd },
		{ addr, I2C_M_RD, 1, &data },
	};
	struct i2c_rdwr_ioctl_data iodata = { messages, 2 };
	int res = ioctl(fd, I2C_RDWR, &iodata);
	if (res != 2) return -1;
	return data;
}


bool adc_read_all(uint8_t addr, uint8_t data[8]){
	int res;
	uint8_t cmd;

	for(uint8_t i = 0; i < 8; ++i) {
		cmd = ADS7830_SINGLE | ADS7830_POWER_AD_ON | adc_ch2C(i);
		struct i2c_msg messages[] = {
			{ addr,        0, 1, &cmd },
			{ addr, I2C_M_RD, 1, &data[i] },
		};
		struct i2c_rdwr_ioctl_data iodata = { messages, 2 };
		res = ioctl(fd, I2C_RDWR, &iodata);
		if (res != 2) return false;
	}
	return true;
}


void swap(uint8_t* var, size_t i, size_t j){
	uint8_t c = var[i];
	var[i] = var[j];
	var[j] = c;
}


bool is_big_endian(void){
	static union {
		uint32_t i;
		char c[4];
	} bint = {0x01020304};
	return bint.c[0] == 1;
}