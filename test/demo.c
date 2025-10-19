#define _USE_MATH_DEFINES
#include <math.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // sleep()
#include <minibot/base.h>
#include <minibot/sensors.h>

#define ADVANCE 0.1 // How much the robot advances every step
#define OBS_DST 0.1 // Distance to obstacle to trigger avoidance

enum {
	OBS_NONE  = 0x00,
	OBS_LEFT  = 0x02,
	OBS_RIGHT = 0x01,
	OBS_FRONT = 0x03
} typedef obstacle_t;

void init_base(const char* serial_path);
void init_sensors(const char* i2c_bus_path);
void get_light_source(float* ls_angle, float* ls_strength);
void move_towards_light(float ls_angle);
obstacle_t detect_obstacles();
void avoid_obstacle(obstacle_t obs);

int main(int argc, char const **argv){
	const char* i2c_bus_path = (argc > 1) ? argv[1] : "/dev/i2c-1";
	const char* serial_path  = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

	init_sensors(i2c_bus_path);
	init_base(serial_path);

	printf("Minibot demo: behavior 1.\n");

	obstacle_t obs;
	float ls_angle, ls_strength;

	while(true){
		get_light_source(&ls_angle, &ls_strength);
		if(ls_strength < 0.1){ // Arrived to light source
			usleep(1000000);
			continue;
		}

		obs = detect_obstacles();
		if( obs ) avoid_obstacle(obs);
		else      move_towards_light(ls_angle);
	}
	return 0;
}


void get_light_source(float* ls_angle, float* ls_strength){
	/*
	* Sensor order: UNKNOWN
	*	300° 0° 60° 120° 180° 240°
	*/
	float data[8];

	*ls_angle = *ls_strength = 0;
	if(!light_sens_read(data)) return;

	float x = 0, y = 0;
	for(uint8_t i = 0; i < 8; ++i) {
		x+= data[i] * cos(i * 2.0 * M_PI / 8.0 );
		y+= data[i] * sin(i * 2.0 * M_PI / 8.0 );
	}
	*ls_angle    = atan2(y, x);
	*ls_strength = sqrt(x*x + y*y);
}


obstacle_t detect_obstacles(){
	/*
	* Sensor order (y = front):
	*	150° 90° 30° 330° 270° 210°
	*
	* Only frontal sensors are considered (150L, 90F, 30R)
	*/
	float data[6];
	dist_sens_read(data);
	bool ol = (data[0] * 0.5) <= OBS_DST;
	bool of = (data[1] * 1.0) <= OBS_DST;
	bool or = (data[2] * 0.5) <= OBS_DST;
	if( of || (ol && or) ) return OBS_FRONT;
	else if( ol ) return OBS_LEFT;
	else if( or ) return OBS_RIGHT;
	return OBS_NONE;
}


void avoid_obstacle(obstacle_t obs){
	switch(obs){
		case OBS_NONE: return;
		case OBS_LEFT:
			rotate(-M_PI_4);
			move_y(ADVANCE);
		case OBS_RIGHT:
			rotate(M_PI_4);
			move_y(ADVANCE);
		case OBS_FRONT:
			rotate(M_PI_2);
			move_y(ADVANCE);
			rotate(-M_PI_2);
	}
}


void move_towards_light(float ls_angle){
	rotate(ls_angle);
	move_y(0.1);
}


void init_base(const char* serial_path){
	// 1. Initialize serial port
	if( !serial_init(serial_path) ){
		fprintf(stderr, "Error %i opening serial port %s: %s\n", errno, serial_path, strerror(errno));
		exit(-1);
	}

	// 2. Check motor controller board
	if( !detect_mc() ){
		fprintf(stderr, "Controller board not found\n");
		exit(-1);
	}
}


void init_sensors(const char* i2c_bus_path){
	bool light_ok, dist_ok;
	// 1. Initialize I²C bus
	if( !i2c_init(i2c_bus_path) ) {
		fprintf(stderr, "Error %i opening I2C bus %s: %s\n", errno, i2c_bus_path, strerror(errno));
		exit(-1);
	}
	// 2. Enumerate I²C devices to find sensors
	find_sensors();

	// 3. Initialize sensors
	// 3.1. Initialize light source detector (turret)
	light_ok = light_sens_init();
	printf("Light sensor initialization: %s\n", light_ok ? "OK" : "Err");
	if( !light_ok ) exit(-1);
	// 3.3. Initialize obstacle detectors (IR)
	dist_ok  = lidar_sens_init();
	printf("IR obstacle detection initialization: %s\n", dist_ok ? "OK" : "Err");
	if( !dist_ok ) exit(-1);
}
