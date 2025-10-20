#define BASE_PWM   0.8
#define _USE_MATH_DEFINES
#include <math.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
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
const char* get_light_quadrant_name(float ls_angle);
const char* get_obstacle_name(obstacle_t obs);
void mv(float dist, float angle);
void ctrlc_handler(int signum);


int main(int argc, char const **argv){
	const char* i2c_bus_path = (argc > 1) ? argv[1] : "/dev/i2c-1";
	const char* serial_path  = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

	init_sensors(i2c_bus_path);
	init_base(serial_path);
	signal(SIGINT, ctrlc_handler);

	printf("Minibot demo: behavior 1\n");
	float vbat = read_batt_volt();
	printf("Battery level: %0.2fV (%0.1f%)\n", read_batt_volt(), read_batt_perc());

	obstacle_t obs;
	float ls_angle, ls_strength;

	while(true){
		get_light_source(&ls_angle, &ls_strength);
		if(ls_strength < 0.05){ // Arrived to light source
			printf("Arrived to light source (r=%0.3f, θ=%0.1f)\n", ls_strength, ls_angle*57.3);
			usleep(1000000);
			continue;
		}
		printf("Light source detected: %s (θ=%0.1f)\n", get_light_quadrant_name(ls_angle), ls_angle*57.3);
		obs = detect_obstacles();
		printf("  Obstacles? %s\n", get_obstacle_name(obs));
		if( obs ) avoid_obstacle(obs);
		else      move_towards_light(ls_angle);
		usleep(1000000);
	}
	return 0;
}


void get_light_source(float* ls_angle, float* ls_strength){
	/*
	* Sensor order (y = front):
	*	   90° 45° 0° 315° 270° 225° 180° 135°
	*/
	float data[8];

	*ls_angle = *ls_strength = 0;
	if( !light_sens_read(data) ) return;

	float x = 0, y = 0, phi = M_PI_2;
	for(uint8_t i = 0; i < 8; ++i) {
		x+= data[i] * cos(phi);
		y+= data[i] * sin(phi);
		phi-= M_PI_4;
	}
	*ls_angle    = atan2(y, x);
	*ls_strength = sqrt(x*x + y*y);
}


const char* get_light_quadrant_name(float ls_angle){
	// Front at 90° (y axis) range from 112.5–67.5
	// 1. Normalize angle to range [0, 7] (8 quadrants)
	const float TwoPi = 2*M_PI;
	static const char quadrants[8][16] = {
		"left",
		"front-left",
		"front",
		"front-right",
		"right",
		"back-right",
		"back",
		"back-left",
	};
	// Map angle to quadrant
	ls_angle+= 0.3927;
	while(ls_angle > TwoPi) ls_angle-= TwoPi;
	uint8_t qi = 8 * (ls_angle / TwoPi);
	return quadrants[qi];
}


const char* get_obstacle_name(obstacle_t obs){
	switch(obs){
		case OBS_NONE:  return "none";
		case OBS_LEFT:  return "left";
		case OBS_RIGHT: return "right";
		case OBS_FRONT: return "front";
	}
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
	bool ol = ( data[0] > 0 ) && ( (data[0] * 0.5) <= OBS_DST );
	bool of = ( data[1] > 0 ) && ( (data[1] * 1.0) <= OBS_DST );
	bool or = ( data[2] > 0 ) && ( (data[2] * 0.5) <= OBS_DST );
	if( of || (ol && or) ) return OBS_FRONT;
	else if( ol ) return OBS_LEFT;
	else if( or ) return OBS_RIGHT;
	return OBS_NONE;
}


void avoid_obstacle(obstacle_t obs){
	switch(obs){
		case OBS_NONE: return;
		case OBS_LEFT:
			mv(ADVANCE, -M_PI_4);
		case OBS_RIGHT:
			mv(ADVANCE,  M_PI_4);
		case OBS_FRONT:
			mv(ADVANCE,  M_PI_2);
			mv(      0, -M_PI_2);
	}
}


void move_towards_light(float ls_angle){
	static const float M_TWO_PI = 2*M_PI;
	// Robot advances over y axis (90° means no rotation).
	// Thus angle must be shifted -90°
	// 1. Substract π/2 from angle
	float ra = ls_angle - M_PI_2;
	// 2. Whatever the angle is, normalize it to [0, 2π)
	while(ra < 0) ra += M_TWO_PI;
	while(ra >= M_TWO_PI) ra -= M_TWO_PI;
	// 3. Shift back to [-π, π] for the shortest turn
	if(ra > M_PI) ra -= M_TWO_PI;
	mv(0.1, ra);
}


void mv(float dist, float angle){
	printf("  mv %0.1f %0.1f\n", 100*dist, 360*angle/(2*M_PI));
	// rotate(angle);
	// move_y(dist);
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
	dist_ok  = dist_sens_init();
	printf("IR obstacle detection initialization: %s\n", dist_ok ? "OK" : "Err");
	if( !dist_ok ) exit(-1);
}


void ctrlc_handler(int signum){
	set_pwm(0, 0, 0, 0);
	stop();
	exit(0);
}

