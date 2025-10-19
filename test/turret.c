#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <unistd.h> // sleep()
#include <minibot/sensors.h>

int main(int argc, char const **argv){
	const char* i2c_bus_path = (argc > 1) ? argv[1] : "/dev/i2c-1";
	printf("Testing minibot light detection turret.\n");

	// 1. Initialize I²C bus
	if( !i2c_init(i2c_bus_path) ) return -1;
	// 2. Enumerate I²C devices to find sensors
	find_sensors();

	// 3. Initialize light source detector (turret) on 0x
	light_sens_init();

	// 4. Poll
	float data[8];
	float x, y;

	printf("\n");
	while(true){
		x = y = 0;
		printf("\rLight:");
		light_sens_read(data);
		for(uint8_t i = 0; i < 8; ++i) {
			printf(" %0.4f", data[i]);
			x+= data[i] * cos(i * 2.0 * M_PI / 8.0 );
			y+= data[i] * sin(i * 2.0 * M_PI / 8.0 );
		}
		printf("| Dir: %0.4f", atan2(y, x));
		fflush(stdout);
		usleep(100000);
	}
	return 0;
}
