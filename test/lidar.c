#include <stdio.h>
#include <unistd.h> // sleep()
#include <minibot/sensors.h>

int main(int argc, char const **argv){
	const char* i2c_bus_path = (argc > 1) ? argv[1] : "/dev/i2c-1";
	printf("Testing minibot LIDAR distance sensors for obstacle avoidance.\n");

	// 1. Initialize I²C bus
	if( !i2c_init(i2c_bus_path) ) return -1;
	// 2. Enumerate I²C devices to find sensors
	find_sensors();

	// 3. Initialize obstacle detectors (LIDAR) on 0x70
	uint8_t lidar_count = lidar_sens_init();

	// 4. Poll
	float data[8];

	printf("\n");
	while(true){
		printf("\rLidar:");
		lidar_sens_read(data);
		for(uint8_t i = 0; i < lidar_count; ++i) printf(" %0.1f", 100*data[i]);
		fflush(stdout);
		usleep(100000);
	}
	return 0;
}



