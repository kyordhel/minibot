#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // sleep()
#include <minibot/base.h>

int main(int argc, char const **argv){
	const char* serial_path = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
	printf("Testing minibot mobile base.\n");

	// 1. Initialize serial port
	if( !serial_init(serial_path) ) return -1;
	printf("Serial port %s open\n", serial_path);

	// 2. Check motor controller board
	if( !detect_mc() ){
		fprintf(stderr, "Controller board not found\n");
		return -1;
	}

	float vbat = read_batt_volt();
	printf("Battery level: %0.2fV (%0.1f%)\n", vbat, 100.0*(vbat-4.5)/2.7);

	float res;
	encoders e;
	read_encoders_abs(&e);

	printf("Robot will advance 0.1m\n");
	res = move_y(0.1);
	printf("Robot moved %0.3fm\n\n", res);
	sleep(2);
	printf("Robot will reverse 0.1m\n");
	res = move_y(-0.1);
	printf("Robot moved %0.3fm\n", res);
	sleep(2);
	printf("\n");

	printf("Robot will turn to the left\n");
	res = rotate(1.5708);
	printf("Robot turned %0.1f°\n\n", res * 57.3);
	sleep(2);
	printf("Robot will turn to the right\n");
	res = rotate(-3.1416);
	printf("Robot turned %0.1f°\n\n", res * 57.3);
	sleep(2);
	printf("Robot will turn to the left\n");
	res = rotate(1.5708);
	printf("Robot turned %0.1f°\n\n", res * 57.3);
}
