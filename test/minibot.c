#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <signal.h>
#include <string.h>
#include <unistd.h> // sleep()
#include <ncurses.h>
#include <pthread.h>
#include <minibot/base.h>
#include <minibot/sensors.h>

WINDOW * win_top = NULL;
WINDOW * win_btm = NULL;
size_t win_btm_curr_row = 1;
bool shut_down = false;
uint8_t lidar_count = 0;
pthread_t asp_thread;
bool dist_ok  = false;
bool floor_ok = false;
bool lidar_ok = false;
bool light_ok = false;

void init_base(const char* serial_path);
void init_sensors(const char* i2c_bus_path);
void update_sensors();
void update_sen_info(int row, const char* caption, float* data, uint8_t dcount, uint8_t decimals);
void update_batt(int row, const char* caption, float vbat, float maxVbat);
void init_windows();
void destroy_windows();
void ctrlc_handler(int signum);
void start_aync_sensor_poll();
void* async_sensor_update_poll(void*);
void fetch_execute_command();
char* fetch_command();

int main(int argc, char** argv){
	const char* i2c_bus_path = (argc > 1) ? argv[1] : "/dev/i2c-1";
	const char* serial_path  = (argc > 2) ? argv[2] : "/dev/ttyUSB0";

	// init_sensors(i2c_bus_path);
	// init_base(serial_path);

	init_windows();
	start_aync_sensor_poll();

	wprintw(win_btm, "| mv dist rad | pwm l r f b |\n");
	while(!shut_down){
		fetch_execute_command();
	}

	destroy_windows();
	shut_down = true;
	pthread_join(asp_thread, NULL);
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
	// 1. Initialize I²C bus
	if( !i2c_init(i2c_bus_path) ) {
		fprintf(stderr, "Error %i opening I2C bus %s: %s\n", errno, i2c_bus_path, strerror(errno));
		exit(-1);
	}
	// 2. Enumerate I²C devices to find sensors
	find_sensors();

	// 3. Initialize sensors
	// 3.1. Initialize light source detector (turret) on 0x
	light_ok = light_sens_init();
	printf("Light sensor initialization: %s\n", light_ok ? "OK" : "Err");
	// 3.2. Initialize line detectors (floor) on 0x
	floor_ok = floor_sens_init();
	printf("Floor sensor initialization: %s\n", floor_ok ? "OK" : "Err");
	// 3.3. Initialize obstacle detectors (LIDAR) on 0x70
	dist_ok  = dist_sens_init();
	printf("IR obstacle detection initialization: %s\n", dist_ok ? "OK" : "Err");
	// 3.3. Initialize obstacle detectors (LIDAR) on 0x70
	lidar_ok = (lidar_count = lidar_sens_init()) > 0;
	printf("LIDAR sensor initialization: %s (%d detected)\n", lidar_ok ? "OK" : "Err", lidar_count);
}


void update_sen_info(int row, const char* caption, float* data, uint8_t dcount, uint8_t decimals){
	char fmt[16];
	char str[64];
	char* cc;

	sprintf(fmt, " %%0.%uf", decimals);
	sprintf(str, caption);
	cc = str + strlen(str);

	if(data && dcount > 0){
		for(uint8_t i = 0; i < dcount; ++i){
			sprintf(cc, fmt, data[i]);
			cc = str + strlen(str);
		}
	}
	else
		sprintf(cc, "Not connected");

	wmove(win_top, row, 0);
	wprintw(win_top, str);
}


void update_batt(int row, const char* caption, float vbat, float maxVbat){
	char str[64];
	sprintf(str, "Batt:  %0.2fV (%0.1f%)\n", vbat, 100.0*vbat/maxVbat);
	wmove(win_top, row, 0);
	wprintw(win_top, str);
}


void update_sensors(){
	float data[8];

	if (lidar_ok){
		lidar_sens_read(data);
		update_sen_info(0, "LiDAR:", data, lidar_count, 3);
	}
	else
		update_sen_info(0, "LiDAR:", NULL, 0, 3);

	if (dist_ok){
		dist_sens_read(data);
		update_sen_info(1, "Dist: ", data, 6, 4);
	}
	else
		update_sen_info(1, "Dist: ", NULL, 0, 4);

	if (light_ok){
		light_sens_read(data);
		update_sen_info(2, "Light:", data, 8, 4);
	}
	else
		update_sen_info(2, "Light:", NULL, 0, 4);

	if (floor_ok){
		floor_sens_read(data);
		update_sen_info(3, "Floor:", data, 4, 4);
	}
	else
		update_sen_info(3, "Floor:", NULL, 0, 4);

	float vbat = 7.2;
	// float vbat = read_batt_volt();
	update_batt(4, "Batt", vbat, 7.2f);

	wrefresh(win_top);
	wrefresh(win_btm);
}


void init_windows(){
	signal(SIGINT, ctrlc_handler);
	setlocale(LC_ALL, "");
	initscr();            // Start curses mode

	int rows, cols;
	getmaxyx(stdscr, rows, cols);
	win_top = newwin(     6, cols,      0, 0); // <- h, w, y, x
	win_btm = newwin(rows-6, cols,      6, 0);
	// scrollok(mid, true);
	noecho();
	refresh(); // Print it on to the real screen
}


void destroy_windows(){
	if(win_top) delwin(win_top);
	if(win_btm) delwin(win_btm);
	endwin();
}


void start_aync_sensor_poll(){
	int res = pthread_create(&asp_thread, NULL, async_sensor_update_poll, NULL);
	if( res ){
		fprintf(stderr, "Error %i creating async thread for sensor polling: %s\n", errno, strerror(errno));
		destroy_windows();
		exit(-1);
	}
}


char* fetch_command(){
	static size_t bufsize = 256;
	uint32_t      c;
	char*  buffer  = (char*)malloc(bufsize);
	size_t ix = 0;

	wtimeout(win_btm, 250);
	wmove(win_btm, win_btm_curr_row, 0);
	wprintw(win_btm, "> ");
	while(!shut_down){
		if ((c = wgetch(win_btm)) == ERR) continue;
		switch(c){
			case KEY_ENTER: case '\n': case '\r':
			// save_prev_input();
			if(ix == 0) break;
			++win_btm_curr_row;
			return buffer;
			break;

			case KEY_BACKSPACE: case 127:
				if(!ix) break;
				buffer[--ix] = 0;
				wmove(win_btm, win_btm_curr_row, 2 + ix);
				waddch(win_btm, ' ');
				wmove(win_btm, win_btm_curr_row, 2 + ix);
				break;
		}

		if( (c >= 0x20) && (c < 0x7f) ){
			buffer[ix] = c;
			waddch(win_btm, c);
			ix = (ix+1) % bufsize;
			if(ix >= bufsize) buffer = realloc(buffer, bufsize*=2);
			buffer[ix] = 0;
		}
	}

}


void fetch_execute_command(){
	char* buffer = fetch_command();
	if ( !strcmp(buffer, "q") || !strcmp(buffer, "quit") || !strcmp(buffer, "exit")){
		ctrlc_handler(0);
		return;
	}
	else if( (strlen(buffer) > 5) && (buffer[0] == 'm') && (buffer[1] == 'v') ){
		float dist, angle;
		sscanf(buffer, "mv %f %f", &dist, &angle);
		// rotate(angle);
		// move_y(dist);
		wprintw(win_btm, " OK!\n");
	}
	else if( (strlen(buffer) > 10) && (buffer[0] == 'p') && (buffer[1] == 'w') && (buffer[2] == 'm') ){
		float l, r, f, b;
		sscanf(buffer, "pwm %f %f %f %f", &l, &r, &f, &b);
		// set_pwm(l, r, f, b);
		sleep(1);
		// set_pwm(0, 0, 0, 0);
		wprintw(win_btm, " OK!\n");
	}
	else
		wprintw(win_btm, " REJECTED\n");


	free(buffer);
}


void* async_sensor_update_poll(void *){
	while(!shut_down){
		update_sensors();
		usleep(100000);
	}
}


void ctrlc_handler(int signum){
	shut_down = true;
	destroy_windows();
	endwin();
	pthread_join(asp_thread, NULL);
	exit(0);
}
