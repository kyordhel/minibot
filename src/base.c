#include "base.h"

// C library headers
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Linux headers
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>

const float KP = 0.0030;
const float KI = 0.00002; //0.00025;
const float KD = 0.0030; //0.000125;

/* ** *****************************************************************
* Types and structures
** ** ****************************************************************/
struct{
	float front;
	float back;
	float left;
	float right;
} typedef spds_t;
typedef spds_t pwms_t;

struct{
	char  mv[8]; // Motor version
	uint8_t  mt; // Motor type
	uint16_t dz; // Dead zone
	uint8_t  pl; // Pulse line
	uint8_t  pp; // Pulse phase
	float    wd; // Wheel diameter
	float    kp; // PID proportional constant
	float    ki; // PID integral constant
	float    kd; // PID derivative constant
}typedef flash_data;

/* ** *****************************************************************
* Global variables
** ** ****************************************************************/
static int serial = 0;
static bool __abort_move = true;
flash_data fd;
static spds_t current_spd;
static spds_t currbrd_spd;
static pwms_t current_pwm;


/* ** *****************************************************************
* Prototypes
** ** ****************************************************************/
bool read_flash();
bool serial_writeline(const char* str);
bool serial_readline(char* str, size_t max);

/* ** *****************************************************************
* Prototypes (helpers)
** ** ****************************************************************/
static inline void clamp(float*value, float min, float max);
static inline void clamp2one(float*value);
static inline void enc_acc(encoders *e, encoders other);
static inline encoders enc_add(encoders e1, encoders e0);
static inline encoders enc_diff(encoders e1, encoders e0);
static inline int32_t enc_avg_diff(encoders e1, encoders e0);
static inline int32_t enc_avg_diff(encoders e1, encoders e0);
static inline void update_current_pwm_values(float l, float r, float f, float b);
static inline void update_current_speed_values(float l, float r, float f, float b);
static inline void update_current_board_speed_values(float l, float r, float f, float b);


/* ** *****************************************************************
* Function definitions
** ** ****************************************************************/
bool init_mc(const char* serial_path){
	if( !serial_init(serial_path) || !detect_mc() )
		return false;
	stop();
	return true;
}


bool serial_init(const char* serial_path){
	serial = open(serial_path, O_RDWR | O_NOCTTY);
	// Check path exists and we have access to it
	if (serial < 0) {
		fprintf(stderr, "Error %i from open: %s\n", errno, strerror(errno));
		return false;
	}
	// Flush buffers
	for(uint8_t i = 0; i < 10; ++i){
		tcflush(serial, TCIOFLUSH);
		usleep(1000);
	}
	// Check open path is a serial port
	struct termios tty;
	if(tcgetattr(serial, &tty)) {
		fprintf(stderr, "Error %i reading serial port attributes: %s\n", errno, strerror(errno));
		close(serial); serial = 0;
		return false;
	}
	// Clear settings:
	tty.c_cflag &= ~CSIZE;   // Clear word size
	tty.c_cflag &= ~CRTSCTS; // Disable control flow
	tty.c_lflag &= ~ICANON;  // Disable cannonical mode (read bytes, not lines)
	                         // Disable echo
	tty.c_lflag &= ~(ECHO | ECHOE | ECHONL);
	                         // Disable software control flow
	tty.c_iflag &= ~(IXON | IXOFF | IXANY );
	                         // Disable interpret of Control Characters
	// tty.c_iflag &= ~(INLCR|IGNCR|ICRNL|IXON|IXOFF|ONLCR|OCRNL|ISIG|IEXTEN);
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
	tty.c_cflag |=  CLOCAL;  // Disable modem signals
	tty.c_cflag |=  CREAD;   // Allow reading

	// Set N81 for TTY
	tty.c_cflag &= ~PARENB;  // N: Clear parity bit
	tty.c_cflag |=  CS8;     // 8: Set 8 bits per word
	tty.c_cflag &= ~CSTOPB;  // 1: 1 STOP bits

	// tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	// tty.c_oflag &= ~(ONLCR | OCRNL);
	// tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_cc[VTIME] = 1;    // read() waits for up to 100ms.
	tty.c_cc[VMIN]  = 0;    // read() returns when ANY data is available.

	// Set baudrate: B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);
	if(tcsetattr(serial, TCSANOW, &tty)) {
		fprintf(stderr, "Error %i setting serial port attributes: %s\n", errno, strerror(errno));
		close(serial); serial = 0;
		return false;
	}
	return true;
}


bool serial_writeline(const char* str){
	static char nl[2] = "\n";
	ssize_t len = strlen(str);
	ssize_t written = write(serial, str, len);
	if(written != len) return false;
	write(serial, nl, 1);

	return true;
}


bool serial_readline(char* buffer, size_t max){
	ssize_t cnt;
	char* cc = buffer;
	*cc = 0;
	while((cc - buffer) < max){
		cnt = read(serial, cc, 1);
		if ((*cc == '\n') || (cnt < 1)) break;
		++cc;
	}
	*cc = 0;
	return cnt >= 0;
}


bool detect_mc(){
	return read_flash();
}


bool read_flash(){
	char buffer[64];
	serial_writeline("$read_flash#");
	usleep(10000); // Wait for board to respond
	serial_readline(buffer, sizeof(buffer));
	if( !strcmp(buffer, "$read_flash:OK!") ) return false;
	serial_readline(buffer, sizeof(buffer));

	if( !strcmp(buffer, "Motor_Version:1.7.3") ) return false;
	sscanf(buffer, "Motor_Version:%s", fd.mv);
	// printf("Version: %s\n", fd.mv);

	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "Motor_type:%d", &fd.mt);
	// printf("%s\n", buffer);

	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "Dead_Zone:%d", &fd.dz);
	// printf("%s\n", buffer);

	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "Pulse_Line:%d", &fd.pl);

	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "Pulse_Phase:%d", &fd.pp);
	// printf("%s\n", buffer);

	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "wheel_diameter:%d", &fd.wd);
	// printf("Wheel diameter: %0.1f\n", fd.wd/10.0f);

	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "P:%f\tI:%f\tD:%f\t", &fd.kp, &fd.ki, &fd.kd);
	// printf("%s\n", buffer);
	return true;
}


float read_batt_volt(){
	float batt;
	static char buffer[16];
	serial_writeline("$read_vol#");
	serial_readline(buffer, sizeof(buffer));
	sscanf(buffer, "$Battery:%fV#", &batt);
	return batt;
}


float read_batt_perc(){
	float batt = read_batt_volt();
	if(batt < 4.5) return 0;
	else if(batt > 7.2) return 100;
	return (batt - 4.5) / 0.027;
}


bool read_encoders_abs(encoders* e){
	static char buffer[64];
	if(!e) return false;
	serial_writeline("$upload:1,0,0#$upload:0,0,0#");
	if( !serial_readline(buffer, sizeof(buffer)) ){
		e->left = e->front = e->back = e->right = 0;
		return false;
	}
	sscanf(buffer, "$MAll:%d,%d,%d,%d#", &e->left, &e->front, &e->back, &e->right);
	// printf("buffer: %s\n", buffer);
	return true;
}


bool read_encoders_dt(encoders* e){
	static char buffer[64];
	if(!e) return false;
	serial_writeline("$upload:0,1,0#$upload:0,0,0#");
	if( !serial_readline(buffer, sizeof(buffer)) ){
		e->left = e->front = e->back = e->right = 0;
		return false;
	}
	sscanf(buffer, "$MTEP:%d,%d,%d,%d#", &e->left, &e->front, &e->back, &e->right);
	// printf("buffer: %s\n", buffer);
	return true;
}


void stop(){
	__abort_move = true;
	set_pwm(0, 0, 0, 0);
}


void set_pwm(float left, float right, float front, float back){
	char buffer[32];
	clamp2one(&left);	clamp2one(&right);
	clamp2one(&front);	clamp2one(&back);
	update_current_pwm_values(left, right, front, back);
	int16_t l = 3600 * left;
	int16_t r = 3600 * right;
	int16_t f = 3600 * front;
	int16_t b = 3600 * back;
	sprintf(buffer, "$pwm:%d,%d,%d,%d#", l, f, b, r);
	serial_writeline(buffer);
}


void get_pwm(float* left, float* right, float* front, float* back){
	*left  = current_pwm.left;     *right = current_pwm.right;
	*front = current_pwm.front;    *back  = current_pwm.back;
}


void set_board_speed(float left, float right, float front, float back){
	char buffer[32];
	clamp2one(&left);	clamp2one(&right);
	clamp2one(&front);	clamp2one(&back);
	update_current_board_speed_values(left, right, front, back);
	int16_t l = 1000 * left;
	int16_t r = 1000 * right;
	int16_t f = 1000 * front;
	int16_t b = 1000 * back;
	sprintf(buffer, "$spd:%d,%d,%d,%d#", l, f, b, r);
	serial_writeline(buffer);
}


void get_board_speed(float* left, float* right, float* front, float* back){
	*left  = current_spd.left;     *right = current_spd.right;
	*front = current_spd.front;    *back  = current_spd.back;
}

/*
float move_y(float dist){
	// 0.1m → ~926 encoder pulses
	encoders ei, ef, diff;
	float curr_dist = 0;
	int16_t sgn = dist < 0 ? -1.0 : 1.0;

	stop();
	if(dist == 0) return 0;
	read_encoders_abs(&ei);
	set_pwm(sgn * 1.1 * BASE_PWM, sgn * 1.1 * BASE_PWM, 0, 0);
	do{
		usleep(10000);
		// if(angle > 0) set_speed2(-base_speed,  base_speed, -base_speed,  base_speed);
		// else          set_speed2( base_speed, -base_speed,  base_speed, -base_speed);
		if(!read_encoders_abs(&ef)){
			fprintf(stderr, "Error reading encoders.\n");
			break;
		}
		diff = enc_diff(ef, ei);
		curr_dist = 0.1 * (diff.right + diff.left) / (2.0 * 1800); //168.11;
		// printf("Avg: %d, Dist: %0.2f, Cur: %0.2f, Err:%0.2f, AErr: %0.2f\n",
		// 	(diff.right + diff.left) /2,
		// 	dist, curr_dist, dist - curr_dist, fabsf(dist - curr_dist));
	}while( fabsf(dist - curr_dist) > 0.03 );

	stop(); // Stop motors after turn
	usleep(4000);// Wait for command to arrive
	return curr_dist;
}
*/
float move_y(float dist){
	// 0.1m → ~926 encoder pulses
	encoders diff, e0, ei, ef, err, errI, errD, err_;
	float curr_dist = 0;
	float pwml, pwmr;
	int32_t est_steps = dist * 9260; // 926/0.1

	stop();
	if(dist == 0) return 0;
	__abort_move = false;
	read_encoders_abs(&e0);
	ef = (encoders){ .left = e0.left + est_steps, .right = e0.right + est_steps, .front = e0.front, .back = e0.back };
	err = err_ = errI = errD = (encoders){0, 0, 0, 0};

	do{
		if(!read_encoders_abs(&ei))	break;
		err_ = err;
		err = enc_diff(ef, ei);
	    enc_acc(&errI, err);
	    errD = enc_diff(err, err_);
		pwml = KP * err.left  + KI * errI.left  + KD * errD.left;
		pwmr = KP * err.right + KI * errI.right + KD * errD.right;

		// printf("pwml = KP * %d + KI * %d + KD * %d = %0.3f\n", err.left, errI.left, errD.left, pwml);
		// printf("pwmr = KP * %d + KI * %d + KD * %d = %0.3f\n", err.right, errI.right , errD.right, pwmr);
		if( __abort_move ) break;
		set_pwm(pwml, pwmr, 0, 0);
		usleep(10000);
	}while( (abs(err.right + err.left) / 2) > 200 ); // About 2cm

	read_encoders_abs(&ei);
	diff = enc_diff(ei, e0);
	curr_dist = (diff.right + diff.left) / (2 * 9260.0);
	stop(); // Stop motors after turn
	usleep(4000);// Wait for command to arrive
	return curr_dist;
}

/*
float rotate(float angle){
	// 360° → ~4722–4816 encoder pulses
	encoders ei, ef, diff;
	int32_t w_avg;
	float curr_ang = 0;

	stop();
	if(angle == 0) return 0;
	read_encoders_abs(&ei);
	if(angle > 0) set_pwm(-BASE_PWM,  BASE_PWM, -BASE_PWM,  BASE_PWM);
	else          set_pwm( BASE_PWM, -BASE_PWM,  BASE_PWM, -BASE_PWM);
	do{
		usleep(10000);
		// if(angle > 0) set_speed(-base_speed,  base_speed, -base_speed,  base_speed);
		// else          set_speed( base_speed, -base_speed,  base_speed, -base_speed);
		if(!read_encoders_abs(&ef)){
			fprintf(stderr, "Error reading encoders.\n");
			break;
		}
		diff = enc_diff(ef, ei);
		int32_t w_avg = (diff.right - diff.left + diff.back - diff.front) / 4;
		curr_ang = w_avg / 16.11;
		// printf("WAvg: %d, Ang: %0.2f, Cur: %0.2f, Err: %0.2f\n", w_avg, angle, curr_ang, fabsf(angle - curr_ang));
	}while( fabsf(angle - curr_ang) > 0.3 );

	stop(); // Stop motors after turn
	usleep(4000);// Wait for command to arrive
	return curr_ang;
}
*/
float rotate(float angle){
	// 360° → ~4722–4816 encoder pulses
	encoders diff, e0, ei, ef, err, errI, errD, err_;
	float curr_ang = 0;
	float pwml, pwmr, pwmf, pwmb;
	int32_t est_steps = angle * 751.5; // 4722 / 2π

	stop();
	// if(angle == 0) return 0;
	if( fabsf(angle) < 18) return 0; // Ignores angles smaller than incertitude.
	__abort_move = false;
	read_encoders_abs(&e0);
	ef = (encoders){ .left  = e0.left - est_steps,  .right = e0.right + est_steps,
	                 .front = e0.front - est_steps, .back  = e0.back + est_steps   };
	err = err_ = errI = errD = (encoders){0, 0, 0, 0};

	do{
		if(!read_encoders_abs(&ei))	break;
		err_ = err;
		err = enc_diff(ef, ei);
	    enc_acc(&errI, err);
	    errD = enc_diff(err, err_);
		pwmf = 0.8 * KP * err.front + 0.2 * KI * errI.front + KD * errD.front;
		pwmb = 0.8 * KP * err.back  + 0.2 * KI * errI.back  + KD * errD.back;
		pwml = 0.8 * KP * err.left  + 0.2 * KI * errI.left  + KD * errD.left;
		pwmr = 0.8 * KP * err.right + 0.2 * KI * errI.right + KD * errD.right;

		// diff = enc_diff(ei, e0);
		// curr_ang = (diff.front - diff.back - diff.right + diff.left) / (4 * 766.5);
		// printf("pwml = KP * %d + KI * %d + KD * %d = %0.3f\n", err.left, errI.left, errD.left, pwml);
		// printf("pwmr = KP * %d + KI * %d + KD * %d = %0.3f\n | %+0.3f", err.right, errI.right , errD.right, pwmr, curr_ang);
		if( __abort_move ) break;
		set_pwm(pwml, pwmr, pwmf, pwmb);
		usleep(10000);
	// }while( abs((err.front - err.back - err.right + err.left) / 4) > 100 ); // About 2cm or 7.5°
	}while( fabsf(curr_ang - angle) > 0.09 ); // About 5–7°

	read_encoders_abs(&ei);
	diff = enc_diff(ei, e0);
	curr_ang = (diff.front - diff.back - diff.right + diff.left) / (4 * 751.5);
	stop(); // Stop motors after turn
	usleep(4000);// Wait for command to arrive
	return curr_ang;
}


void disconnect_mc(){
	if(!serial) return;
	stop();
	close(serial);
	serial = 0;
}

/* ** *****************************************************************
* Function Definitions (helpers)
** ** ****************************************************************/

static inline
encoders enc_diff(encoders e1, encoders e0){
	encoders diff;
	diff.front = e1.front - e0.front;
	diff.back  = e1.back  - e0.back;
	diff.left  = e1.left  - e0.left;
	diff.right = e1.right - e0.right;
	return diff;
}

static inline
void enc_acc(encoders *e, encoders other){
	e->front+= other.front;
	e->back += other.back;
	e->left += other.left;
	e->right+= other.right;
}

static inline
encoders enc_add(encoders e1, encoders e0){
	encoders sum;
	sum.front = e1.front + e0.front;
	sum.back  = e1.back  + e0.back;
	sum.left  = e1.left  + e0.left;
	sum.right = e1.right + e0.right;
	return sum;
}

static inline
int32_t enc_avg_diff(encoders e1, encoders e0){
	return (
		abs(e1.front - e0.front) +
		abs(e1.back  - e0.back ) +
		abs(e1.left  - e0.left ) +
		abs(e1.right - e0.right)
	) / 4;
}

static inline
void clamp(float*value, float min, float max){
	if(*value < min) *value = min;
	if(*value > max) *value = max;
}


static inline
void clamp2one(float*value){
	if(*value < -1) *value = -1;
	if(*value >  1) *value =  1;
}


static inline
void update_current_pwm_values(float l, float r, float f, float b){
	current_spd.left  = 0;    current_spd.front = 0;
	current_spd.back  = 0;    current_spd.right = 0;
	currbrd_spd.left  = 0;    currbrd_spd.front = 0;
	currbrd_spd.back  = 0;    currbrd_spd.right = 0;
	current_pwm.left  = l;    current_pwm.front = f;
	current_pwm.back  = b;    current_pwm.right = r;
}


static inline
void update_current_speed_values(float l, float r, float f, float b){
	current_pwm.left  = 0;    current_pwm.front = 0;
	current_pwm.back  = 0;    current_pwm.right = 0;
	currbrd_spd.left  = 0;    currbrd_spd.front = 0;
	currbrd_spd.back  = 0;    currbrd_spd.right = 0;
	current_spd.left  = l;    current_spd.front = f;
	current_spd.back  = b;    current_spd.right = r;
}


static inline
void update_current_board_speed_values(float l, float r, float f, float b){
	current_pwm.left  = 0;    current_pwm.front = 0;
	current_pwm.back  = 0;    current_pwm.right = 0;
	currbrd_spd.left  = l;    currbrd_spd.front = f;
	currbrd_spd.back  = b;    currbrd_spd.right = r;
	current_spd.left  = 0;    current_spd.front = 0;
	current_spd.back  = 0;    current_spd.right = 0;
}
/*
Encoders: 78-80 pulses / 10ms (max speed, pwm=3600)
Encoders: 30-35 pulses / 10ms (max speed, spd=1000)
 */
