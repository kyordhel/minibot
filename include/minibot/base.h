#pragma once

#include <stdint.h>
#include <stdbool.h>

// Base PWM to use for mv commands
#ifndef BASE_PWM
#define BASE_PWM   0.6
#endif

// Base SPEED to use for mv commands
#ifndef BASE_SPEED
#define BASE_SPEED 1.0
#endif

struct{
	int32_t front;
	int32_t back;
	int32_t left;
	int32_t right;
} typedef encoders;

bool serial_init(const char* serial_path);
bool detect_mc();

void stop();
float read_batt_volt();
bool read_encoders(encoders* e);
float rotate(float angle);
float move_y(float angle);
void set_pwm(float left, float right, float front, float back);
void set_speed(float left, float right, float front, float back);

inline void set_pwm2(float left, float right){ set_pwm(left, right, 0, 0); }
inline void set_speed2(float left, float right){ set_speed(left, right, 0, 0); }
