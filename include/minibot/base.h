#pragma once

#include <stdint.h>
#include <stdbool.h>

// Base PWM to use for mv commands
#ifndef BASE_PWM
#define BASE_PWM   0.72
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


/**
 * Initializes the robot's driver board connected to the specified
 * serial port device.
 * @param  serial_path Device path of the serial port (/dev/ttyUSB0)
 * @return             true if the serial port was correctly
 *                     initialized; false otherwise
 */
bool init_mc(const char* serial_path);

/**
 * Disconnects from the motor board and closes the serial port
 */
void disconnect_mc();

/**
 * Stops the robot by setting all PWMs to zero
 */
void stop();

/**
 * Reads the battery charge in volts
 */
float read_batt_volt();

/**
 * Reads the absolute count value of all encoders
 * @param  e A pointer to a encoders structure
 * @return   true if encoders were successfully read and e contains
 *           valid information; false otherwise
 */
bool read_encoders_abs(encoders* e);

/**
 * Reads the encoder pulse count value of all encoders over the last 10ms
 * @param  e A pointer to a encoders structure
 * @return   true if encoders were successfully read and e contains
 *           valid information; false otherwise
 */
bool read_encoders_dt(encoders* e);

/**
 * Rotates the robot
 * @param  angle The desired rotation angle, in radians
 * @return       The real rotation angle estimated with base on
 *               the value of the encoders
 */
float rotate(float angle);

/**
 * Moves the robot along the y axis (front-back)
 * @param  distance  The desired distance to move, in meters
 * @return           The real rotation angle estimated with base
 *                   on the value of the encoders
 */
float move_y(float distance);

/**
 * Sets the PWM duty cycle for all four wheels.
 * Values are NORMALIZED in the interval [-1, 1]
 */
void set_pwm(float left, float right, float front, float back);

/**
 * Retrieves the latest normalized PWM duty cycle set on all the
 * four wheels.
 * @remark    Return zeroes when using speed commands.
 */
void get_pwm(float* left, float* right, float* front, float* back);

/**
 * Sets the speed in radians per second for all four wheels
 * The speed is maintained using PWM an internal PWM
*/
void set_speed(float left, float right, float front, float back);

/**
 * Sets the speed for all four wheels using the driver board's
 * speed controller. Values are NORMALIZED in the interval [-1, 1]
 */
void set_board_speed(float left, float right, float front, float back);

/**
 * Retrieves the latest speed values for all the four wheels in rad/s.
 * @remark    Return zeroes when using PWM or board SPD commands.
 */
void get_speed(float* left, float* right, float* front, float* back);

/**
 * Retrieves the latest normalized speed values sent to the driver
 * board's speed controllet for all the four wheels.
 * @remark    Return zeroes when using PWM or board SPD commands.
 */
void get_board_speed(float* left, float* right, float* front, float* back);

inline void set_pwm2(float left, float right){ set_pwm(left, right, 0, 0); }
inline void set_speed2(float left, float right){ set_speed(left, right, 0, 0); }
inline void set_board_speed2(float left, float right){ set_board_speed(left, right, 0, 0); }

/**
 * Initializes the serial port used to communicate with the robot's
 * driver board
 * @param  serial_path Device path of the serial port (/dev/ttyUSB0)
 * @return             true if the serial port was correctly
 *                     initialized; false otherwise
 */
bool serial_init(const char* serial_path);

/**
 * Detects the presence of the robot's in the initialized serial port
 * @return             true if a compatible driver board is present;
 *                     false otherwise
 */
bool detect_mc();