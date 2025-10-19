#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


bool i2c_init(const char* i2c_bus_path);
void find_sensors();
bool light_sens_init();
bool floor_sens_init();
bool dist_sens_init();
uint8_t lidar_sens_init();

bool floor_sens_read(float data[4]);
bool light_sens_read(float data[8]);
bool dist_sens_read(float data[6]);
bool dist_sens_readu(uint8_t data[6]);

bool lidar_sens_read(float data[8]);
bool lidar_sens_readu(uint8_t data[8]);
uint8_t lidar_sens_count();
