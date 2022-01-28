
#pragma once

#include <Arduino.h>
#include <TinyGPS++.h>

extern TinyGPSPlus tGPS;

void gps_loop(boolean print_it);
void gps_setup(void);
void gps_time(char *buffer, uint8_t size);
void gps_passthrough(void);
void gps_end(void);
