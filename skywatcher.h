#pragma once

#include <socket/socket.h>
#include <stdbool.h>
#include <stdint.h>

#include "types.h"

bool skywatcher_open(const char* ip);
void skywatcher_close();
bool skywatcher_initialize_axis(double pixel_size_um, double focal_length_mm);
bool skywatcher_start_motion(enum skywatcher_axis axis);
bool skywatcher_stop_motion(enum skywatcher_axis axis);
bool skywatcher_instant_stop(enum skywatcher_axis axis);
bool skywatcher_get_axis_status(enum skywatcher_axis axis, skywatcher_axis_status_t status);
bool skywatcher_get_position(enum skywatcher_axis axis, int32_t* position);
bool skywatcher_set_position(enum skywatcher_axis axis, int32_t position);
bool skywatcher_get_axis_position(enum skywatcher_axis axis, int32_t* position);
bool skywatcher_get_speed(enum skywatcher_axis axis, double* angular_speed_degrees_per_s);
bool skywatcher_set_speed(enum skywatcher_axis axis, double angular_speed_degrees_per_s);
bool skywatcher_set_motion_mode(enum skywatcher_axis axis, bool tracking, bool fast, bool ccw, bool south);
bool skywatcher_set_axis_sleep(enum skywatcher_axis axis, bool sleep);
bool skywatcher_get_goto_target(enum skywatcher_axis axis, int32_t* target);
bool skywatcher_set_goto_target(enum skywatcher_axis axis, int32_t target);
bool skywatcher_set_aux(enum skywatcher_axis axis, bool on);
bool skywatcher_set_siderial_speed();

