#pragma once

#include <socket/socket.h>
#include <stdbool.h>
#include <stdint.h>

#include "types.h"

bool skywatcher_open(const char* ip);
void skywatcher_close();
bool skywatcher_initialize_axis();
bool skywatcher_start_motion(enum skywatcher_axis axis);
bool skywatcher_stop_motion(enum skywatcher_axis axis);
bool skywatcher_instant_stop(enum skywatcher_axis axis);
bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis);
bool skywatcher_get_position(enum skywatcher_axis axis, int32_t* position);
bool skywatcher_set_position(enum skywatcher_axis axis, int32_t position);
bool skywatcher_get_axis_position(enum skywatcher_axis axis, int32_t* position);
bool skywatcher_get_speed(enum skywatcher_axis axis, double* angular_speed_degrees_per_s);
bool skywatcher_set_speed(enum skywatcher_axis axis, double angular_speed_degrees_per_s);
bool skywatcher_set_motion_mode(enum skywatcher_axis axis, bool tracking, bool fast, bool ccw, bool south);
bool skywatcher_set_axis_sleep(enum skywatcher_axis axis, bool sleep);
