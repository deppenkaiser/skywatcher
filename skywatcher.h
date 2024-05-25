#pragma once

#include <socket/socket.h>
#include <stdbool.h>
#include <stdint.h>

#include "types.h"

bool skywatcher_open(const char* ip);
void skywatcher_close();
bool skywatcher_initialize_axis();
bool skywatcher_instant_stop(enum skywatcher_axis axis);
bool skywatcher_start_motion(enum skywatcher_axis axis);
bool skywatcher_stop_motion(enum skywatcher_axis axis);
bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis);
bool skywatcher_set_motion_mode_tracking(enum skywatcher_axis axis, enum skywatcher_direction direction);
bool skywatcher_set_motion_mode_tracking_slow(enum skywatcher_axis axis, enum skywatcher_direction direction);
bool skywatcher_set_auto_guide_speed(enum skywatcher_axis axis, enum skywatcher_auto_guide_speed speed);
void skywatcher_handle_arrow_keys(uint32_t key_value);
