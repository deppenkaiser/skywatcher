#pragma once

#include <socket/socket.h>
#include <stdbool.h>
#include <stdint.h>

#include "types.h"

bool skywatcher_initialize(const char* ip);
void skywatcher_uninitialize();
bool skywatcher_instant_stop(enum skywatcher_axis axis);
bool skywatcher_start_motion(enum skywatcher_axis axis);
bool skywatcher_get_timer_frequency(uint32_t* frequency);
bool skywatcher_get_axis_resolution(uint32_t* step_count, enum skywatcher_axis axis);
bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis);
bool skywatcher_set_motion_mode_tracking(enum skywatcher_axis axis, enum skywatcher_direction direction);
void skywatcher_print_mode(char* buffer, size_t buffer_size, enum skywatcher_mode mode);
void skywatcher_print_direction(char* buffer, size_t buffer_size, enum skywatcher_direction direction);
void skywatcher_print_speed_mode(char* buffer, size_t buffer_size, enum skywatcher_speed_mode speed_mode);
void skywatcher_handle_arrow_keys(uint32_t key_value);
