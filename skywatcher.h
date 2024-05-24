#pragma once

#include <socket/socket.h>
#include <stdbool.h>
#include <stdint.h>

enum skywatcher_axis
{
    SA_AXIS_RA_AZ_1 = 1,
    SA_AXIS_DEC_ALT_2 = 2,
    SA_BOTH = 3
};

enum skywatcher_mode
{
    SM_GOTO = 0,
    SM_TRACKING = 1
};

enum skywatcher_direction
{
    SD_CW = 0,
    SD_CCW = 1
};

enum skywatcher_speed_mode
{
    SSM_SLOW = 0,
    SSM_FAST = 1
};

typedef struct skywatcher_axis_status
{
    enum skywatcher_mode mode;
    enum skywatcher_direction direction;
    enum skywatcher_speed_mode speed;
} *skywatcher_axis_status_t;

bool skywatcher_initialize();
void skywatcher_uninitialize();
bool skywatcher_instant_stop(enum skywatcher_axis axis);
bool skywatcher_start_motion(enum skywatcher_axis axis);
bool skywatcher_get_timer_frequency(uint32_t* frequency);
bool skywatcher_get_axis_resolution(uint32_t* step_count, enum skywatcher_axis axis);
bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis);
bool skywatcher_set_motion_mode_tracking(enum skywatcher_axis axis, enum skywatcher_direction direction);
