#pragma once

#include <stdint.h>

enum skywatcher_axis
{
    SA_NONE = 0,
    SA_AXIS_1 = 1,
    SA_AXIS_2 = 2
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

enum skywatcher_axis_action
{
    SAA_STOPPED = 0,
    SAA_RUNNING = 1
};

enum skywatcher_axis_state
{
    SAS_NORMAL = 0,
    SAS_BLOCKED = 1
};

enum skywatcher_init_state
{
    SIS_NOT_INIT = 0,
    SIS_DONE = 1
};

enum skywatcher_auto_guide_speed
{
    AGS_NORMAL = 0,
    AGS_SLOWER = 1,
    AGS_HALF = 2,
    AGS_QUARTER = 3,
    AGS_SLOW = 4
};

typedef struct skywatcher_axis_status
{
    enum skywatcher_mode mode;
    enum skywatcher_direction direction;
    enum skywatcher_speed_mode speed;
    enum skywatcher_axis_action action;
    enum skywatcher_axis_state axis_state;
    enum skywatcher_init_state init_state;
} *skywatcher_axis_status_t;

typedef void (*skywatcher_goto_callback_t)(enum skywatcher_axis axis, void* user_data);

typedef struct skywatcher_status
{
    struct skywatcher_axis_status axis_status_1;
    struct skywatcher_axis_status axis_status_2;
    int32_t position[3];
    double deg[3];
    void* user_data;
    skywatcher_goto_callback_t goto_callback;
} *skywatcher_status_t;

typedef char data_t[16];
