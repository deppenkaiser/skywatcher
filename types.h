#pragma once

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
