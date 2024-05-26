#include "skywatcher.h"

int main(int argc, char **argv)
{
    if (skywatcher_open("192.168.0.51"))
    {
        if (skywatcher_initialize_axis())
        {
            bool is_ok = true;
            struct skywatcher_axis_status status_1 = {0};
            struct skywatcher_axis_status status_2 = {0};
            int32_t position_1 = 0;
            int32_t position_2 = 0;
            int32_t axis_position_1 = 0;
            int32_t axis_position_2 = 0;
            is_ok = is_ok && skywatcher_set_position(SA_AXIS_1, 0);
            is_ok = is_ok && skywatcher_set_position(SA_AXIS_2, 0);
            is_ok = is_ok && skywatcher_get_position(SA_AXIS_1, &position_1);
            is_ok = is_ok && skywatcher_get_position(SA_AXIS_2, &position_2);
            is_ok = is_ok && skywatcher_get_axis_position(SA_AXIS_1, &axis_position_1);
            is_ok = is_ok && skywatcher_get_axis_position(SA_AXIS_2, &axis_position_2);
            is_ok = false;
        }

        skywatcher_close();
    }

    return 0;
}
