
#include "skywatcher.h"

#include <threading/threading.h>

int main(int argc, char **argv)
{
    if (skywatcher_open("192.168.0.51"))
    {
        if (skywatcher_initialize_axis(2.4, 1624.0))
        {
            if (skywatcher_set_siderial_speed())
            {
                threading_sleep(TSR_SECOND, 30);
                skywatcher_stop_motion(SA_AXIS_1);
            }
        }

        skywatcher_close();
    }

    return 0;
}


/*
                    if (skywatcher_set_position(SA_AXIS_1, 0) && skywatcher_set_position(SA_AXIS_2, 0))
                    {
                        if (skywatcher_set_siderial_speed())
                        {
                            if (skywatcher_set_motion_mode(SA_AXIS_1, true, false, true, false))
                            {
                                skywatcher_start_motion(SA_AXIS_1);
                                skywatcher_stop_motion(SA_AXIS_1);
                                skywatcher_instant_stop(SA_AXIS_1);
                            }
                        }
                    }
            is_ok = is_ok && skywatcher_set_speed(SA_AXIS_1, 0.1);
            is_ok = is_ok && skywatcher_set_motion_mode(SA_AXIS_1, true, true, true, false);
            is_ok = is_ok && skywatcher_start_motion(SA_AXIS_1);
            for (uint32_t i = 0; i < 10; ++i)
            {
                threading_sleep(TSR_SECOND, 1);
            }
            is_ok = is_ok && skywatcher_stop_motion(SA_AXIS_1);
            is_ok = is_ok && skywatcher_instant_stop(SA_AXIS_1);

            is_ok = is_ok && skywatcher_set_speed(SA_AXIS_1, 0.1);
            is_ok = is_ok && skywatcher_set_motion_mode(SA_AXIS_1, true, true, false, false);
            is_ok = is_ok && skywatcher_start_motion(SA_AXIS_1);
            for (uint32_t i = 0; i < 10; ++i)
            {
                threading_sleep(TSR_SECOND, 1);
            }
            is_ok = is_ok && skywatcher_stop_motion(SA_AXIS_1);
            is_ok = is_ok && skywatcher_instant_stop(SA_AXIS_1);

            int32_t target = -500000;
            is_ok = is_ok && skywatcher_set_goto_target(SA_AXIS_1, target);
            is_ok = is_ok && skywatcher_set_motion_mode(SA_AXIS_1, false, true, true, false);
            is_ok = is_ok && skywatcher_start_motion(SA_AXIS_1);

            do
            {
                skywatcher_get_axis_status(SA_AXIS_1, &status_1);
                threading_sleep(TSR_SECOND, 1);
            } while (status_1.mode == SM_GOTO);

            is_ok = is_ok && skywatcher_set_goto_target(SA_AXIS_1, 0);
            is_ok = is_ok && skywatcher_set_motion_mode(SA_AXIS_1, false, true, true, false);
            is_ok = is_ok && skywatcher_start_motion(SA_AXIS_1);
*/