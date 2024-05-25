#include "print.h"
#include "types.h"

#include <string/string.h>

void skywatcher_print_status_change(skywatcher_axis_status_t axis_1, skywatcher_axis_status_t axis_2)
{
    static struct skywatcher_axis_status last_status_1 = {0};
    static struct skywatcher_axis_status last_status_2 = {0};

    if (last_status_1.direction != axis_1->direction)
    {
        string_t message = {0};
        char status_old[64] = {0};
        char status_new[64] = {0};
        skywatcher_print_direction(status_old, sizeof(status_old), last_status_1.direction);
        skywatcher_print_direction(status_new, sizeof(status_new), axis_1->direction);
        sprintf(message, "direction axis 1: %s -> %s", status_old, status_new);
        logging_log_message(message, true);
    }

    if (last_status_1.mode != axis_1->mode)
    {
        string_t message = {0};
        char status_old[64] = {0};
        char status_new[64] = {0};
        skywatcher_print_mode(status_old, sizeof(status_old), last_status_1.mode);
        skywatcher_print_mode(status_new, sizeof(status_new), axis_1->mode);
        sprintf(message, "mode axis 1: %s -> %s", status_old, status_new);
        logging_log_message(message, true);
    }

    if (last_status_1.speed != axis_1->speed)
    {
        string_t message = {0};
        char status_old[64] = {0};
        char status_new[64] = {0};
        skywatcher_print_speed_mode(status_old, sizeof(status_old), last_status_1.speed);
        skywatcher_print_speed_mode(status_new, sizeof(status_new), axis_1->speed);
        sprintf(message, "speed mode axis 1: %s -> %s", status_old, status_new);
        logging_log_message(message, true);
    }

    if (last_status_2.direction != axis_2->direction)
    {
        string_t message = {0};
        char status_old[64] = {0};
        char status_new[64] = {0};
        skywatcher_print_direction(status_old, sizeof(status_old), last_status_2.direction);
        skywatcher_print_direction(status_new, sizeof(status_new), axis_2->direction);
        sprintf(message, "direction axis 2: %s -> %s", status_old, status_new);
        logging_log_message(message, true);
    }

    if (last_status_2.mode != axis_2->mode)
    {
        string_t message = {0};
        char status_old[64] = {0};
        char status_new[64] = {0};
        skywatcher_print_mode(status_old, sizeof(status_old), last_status_2.mode);
        skywatcher_print_mode(status_new, sizeof(status_new), axis_2->mode);
        sprintf(message, "mode axis 2: %s -> %s", status_old, status_new);
        logging_log_message(message, true);
    }

    if (last_status_2.speed != axis_2->speed)
    {
        string_t message = {0};
        char status_old[64] = {0};
        char status_new[64] = {0};
        skywatcher_print_speed_mode(status_old, sizeof(status_old), last_status_2.speed);
        skywatcher_print_speed_mode(status_new, sizeof(status_new), axis_2->speed);
        sprintf(message, "speed mode axis 2: %s -> %s", status_old, status_new);
        logging_log_message(message, true);
    }

    last_status_1 = *axis_1;
    last_status_2 = *axis_2;
}
