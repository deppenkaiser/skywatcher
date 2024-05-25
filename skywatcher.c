#include "skywatcher.h"

#include <stdio.h>
#include <ctype.h>
#include <gtk/gtk.h>
#include <logging/logging.h>
#include <threading/threading.h>

#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128

#define AXIS_COUNT 2
#define POSITION_OFFSET 8388608

/*------------------------------------------------- PRIVATE ------------------------------------------------------*/

socket_handle_t _socket = SOCKET_INVALID_SOCKET;
uint32_t _counts_per_revolution[AXIS_COUNT] = {0};
int32_t _position[AXIS_COUNT] = {0};
int32_t _axis_position[AXIS_COUNT] = {0};
uint32_t _timer_frequency = 0;

bool _skywatcher_initialize_axis(enum skywatcher_axis axis);
bool _skywatcher_start_motion(enum skywatcher_axis axis);
bool _skywatcher_stop_motion(enum skywatcher_axis axis);
bool _skywatcher_get_axis_counts_per_revolution(uint32_t* step_count, enum skywatcher_axis axis);
bool _skywatcher_get_timer_frequency(uint32_t* frequency);
double _skywatcher_calculate_speed_cps(enum skywatcher_axis axis, double angular_speed_degree_per_s);
double _skywatcher_calculate_preset_value(enum skywatcher_axis axis, double angular_speed_degree_per_s);

bool _skywatcher_initialize_axis(enum skywatcher_axis axis)
{
    bool is_ok = false;
    if (axis != SA_BOTH)
    {
        char buffer[64] = {0};
        sprintf(buffer, ":F%d\r", axis);
        if (socket_send(_socket, buffer, strlen(buffer)))
        {
            memset(buffer, 0, sizeof(buffer));
            size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
            is_ok = strcmp(buffer, "=\r") == 0;
        }
    }
    return is_ok;
}

bool _skywatcher_stop_motion(enum skywatcher_axis axis)
{
    bool is_ok = false;
    char buffer[64] = {0};
    sprintf(buffer, ":K%d\r", axis);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}

bool _skywatcher_start_motion(enum skywatcher_axis axis)
{
    bool is_ok = false;
    char buffer[64] = {0};
    sprintf(buffer, ":J%d\r", axis);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}

bool _skywatcher_get_axis_counts_per_revolution(uint32_t* step_count, enum skywatcher_axis axis)
{
    bool is_ok = false;
    if (axis != SA_BOTH)
    {
        char buffer[64] = {0};
        sprintf(buffer, ":a%d\r", axis);
        if (socket_send(_socket, buffer, strlen(buffer)))
        {
            memset(buffer, 0, sizeof(buffer));
            char* endptr = NULL;
            size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
            is_ok = (buffer[0] == '=') && (buffer[7] == '\r');

            string_t hex_number = {0};
            hex_number[0] = buffer[6];
            hex_number[1] = buffer[5];
            hex_number[2] = buffer[4];
            hex_number[3] = buffer[3];
            hex_number[4] = buffer[2];
            hex_number[5] = buffer[1];
            
            *step_count = strtol(hex_number, &endptr, 16);
            is_ok = is_ok && (*endptr == '\0');
            *step_count -= POSITION_OFFSET;
        }
    }
    return is_ok;
}

bool _skywatcher_get_timer_frequency(uint32_t* frequency)
{
    bool is_ok = false;
    char buffer[64] = {0};
    sprintf(buffer, ":b1\r");
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        char* endptr = NULL;
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = (buffer[0] == '=') && (buffer[7] == '\r');

        string_t hex_number = {0};
        hex_number[0] = buffer[6];
        hex_number[1] = buffer[5];
        hex_number[2] = buffer[4];
        hex_number[3] = buffer[3];
        hex_number[4] = buffer[2];
        hex_number[5] = buffer[1];
        
        *frequency = strtol(hex_number, &endptr, 16);
        is_ok = is_ok && (*endptr == '\0');
    }
    return is_ok;
}

double _skywatcher_calculate_speed_cps(enum skywatcher_axis axis, double angular_speed_degree_per_s)
{
    double speed_cps = 0.0;
    if (axis != SA_BOTH)
    {
        speed_cps = angular_speed_degree_per_s * _counts_per_revolution[axis - 1] / 360.0;
    }
    return speed_cps;
}

double _skywatcher_calculate_preset_value(enum skywatcher_axis axis, double angular_speed_degree_per_s)
{
    double speed_cps = _skywatcher_calculate_speed_cps(axis, angular_speed_degree_per_s);
    double preset_value = 0.0;
    if (speed_cps != 0.0)
    {
        preset_value = _timer_frequency / speed_cps;
    }
    return preset_value;
}

/*------------------------------------------------- PUBLIC ------------------------------------------------------*/

bool skywatcher_open(const char* ip)
{
    _socket = socket_create_socket(1, false);
    return socket_connect(_socket, ip, 11880);
}

void skywatcher_close()
{
    socket_close(&_socket);
}

bool skywatcher_initialize_axis()
{
    bool is_ok = false;
    struct skywatcher_axis_status status = {0};
    
    while (status.init_state == SIS_NOT_INIT)
    {
        while (_skywatcher_initialize_axis(SA_AXIS_RA_AZ_1) == false)
        {
            threading_sleep(TSR_MILLI, 100);
        }
        
        while (skywatcher_get_axis_status(&status, SA_AXIS_RA_AZ_1) == false)
        {
            threading_sleep(TSR_MILLI, 100);
        }
    }
 
    status.init_state = SIS_NOT_INIT;
    while (status.init_state == SIS_NOT_INIT)
    {
        while (_skywatcher_initialize_axis(SA_AXIS_DEC_ALT_2) == false)
        {
            threading_sleep(TSR_MILLI, 100);
        }
        
        while (skywatcher_get_axis_status(&status, SA_AXIS_DEC_ALT_2) == false)
        {
            threading_sleep(TSR_MILLI, 100);
        }
    }

    is_ok = _skywatcher_get_axis_counts_per_revolution(&_counts_per_revolution[SA_AXIS_RA_AZ_1 - 1], SA_AXIS_RA_AZ_1);
    is_ok = is_ok && _skywatcher_get_axis_counts_per_revolution(&_counts_per_revolution[SA_AXIS_DEC_ALT_2 - 1], SA_AXIS_DEC_ALT_2);
    is_ok = is_ok && skywatcher_get_position(&_position[SA_AXIS_RA_AZ_1 - 1], SA_AXIS_RA_AZ_1);
    is_ok = is_ok && skywatcher_get_position(&_position[SA_AXIS_DEC_ALT_2 - 1], SA_AXIS_DEC_ALT_2);
    is_ok = is_ok && skywatcher_get_axis_position(&_axis_position[SA_AXIS_RA_AZ_1 - 1], SA_AXIS_RA_AZ_1);
    is_ok = is_ok && skywatcher_get_axis_position(&_axis_position[SA_AXIS_DEC_ALT_2 - 1], SA_AXIS_DEC_ALT_2);
    is_ok = is_ok && _skywatcher_get_timer_frequency(&_timer_frequency);
    return is_ok;
}

bool skywatcher_stop_motion(enum skywatcher_axis axis)
{
    bool is_ok = false;
    struct skywatcher_axis_status status_1 = {0};
    struct skywatcher_axis_status status_2 = {0};

    switch (axis)
    {
        case SA_BOTH:
            do
            {
                while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                if ((status_1.action == SAA_RUNNING) || (status_2.action == SAA_RUNNING))
                {
                    while (_skywatcher_stop_motion(axis) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }
                }
            } while ((status_1.action == SAA_RUNNING) || (status_2.action == SAA_RUNNING));
            is_ok = true;
            break;

        case SA_AXIS_RA_AZ_1:
            do
            {
                while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                if (status_1.action == SAA_RUNNING)
                {
                    while (_skywatcher_stop_motion(axis) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }
                }
            } while (status_1.action == SAA_RUNNING);
            is_ok = true;
            break;

        case SA_AXIS_DEC_ALT_2:
            do
            {
                while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                if (status_2.action == SAA_RUNNING)
                {
                    while (_skywatcher_stop_motion(axis) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }
                }
            } while (status_2.action == SAA_RUNNING);
            is_ok = true;
            break;
    }

    return is_ok;
}

bool skywatcher_start_motion(enum skywatcher_axis axis)
{
    bool is_ok = false;
    struct skywatcher_axis_status status_1 = {0};
    struct skywatcher_axis_status status_2 = {0};

    switch (axis)
    {
        case SA_BOTH:
            do
            {
                while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                if ((status_1.action == SAA_STOPPED) || (status_2.action == SAA_STOPPED))
                {
                    while (_skywatcher_start_motion(axis) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }
                }
            } while ((status_1.action == SAA_STOPPED) || (status_2.action == SAA_STOPPED));
            is_ok = true;
            break;

        case SA_AXIS_RA_AZ_1:
            do
            {
                while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                if (status_1.action == SAA_STOPPED)
                {
                    while (_skywatcher_start_motion(axis) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_1, SA_AXIS_RA_AZ_1) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }
                }
            } while (status_1.action == SAA_STOPPED);
            is_ok = true;
            break;

        case SA_AXIS_DEC_ALT_2:
            do
            {
                while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                {
                    threading_sleep(TSR_MILLI, 100);
                }

                if (status_2.action == SAA_STOPPED)
                {
                    while (_skywatcher_start_motion(axis) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }

                    while (skywatcher_get_axis_status(&status_2, SA_AXIS_DEC_ALT_2) == false)
                    {
                        threading_sleep(TSR_MILLI, 100);
                    }
                }
            } while (status_2.action == SAA_STOPPED);
            is_ok = true;
            break;
    }

    return is_ok;
}

bool skywatcher_instant_stop(enum skywatcher_axis axis)
{
    bool is_ok = false;
    char buffer[64] = {0};
    sprintf(buffer, ":L%d\r", axis);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}

bool skywatcher_get_position(int32_t* position, enum skywatcher_axis axis)
{
    bool is_ok = false;
    if (axis != SA_BOTH)
    {
        char buffer[64] = {0};
        sprintf(buffer, ":j%d\r", axis);
        if (socket_send(_socket, buffer, strlen(buffer)))
        {
            memset(buffer, 0, sizeof(buffer));
            char* endptr = NULL;
            size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
            is_ok = (buffer[0] == '=') && (buffer[7] == '\r');

            string_t hex_number = {0};
            hex_number[0] = buffer[6];
            hex_number[1] = buffer[5];
            hex_number[2] = buffer[4];
            hex_number[3] = buffer[3];
            hex_number[4] = buffer[2];
            hex_number[5] = buffer[1];
            
            *position = strtol(hex_number, &endptr, 16);
            is_ok = is_ok && (*endptr == '\0');
            *position -= POSITION_OFFSET;
        }
    }
    return is_ok;
}

bool skywatcher_get_axis_position(int32_t* position, enum skywatcher_axis axis)
{
    bool is_ok = false;
    if (axis != SA_BOTH)
    {
        char buffer[64] = {0};
        sprintf(buffer, ":d%d\r", axis);
        if (socket_send(_socket, buffer, strlen(buffer)))
        {
            memset(buffer, 0, sizeof(buffer));
            char* endptr = NULL;
            size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
            is_ok = (buffer[0] == '=') && (buffer[7] == '\r');

            string_t hex_number = {0};
            hex_number[0] = buffer[6];
            hex_number[1] = buffer[5];
            hex_number[2] = buffer[4];
            hex_number[3] = buffer[3];
            hex_number[4] = buffer[2];
            hex_number[5] = buffer[1];
            
            *position = strtol(hex_number, &endptr, 16);
            is_ok = is_ok && (*endptr == '\0');
            *position -= POSITION_OFFSET;
        }
    }
    return is_ok;
}

bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis)
{
    bool is_ok = false;
    if (axis != SA_BOTH)
    {
        char buffer[64] = {0};
        sprintf(buffer, ":f%d\r", axis);
        if (socket_send(_socket, buffer, strlen(buffer)))
        {
            memset(buffer, 0, sizeof(buffer));
            char* endptr = NULL;
            size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
            is_ok = (buffer[0] == '=') && (buffer[4] == '\r');

            status->mode = buffer[1] & BIT_0 ? SM_TRACKING : SM_GOTO;
            status->direction = buffer[1] & BIT_1 ? SD_CCW : SD_CW;
            status->speed = buffer[1] & BIT_2 ? SSM_FAST : SSM_SLOW;
            status->action = buffer[2] & BIT_0 ? SAA_RUNNING : SAA_STOPPED;
            status->init_state = buffer[3] & BIT_0 ? SIS_DONE : SIS_NOT_INIT;
        }
    }
    return is_ok;
}

bool skywatcher_set_motion_mode_tracking(enum skywatcher_axis axis, enum skywatcher_direction direction)
{
    bool is_ok = false;
    string_t buffer = {0};

    snprintf(buffer, sizeof(buffer), ":G%d%d%d\r", axis, BIT_0 | BIT_1, BIT_0);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        char* endptr = NULL;
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}

bool skywatcher_set_motion_mode_tracking_slow(enum skywatcher_axis axis, enum skywatcher_direction direction)
{
    bool is_ok = false;
    string_t buffer = {0};

    snprintf(buffer, sizeof(buffer), ":G%d%d%d\r", axis, BIT_0, direction);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        char* endptr = NULL;
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}

bool skywatcher_set_auto_guide_speed(enum skywatcher_axis axis, enum skywatcher_auto_guide_speed speed)
{
    bool is_ok = false;
    string_t buffer = {0};

    snprintf(buffer, sizeof(buffer), ":P%d%d\r", axis, speed);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        char* endptr = NULL;
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}

bool skywatcher_set_preset_value(enum skywatcher_axis axis, double angular_speed_degree_per_s)
{
    bool is_ok = false;
    string_t buffer = {0};
    if (axis != SA_BOTH)
    {
        int32_t preset_value = (int32_t) _skywatcher_calculate_preset_value(axis, angular_speed_degree_per_s);
        char preset_value_hex[16] = {0};
        sprintf(preset_value_hex, "%x", preset_value + POSITION_OFFSET);
        snprintf(buffer, sizeof(buffer), ":I%dxxxxxx\r", axis);
        buffer[3] = toupper(preset_value_hex[4]);
        buffer[4] = toupper(preset_value_hex[5]);
        buffer[5] = toupper(preset_value_hex[2]);
        buffer[6] = toupper(preset_value_hex[3]);
        buffer[7] = toupper(preset_value_hex[0]);
        buffer[8] = toupper(preset_value_hex[1]);
        if (socket_send(_socket, buffer, strlen(buffer)))
        {
            memset(buffer, 0, sizeof(buffer));
            size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
            is_ok = strcmp(buffer, "=\r") == 0;
        }
    }
    return is_ok;
}

void skywatcher_handle_arrow_keys(uint32_t key_value)
{
    switch (key_value)
    {
        case GDK_KEY_Left:
            logging_log_message("mount move left", true);
            break;

        case GDK_KEY_Right:
            logging_log_message("mount move right", true);
            break;
            
        case GDK_KEY_Up:
            logging_log_message("mount move north", true);
            skywatcher_stop_motion(SA_AXIS_RA_AZ_1);
            skywatcher_set_motion_mode_tracking(SA_AXIS_RA_AZ_1, SD_CW);
            skywatcher_set_preset_value(SA_AXIS_RA_AZ_1, 1.0);
            skywatcher_start_motion(SA_AXIS_RA_AZ_1);
            break;
            
        case GDK_KEY_Down:
            logging_log_message("mount move south", true);
            skywatcher_instant_stop(SA_BOTH);
            break;            
    }
}
