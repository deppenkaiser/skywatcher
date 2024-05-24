#include "skywatcher.h"

#include <stdio.h>

#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128

socket_handle_t _socket = SOCKET_INVALID_SOCKET;

bool skywatcher_initialize()
{
    _socket = socket_create_socket(1, false);
    return socket_connect(_socket, "192.168.0.51", 11880);
}

void skywatcher_uninitialize()
{
    socket_close(&_socket);
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

bool skywatcher_start_motion(enum skywatcher_axis axis)
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

bool skywatcher_get_timer_frequency(uint32_t* frequency)
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

bool skywatcher_get_axis_resolution(uint32_t* step_count, enum skywatcher_axis axis)
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

            *step_count -= strtol("800000", &endptr, 16);
            is_ok = is_ok && (*endptr == '\0');
        }
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

            *position -= strtol("800000", &endptr, 16);
            is_ok = is_ok && (*endptr == '\0');
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

            *position -= strtol("800000", &endptr, 16);
            is_ok = is_ok && (*endptr == '\0');
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
            status->mode = buffer[1] == '0' ? SM_GOTO : SM_TRACKING;
            status->direction = buffer[2] == '0' ? SD_CW : SD_CCW;
            status->speed = buffer[3] == '0' ? SSM_SLOW : SSM_FAST;
        }
    }
    return is_ok;
}

bool skywatcher_set_motion_mode_tracking(enum skywatcher_axis axis, enum skywatcher_direction direction)
{
    bool is_ok = false;
    string_t buffer = {0};

    snprintf(buffer, sizeof(buffer), ":G%d%d%d\r", axis, BIT_0 | BIT_1, direction);
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        char* endptr = NULL;
        size_t received_bytes = socket_receive(_socket, buffer, sizeof(buffer));
        is_ok = strcmp(buffer, "=\r") == 0;
    }
    return is_ok;
}
