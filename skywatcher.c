#include "skywatcher.h"

#include <stdio.h>
#include <ctype.h>
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

#define CMD_INIT                ":F%d\r"
#define CMD_STATUS              ":f%d\r"
#define CMD_INSTANT_STOP        ":L%d\r"
#define CMD_SLEEP               ":B%d%d\r"
#define CMD_GET_POSITION        ":j%d\r"
#define CMD_SET_POSITION        ":E%dxxxxxx\r"
#define CMD_GET_SPEED           ":i%d\r"
#define CMD_SET_SPEED           ":I%dxxxxxx\r"
#define CMD_SET_MOTION_MODE     ":G%dxx\r"
#define CMD_GET_AXIS_POSITION   ":d%d\r"
#define CMD_GET_BOARD_VERSION   ":e%d\r"
#define CMD_GET_TIMER_FREQUENCY ":b1\r"
#define CMD_GET_CPR             ":a%d\r"
#define CMD_START_MOTION        ":J%d\r"
#define CMD_STOP_MOTION         ":K%d\r"

/*------------------------------------------------- PRIVATE ------------------------------------------------------*/

socket_handle_t _socket = SOCKET_INVALID_SOCKET;
char _buffer[16] = {0};
uint32_t _timer_frequency = 0;
uint32_t _cpr[3] = {0};

size_t _skywatcher_telegram(char* buffer, size_t bytes);
bool _skywatcher_initialize_axis(enum skywatcher_axis axis);
bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis);
bool _skywatcher_get_axis_status(enum skywatcher_axis axis);
bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr);
bool _skywatcher_get_timer_frequency();

size_t _skywatcher_telegram(char* buffer, size_t bytes)
{
    size_t bytes_received = 0;
    if (socket_send(_socket, buffer, strlen(buffer)))
    {
        memset(buffer, 0, sizeof(buffer));
        bytes_received = socket_receive(_socket, buffer, sizeof(buffer));
    }
    return bytes_received;
}

bool _skywatcher_initialize_axis(enum skywatcher_axis axis)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_INIT, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_GET_BOARD_VERSION, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool _skywatcher_get_timer_frequency(uint32_t* frequency)
{
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_GET_TIMER_FREQUENCY);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    if (is_ok)
    {
        char* endptr = NULL;
        value[4] = _buffer[1];
        value[5] = _buffer[2];
        value[2] = _buffer[3];
        value[3] = _buffer[4];
        value[0] = _buffer[5];
        value[1] = _buffer[6];
        *frequency = strtol(value, &endptr, 16);
    }
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr)
{
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_GET_CPR, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    if (is_ok)
    {
        char* endptr = NULL;
        value[4] = _buffer[1];
        value[5] = _buffer[2];
        value[2] = _buffer[3];
        value[3] = _buffer[4];
        value[0] = _buffer[5];
        value[1] = _buffer[6];
        *cpr = strtol(value, &endptr, 16);
    }
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool _skywatcher_get_axis_status(enum skywatcher_axis axis)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_STATUS, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
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
    bool is_ok = _skywatcher_initialize_axis(SA_AXIS_1);
    is_ok = is_ok && _skywatcher_initialize_axis(SA_AXIS_2);
    is_ok = is_ok && _skywatcher_get_timer_frequency(&_timer_frequency);
    is_ok = is_ok && _skywatcher_get_cpr(SA_AXIS_1, &_cpr[SA_AXIS_1]);
    is_ok = is_ok && _skywatcher_get_cpr(SA_AXIS_2, &_cpr[SA_AXIS_2]);

    is_ok = is_ok && _skywatcher_get_motor_board_version(SA_AXIS_1);
    logging_log_message(_buffer[3] == '0' ? "board axis 1: eq" : "board axis 1: az", true);

    is_ok = is_ok && _skywatcher_get_motor_board_version(SA_AXIS_2);
    logging_log_message(_buffer[3] == '0' ? "board axis 2: eq" : "board axis 2: az", true);

    return is_ok;
}

bool skywatcher_set_axis_sleep(enum skywatcher_axis axis, bool sleep)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_SLEEP, axis, sleep ? SAS_BLOCKED : SAS_NORMAL);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_start_motion(enum skywatcher_axis axis)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_START_MOTION, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_stop_motion(enum skywatcher_axis axis)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_START_MOTION, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_instant_stop(enum skywatcher_axis axis)
{
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_INSTANT_STOP, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis)
{
    bool is_ok = _skywatcher_get_axis_status(axis);
    char db_1 = _buffer[1];
    char db_2 = _buffer[2];
    char db_3 = _buffer[3];
    status->mode = (db_1 & BIT_0) > 0 ? SM_TRACKING : SM_GOTO;
    status->direction = (db_1 & BIT_1) > 0 ? SD_CCW : SD_CW;
    status->init_state = (db_1 & BIT_2) > 0 ? SSM_FAST : SSM_SLOW;
    status->action = (db_2 && BIT_0) > 0 ? SAA_RUNNING : SAA_STOPPED;
    status->axis_state = (db_2 && BIT_1) > 0 ? SAS_BLOCKED : SAS_NORMAL;
    status->init_state = (db_3 && BIT_0) > 0 ? SIS_DONE : SIS_NOT_INIT;
    return is_ok;
}

bool skywatcher_get_position(enum skywatcher_axis axis, int32_t* position)
{
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_GET_POSITION, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    value[4] = _buffer[1];
    value[5] = _buffer[2];
    value[2] = _buffer[3];
    value[3] = _buffer[4];
    value[0] = _buffer[5];
    value[1] = _buffer[6];
    char* endptr = NULL;
    *position = strtol(value, &endptr, 16) - POSITION_OFFSET;
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_set_motion_mode(enum skywatcher_axis axis, bool tracking, bool fast, bool cw, bool north)
{
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_SET_MOTION_MODE, axis);
    sprintf(value, "%x", BIT_2 | BIT_0);
    _buffer[3] = value[0];
    sprintf(value, "%x", BIT_0);
    _buffer[4] = value[0];
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_get_speed(enum skywatcher_axis axis, double* angular_speed_degrees_per_s)
{
    bool is_ok = false;
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_GET_SPEED, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    value[4] = _buffer[1];
    value[5] = _buffer[2];
    value[2] = _buffer[3];
    value[3] = _buffer[4];
    value[0] = _buffer[5];
    value[1] = _buffer[6];
    char* endptr = NULL;
    uint32_t preset = strtol(value, &endptr, 16);
    double counts_per_s = _timer_frequency / (double) preset;
    *angular_speed_degrees_per_s = counts_per_s * 360.0 / (double) _cpr[axis];
    is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_set_speed(enum skywatcher_axis axis, double angular_speed_degrees_per_s)
{
    bool is_ok = false;
    if (angular_speed_degrees_per_s != 0.0)
    {
        char value[16] = {0};
        double counts_per_s = angular_speed_degrees_per_s * _cpr[axis] / 360.0;
        double preset = _timer_frequency / counts_per_s;
        memset(_buffer, 0, sizeof(_buffer));
        sprintf(value, "%x", (int32_t) preset);
        sprintf(_buffer, CMD_SET_SPEED, axis);
        _buffer[3] = value[4] != 0 ? value[4] : '0';
        _buffer[4] = value[5] != 0 ? value[5] : '0';
        _buffer[5] = value[2] != 0 ? value[2] : '0';
        _buffer[6] = value[3] != 0 ? value[3] : '0';
        _buffer[7] = value[0] != 0 ? value[0] : '0';
        _buffer[8] = value[1] != 0 ? value[1] : '0';
        _skywatcher_telegram(_buffer, strlen(_buffer));
        is_ok = _buffer[0] != '!';
        memset(_buffer, 0, sizeof(_buffer));
    }
    return is_ok;
}

bool skywatcher_set_position(enum skywatcher_axis axis, int32_t position)
{
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(value, "%x", position + POSITION_OFFSET);
    sprintf(_buffer, CMD_SET_POSITION, axis);
    _buffer[3] = value[4];
    _buffer[4] = value[5];
    _buffer[5] = value[2];
    _buffer[6] = value[3];
    _buffer[7] = value[0];
    _buffer[8] = value[1];
    _skywatcher_telegram(_buffer, strlen(_buffer));
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}

bool skywatcher_get_axis_position(enum skywatcher_axis axis, int32_t* position)
{
    char value[16] = {0};
    memset(_buffer, 0, sizeof(_buffer));
    sprintf(_buffer, CMD_GET_AXIS_POSITION, axis);
    _skywatcher_telegram(_buffer, strlen(_buffer));
    value[4] = _buffer[1];
    value[5] = _buffer[2];
    value[2] = _buffer[3];
    value[3] = _buffer[4];
    value[0] = _buffer[5];
    value[1] = _buffer[6];
    char* endptr = NULL;
    *position = strtol(value, &endptr, 16) - POSITION_OFFSET;
    bool is_ok = _buffer[0] != '!';
    memset(_buffer, 0, sizeof(_buffer));
    return is_ok;
}
