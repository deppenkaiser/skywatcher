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
#define CMD_SET_MOTION_MODE     ":G%d%d%d\r"
#define CMD_GET_AXIS_POSITION   ":d%d\r"
#define CMD_GET_BOARD_VERSION   ":e%d\r"
#define CMD_GET_TIMER_FREQUENCY ":b1\r"
#define CMD_GET_CPR             ":a%d\r"
#define CMD_START_MOTION        ":J%d\r"
#define CMD_STOP_MOTION         ":K%d\r"

/*------------------------------------------------- PRIVATE ------------------------------------------------------*/

socket_handle_t _socket = SOCKET_INVALID_SOCKET;
char _buffer_in[16] = {0};
char _buffer_out[16] = {0};
uint32_t _timer_frequency = 0;
uint32_t _cpr[3] = {0};

size_t _skywatcher_telegram();
bool _skywatcher_initialize_axis(enum skywatcher_axis axis);
bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis);
bool _skywatcher_get_axis_status(enum skywatcher_axis axis);
bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr);
bool _skywatcher_get_timer_frequency();
void _skywatcher_execute_with_param_1(char* command, enum skywatcher_axis axis, uint32_t value_1);
void _skywatcher_execute_with_param_2(char* command, enum skywatcher_axis axis, uint32_t value_1, uint32_t value_2);

size_t _skywatcher_telegram()
{
    size_t bytes_received = 0;
    if (socket_send(_socket, _buffer_in, strlen(_buffer_in)))
    {
        bytes_received = socket_receive(_socket, _buffer_out, sizeof(_buffer_out));
    }
    return bytes_received;
}

void _skywatcher_execute(char* command, enum skywatcher_axis axis)
{
    memset(_buffer_in, 0, sizeof(_buffer_in));
    memset(_buffer_out, 0, sizeof(_buffer_out));
    if (axis != SA_NONE)
    {
        sprintf(_buffer_in, command, axis);
    }
    else
    {
        memcpy(_buffer_in, command, strlen(command));
    }
    _skywatcher_telegram();
}

void _skywatcher_execute_with_param_1(char* command, enum skywatcher_axis axis, uint32_t value_1)
{
    memset(_buffer_in, 0, sizeof(_buffer_in));
    memset(_buffer_out, 0, sizeof(_buffer_out));
    sprintf(_buffer_in, command, axis, value_1);
    _skywatcher_telegram();
}

void _skywatcher_execute_with_param_2(char* command, enum skywatcher_axis axis, uint32_t value_1, uint32_t value_2)
{
    memset(_buffer_in, 0, sizeof(_buffer_in));
    memset(_buffer_out, 0, sizeof(_buffer_out));
    sprintf(_buffer_in, command, axis, value_1, value_2);
    _skywatcher_telegram();
}

bool _skywatcher_initialize_axis(enum skywatcher_axis axis)
{
    _skywatcher_execute(CMD_INIT, axis);
    return _buffer_out[0] == '=';
}

bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis)
{
    _skywatcher_execute(CMD_GET_BOARD_VERSION, axis);
    return _buffer_out[0] == '=';
}

bool _skywatcher_get_timer_frequency(uint32_t* frequency)
{
    char value[16] = {0};
    _skywatcher_execute(CMD_GET_TIMER_FREQUENCY, SA_NONE);
    bool is_ok = _buffer_out[0] == '=';
    if (is_ok)
    {
        char* endptr = NULL;
        value[4] = _buffer_out[1];
        value[5] = _buffer_out[2];
        value[2] = _buffer_out[3];
        value[3] = _buffer_out[4];
        value[0] = _buffer_out[5];
        value[1] = _buffer_out[6];
        *frequency = strtol(value, &endptr, 16);
    }
    return is_ok;
}

bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr)
{
    char value[16] = {0};
    _skywatcher_execute(CMD_GET_CPR, axis);
    bool is_ok = _buffer_out[0] == '=';
    if (is_ok)
    {
        char* endptr = NULL;
        value[4] = _buffer_out[1];
        value[5] = _buffer_out[2];
        value[2] = _buffer_out[3];
        value[3] = _buffer_out[4];
        value[0] = _buffer_out[5];
        value[1] = _buffer_out[6];
        *cpr = strtol(value, &endptr, 16);
    }
    return is_ok;
}

bool _skywatcher_get_axis_status(enum skywatcher_axis axis)
{
    _skywatcher_execute(CMD_STATUS, axis);
    return _buffer_out[0] == '=';
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
    logging_log_message(_buffer_out[3] == '0' ? "board axis 1: eq" : "board axis 1: az", true);

    is_ok = is_ok && _skywatcher_get_motor_board_version(SA_AXIS_2);
    logging_log_message(_buffer_out[3] == '0' ? "board axis 2: eq" : "board axis 2: az", true);

    return is_ok;
}

bool skywatcher_set_axis_sleep(enum skywatcher_axis axis, bool sleep)
{
    _skywatcher_execute_with_param_1(CMD_SLEEP, axis, sleep ? SAS_BLOCKED : SAS_NORMAL);
    return _buffer_out[0] == '=';
}

bool skywatcher_start_motion(enum skywatcher_axis axis)
{
    _skywatcher_execute(CMD_START_MOTION, axis);
    return _buffer_out[0] == '=';
}

bool skywatcher_stop_motion(enum skywatcher_axis axis)
{
    _skywatcher_execute(CMD_STOP_MOTION, axis);
    return _buffer_out[0] == '=';
}

bool skywatcher_instant_stop(enum skywatcher_axis axis)
{
    _skywatcher_execute(CMD_INSTANT_STOP, axis);
    return _buffer_out[0] == '=';
}

bool skywatcher_get_axis_status(skywatcher_axis_status_t status, enum skywatcher_axis axis)
{
    bool is_ok = _skywatcher_get_axis_status(axis);
    char db_1 = _buffer_out[1];
    char db_2 = _buffer_out[2];
    char db_3 = _buffer_out[3];
    status->mode = (db_1 & BIT_0) > 0 ? SM_TRACKING : SM_GOTO;
    status->direction = (db_1 & BIT_1) > 0 ? SD_CCW : SD_CW;
    status->init_state = (db_1 & BIT_2) > 0 ? SSM_FAST : SSM_SLOW;
    status->action = (db_2 && BIT_0) > 0 ? SAA_RUNNING : SAA_STOPPED;
    status->axis_state = (db_2 && BIT_1) > 0 ? SAS_BLOCKED : SAS_NORMAL;
    status->init_state = (db_3 && BIT_0) > 0 ? SIS_DONE : SIS_NOT_INIT;
    return is_ok;
}

bool skywatcher_set_motion_mode(enum skywatcher_axis axis, bool tracking, bool fast, bool cw, bool north)
{
    _skywatcher_execute_with_param_2(CMD_GET_POSITION, axis, BIT_2 | BIT_0, BIT_0);
    return _buffer_out[0] == '=';
}

bool skywatcher_get_speed(enum skywatcher_axis axis, double* angular_speed_degrees_per_s)
{
    char value[16] = {0};
    _skywatcher_execute(CMD_GET_SPEED, axis);
    value[4] = _buffer_out[1];
    value[5] = _buffer_out[2];
    value[2] = _buffer_out[3];
    value[3] = _buffer_out[4];
    value[0] = _buffer_out[5];
    value[1] = _buffer_out[6];
    char* endptr = NULL;
    uint32_t preset = strtol(value, &endptr, 16);
    double counts_per_s = _timer_frequency / (double) preset;
    *angular_speed_degrees_per_s = counts_per_s * 360.0 / (double) _cpr[axis];
    return _buffer_out[0] == '=';
}

bool skywatcher_set_speed(enum skywatcher_axis axis, double angular_speed_degrees_per_s)
{
    bool is_ok = false;
    if (angular_speed_degrees_per_s != 0.0)
    {
        char value[16] = {0};
        char value_shift[16] = {0};
        double counts_per_s = angular_speed_degrees_per_s * _cpr[axis] / 360.0;
        double preset = _timer_frequency / counts_per_s;
        memset(_buffer_in, 0, sizeof(_buffer_in));
        sprintf(value, "%x", (int32_t) preset);
        
        uint32_t value_lenght = strlen(value);
        memset(value_shift, '0', 6);
        for (uint32_t i = 0; i < value_lenght; ++i)
        {
            value_shift[5 - i] = toupper(value[value_lenght - 1 - i]);
        }

        sprintf(_buffer_in, CMD_SET_SPEED, axis);
        _buffer_in[3] = value_shift[4];
        _buffer_in[4] = value_shift[5];
        _buffer_in[5] = value_shift[2];
        _buffer_in[6] = value_shift[3];
        _buffer_in[7] = value_shift[0];
        _buffer_in[8] = value_shift[1];

        size_t bytes_received = 0;
        if (socket_send(_socket, _buffer_in, strlen(_buffer_in)))
        {
            bytes_received = socket_receive(_socket, _buffer_out, sizeof(_buffer_out));
        }
    }
    return _buffer_out[0] == '=';
}

bool skywatcher_get_position(enum skywatcher_axis axis, int32_t* position)
{
    char value[16] = {0};
    _skywatcher_execute(CMD_GET_POSITION, axis);
    value[4] = _buffer_out[1];
    value[5] = _buffer_out[2];
    value[2] = _buffer_out[3];
    value[3] = _buffer_out[4];
    value[0] = _buffer_out[5];
    value[1] = _buffer_out[6];
    char* endptr = NULL;
    *position = strtol(value, &endptr, 16) - POSITION_OFFSET;
    return _buffer_out[0] == '=';
}

bool skywatcher_set_position(enum skywatcher_axis axis, int32_t position)
{
    char value[16] = {0};
    char buffer[16] = {0};
    sprintf(buffer, CMD_SET_POSITION, axis);
    sprintf(value, "%x", position + POSITION_OFFSET);
    buffer[3] = value[4];
    buffer[4] = value[5];
    buffer[5] = value[2];
    buffer[6] = value[3];
    buffer[7] = value[0];
    buffer[8] = value[1];
    _skywatcher_execute(buffer, SA_NONE);
    return _buffer_out[0] == '=';
}

bool skywatcher_get_axis_position(enum skywatcher_axis axis, int32_t* position)
{
    char value[16] = {0};
    _skywatcher_execute(CMD_GET_AXIS_POSITION, axis);
    value[4] = _buffer_out[1];
    value[5] = _buffer_out[2];
    value[2] = _buffer_out[3];
    value[3] = _buffer_out[4];
    value[0] = _buffer_out[5];
    value[1] = _buffer_out[6];
    char* endptr = NULL;
    *position = strtol(value, &endptr, 16) - POSITION_OFFSET;
    return _buffer_out[0] == '=';
}
