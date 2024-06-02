#include "skywatcher.h"

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <logging/logging.h>
#include <threading/threading.h>

/*------------------------------------------------- PRIVATE ------------------------------------------------------*/

#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128

#define POSITION_OFFSET 8388608
#define SIDERIAL_DAY_S 86164.0989

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
#define CMD_SET_GOTO_TARGET     ":S%dxxxxxx\r"
#define CMD_GET_GOTO_TARGET     ":h%d\r"

typedef char data_t[16];

socket_handle_t _socket = SOCKET_INVALID_SOCKET;
data_t _buffer_in = {0};
data_t _buffer_out = {0};
uint32_t _timer_frequency = 0;
uint32_t _cpr[3] = {0};
double _siderial_w_deg_per_s = 0.0;
double _siderial_w_error_1_level_deg_per_s = 0.0;
double _siderial_w_error_2_level_deg_per_s = 0.0;
double _max_exposure_time_s = 0.0;

threading_critical_section _cs = {0};

size_t _skywatcher_telegram();
bool _skywatcher_initialize_axis(enum skywatcher_axis axis);
bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis);
bool _skywatcher_get_axis_status(enum skywatcher_axis axis);
bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr);
bool _skywatcher_get_timer_frequency();
void _skywatcher_execute_with_param_1(char* command, enum skywatcher_axis axis, uint32_t value_1);
void _skywatcher_execute_with_param_2(char* command, enum skywatcher_axis axis, uint32_t value_1, uint32_t value_2);
void _skywatcher_calculate_siderial_angular_speed_deg_per_s();
double _skywatcher_calculate_max_exposure_time_s(double pixel_size_um, double focal_length_mm);
void _skywatcher_put_buffer_in(data_t value);
void _skywatcher_get_buffer_out(data_t value);
void _skywatcher_put_buffer_to_buffer(data_t out, data_t in);
int32_t _skywatcher_convert_data(data_t value);
int32_t _skywatcher_convert_position_data(data_t value);

void _skywatcher_calculate_siderial_angular_speed_deg_per_s()
{
    double _siderial_w_real_degree_per_s = 0.0;
    _siderial_w_deg_per_s = 360.0 / SIDERIAL_DAY_S;
    skywatcher_set_speed(SA_AXIS_1, _siderial_w_deg_per_s);
    skywatcher_get_speed(SA_AXIS_1, &_siderial_w_real_degree_per_s);
    _siderial_w_error_1_level_deg_per_s = _siderial_w_deg_per_s - _siderial_w_real_degree_per_s;
    skywatcher_set_speed(SA_AXIS_1, _siderial_w_deg_per_s + _siderial_w_error_1_level_deg_per_s);
    skywatcher_get_speed(SA_AXIS_1, &_siderial_w_real_degree_per_s);
    _siderial_w_error_2_level_deg_per_s = _siderial_w_deg_per_s - _siderial_w_real_degree_per_s;
}

double _skywatcher_calculate_max_exposure_time_s(double pixel_size_um, double focal_length_mm)
{
    double phi_pixel_rad = atan2(pixel_size_um * 1.0e-3, focal_length_mm);
    double phi_pixel_deg = phi_pixel_rad / acos(-1) * 180.0;
    return fabs(phi_pixel_deg / _siderial_w_error_2_level_deg_per_s);
}

size_t _skywatcher_telegram()
{
    size_t bytes_received = 0;
    threading_lock_critical_section(&_cs);
    if (socket_send(_socket, _buffer_in, strlen(_buffer_in)))
    {
        bytes_received = socket_receive(_socket, _buffer_out, sizeof(_buffer_out));
    }
    threading_unlock_critical_section(&_cs);
    return bytes_received;
}

void _skywatcher_execute(char* command, enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
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
    threading_unlock_critical_section(&_cs);
}

void _skywatcher_execute_with_param_1(char* command, enum skywatcher_axis axis, uint32_t value_1)
{
    threading_lock_critical_section(&_cs);
    memset(_buffer_in, 0, sizeof(_buffer_in));
    memset(_buffer_out, 0, sizeof(_buffer_out));
    sprintf(_buffer_in, command, axis, value_1);
    _skywatcher_telegram();
    threading_unlock_critical_section(&_cs);
}

void _skywatcher_execute_with_param_2(char* command, enum skywatcher_axis axis, uint32_t value_1, uint32_t value_2)
{
    threading_lock_critical_section(&_cs);
    memset(_buffer_in, 0, sizeof(_buffer_in));
    memset(_buffer_out, 0, sizeof(_buffer_out));
    sprintf(_buffer_in, command, axis, value_1, value_2);
    _skywatcher_telegram();
    threading_unlock_critical_section(&_cs);
}

bool _skywatcher_initialize_axis(enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_INIT, axis);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_BOARD_VERSION, axis);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool _skywatcher_get_timer_frequency(uint32_t* frequency)
{
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_TIMER_FREQUENCY, SA_NONE);
    bool is_ok = _buffer_out[0] == '=';
    if (is_ok)
    {
        _skywatcher_get_buffer_out(value);
        *frequency = _skywatcher_convert_data(value);
    }
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr)
{
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_CPR, axis);
    bool is_ok = _buffer_out[0] == '=';
    if (is_ok)
    {
        _skywatcher_get_buffer_out(value);
        *cpr = _skywatcher_convert_data(value);
    }
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool _skywatcher_get_axis_status(enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_STATUS, axis);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

void _skywatcher_put_buffer_to_buffer(data_t out, data_t in)
{
    out[3] = toupper(in[4]);
    out[4] = toupper(in[5]);
    out[5] = toupper(in[2]);
    out[6] = toupper(in[3]);
    out[7] = toupper(in[0]);
    out[8] = toupper(in[1]);
}

void _skywatcher_get_buffer_out(data_t value)
{
    value[4] = _buffer_out[1];
    value[5] = _buffer_out[2];
    value[2] = _buffer_out[3];
    value[3] = _buffer_out[4];
    value[0] = _buffer_out[5];
    value[1] = _buffer_out[6];
}

int32_t _skywatcher_convert_data(data_t value)
{
    char* endptr = NULL;
    return strtol(value, &endptr, 16);
}

int32_t _skywatcher_convert_position_data(data_t value)
{
    return _skywatcher_convert_data(value) - POSITION_OFFSET;
}

/*------------------------------------------------- PUBLIC ------------------------------------------------------*/

bool skywatcher_open(const char* ip)
{
    bool connected = false;

    threading_initialize_critical_section(&_cs);
    _socket = socket_create_socket(1, false);
    if (socket_ping(ip))
    {
        // socket_connect does not return false, if the host is not reacheable!?
        // is the udp-socket the reason?
        socket_connect(_socket, ip, 11880);
        connected = true;
    }

    return connected;
}

void skywatcher_close()
{
    socket_close(&_socket);
    threading_destroy_critical_section(&_cs);
}

bool skywatcher_initialize_axis(double pixel_size_um, double focal_length_mm)
{
    threading_lock_critical_section(&_cs);

    bool is_ok = _skywatcher_initialize_axis(SA_AXIS_1);
    is_ok = is_ok && _skywatcher_initialize_axis(SA_AXIS_2);
    is_ok = is_ok && _skywatcher_get_timer_frequency(&_timer_frequency);
    is_ok = is_ok && _skywatcher_get_cpr(SA_AXIS_1, &_cpr[SA_AXIS_1]);
    is_ok = is_ok && _skywatcher_get_cpr(SA_AXIS_2, &_cpr[SA_AXIS_2]);

    is_ok = is_ok && _skywatcher_get_motor_board_version(SA_AXIS_1);
    logging_log_message(_buffer_out[3] == '0' ? "board axis 1: eq" : "board axis 1: az", true);

    is_ok = is_ok && _skywatcher_get_motor_board_version(SA_AXIS_2);
    logging_log_message(_buffer_out[3] == '0' ? "board axis 2: eq" : "board axis 2: az", true);

    _skywatcher_calculate_siderial_angular_speed_deg_per_s();
    _max_exposure_time_s = _skywatcher_calculate_max_exposure_time_s(pixel_size_um, focal_length_mm);

    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_siderial_speed()
{
    return skywatcher_set_speed(SA_AXIS_1, _siderial_w_deg_per_s + _siderial_w_error_1_level_deg_per_s);
}

bool skywatcher_set_axis_sleep(enum skywatcher_axis axis, bool sleep)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute_with_param_1(CMD_SLEEP, axis, sleep ? SAS_BLOCKED : SAS_NORMAL);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_start_motion(enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_START_MOTION, axis);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_stop_motion(enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_STOP_MOTION, axis);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_instant_stop(enum skywatcher_axis axis)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_INSTANT_STOP, axis);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_axis_status(enum skywatcher_axis axis, skywatcher_axis_status_t status)
{
    threading_lock_critical_section(&_cs);
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
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_motion_mode(enum skywatcher_axis axis, bool tracking, bool fast, bool ccw, bool south)
{
    uint32_t param_1 = BIT_2 | (fast ? BIT_1 : 0) | (tracking ? BIT_0 : 0);
    uint32_t param_2 = (south ? BIT_1 : 0) | (ccw ? BIT_0 : 0);
    threading_lock_critical_section(&_cs);
    _skywatcher_execute_with_param_2(CMD_SET_MOTION_MODE, axis, param_1, param_2);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_speed(enum skywatcher_axis axis, double* angular_speed_degrees_per_s)
{
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_SPEED, axis);
    _skywatcher_get_buffer_out(value);
    char* endptr = NULL;
    uint32_t preset = strtol(value, &endptr, 16);
    double counts_per_s = _timer_frequency / (double) preset;
    *angular_speed_degrees_per_s = counts_per_s * 360.0 / (double) _cpr[axis];
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_speed(enum skywatcher_axis axis, double angular_speed_degrees_per_s)
{
    bool is_ok = false;
    threading_lock_critical_section(&_cs);
    if (angular_speed_degrees_per_s != 0.0)
    {
        data_t value = {0}, value_shift = {0};
        double counts_per_s = angular_speed_degrees_per_s * _cpr[axis] / 360.0;
        double preset = _timer_frequency / counts_per_s;
        memset(_buffer_in, 0, sizeof(_buffer_in));
        sprintf(value, "%x", (int32_t) preset);
        
        uint32_t value_lenght = strlen(value);
        memset(value_shift, '0', 6);
        for (uint32_t i = 0; i < value_lenght; ++i)
        {
            value_shift[5 - i] = value[value_lenght - 1 - i];
        }

        sprintf(_buffer_in, CMD_SET_SPEED, axis);
        _skywatcher_put_buffer_to_buffer(_buffer_in, value_shift);

        size_t bytes_received = 0;
        if (socket_send(_socket, _buffer_in, strlen(_buffer_in)))
        {
            bytes_received = socket_receive(_socket, _buffer_out, sizeof(_buffer_out));
        }
    }
    is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_position(enum skywatcher_axis axis, int32_t* position)
{
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_POSITION, axis);
    _skywatcher_get_buffer_out(value);
    *position = _skywatcher_convert_position_data(value);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_position(enum skywatcher_axis axis, int32_t position)
{
    data_t value = {0}, buffer = {0};
    sprintf(buffer, CMD_SET_POSITION, axis);
    sprintf(value, "%x", position + POSITION_OFFSET);
    _skywatcher_put_buffer_to_buffer(buffer, value);
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(buffer, SA_NONE);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_axis_position(enum skywatcher_axis axis, int32_t* position)
{
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_AXIS_POSITION, axis);
    _skywatcher_get_buffer_out(value);
    *position = _skywatcher_convert_position_data(value);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_goto_target(enum skywatcher_axis axis, int32_t target)
{
    data_t value = {0}, buffer = {0};
    sprintf(buffer, CMD_SET_GOTO_TARGET, axis);
    sprintf(value, "%x", target + POSITION_OFFSET);
    _skywatcher_put_buffer_to_buffer(buffer, value);
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(buffer, SA_NONE);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_goto_target(enum skywatcher_axis axis, int32_t* target)
{
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_GOTO_TARGET, axis);
    _skywatcher_get_buffer_out(value);
    *target = _skywatcher_convert_position_data(value);
    bool is_ok = _buffer_out[0] == '=';
    threading_unlock_critical_section(&_cs);
    return is_ok;
}
