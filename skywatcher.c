#include "skywatcher.h"

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <logging/logging.h>
#include <gst/gst.h>
#include <api/api.h>
#include <threading/threading.h>
#include <socket/socket.h>
#include <physics/physics.h>

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
#define CMD_SET_AUX             ":O%d\r"

protected_import(void, _skywatcher_execute(char* command, enum skywatcher_axis axis, data_t buffer_out));
protected_import(void, _skywatcher_execute_with_param_1(char* command, enum skywatcher_axis axis, uint32_t value_1, data_t buffer_in, data_t buffer_out));
protected_import(void, _skywatcher_execute_with_param_2(char* command, enum skywatcher_axis axis, uint32_t value_1, uint32_t value_2, data_t buffer_in, data_t buffer_out));

protected_import(socket_handle_t, _socket);

private uint32_t _timer_frequency = 0;
private uint32_t _cpr[3] = {0};
private double _siderial_w_deg_per_s = 0.0;
private double _siderial_w_error_1_level_deg_per_s = 0.0;
private double _siderial_w_error_2_level_deg_per_s = 0.0;
private double _max_exposure_time_s = 0.0;
private pthread_t _thread_handle = THREADING_INVALID_THREADHANDLE;
private bool _exit_thread = false;
private skywatcher_status_t _status;
protected threading_critical_section _cs = {0};

private void* _skywatcher_mount_thread(void* data)
{
    static struct skywatcher_axis_status last_status_axis_1 = {0};
    static struct skywatcher_axis_status last_status_axis_2 = {0};

    logging_log_message("mount thread started.");
    while (_exit_thread == false)
    {
        skywatcher_get_position(SA_AXIS_1, &_status->position[SA_AXIS_1]);
        skywatcher_get_position(SA_AXIS_2, &_status->position[SA_AXIS_2]);
        _status->deg[SA_AXIS_1] = physics_modulo((double) _status->position[SA_AXIS_1] / _cpr[SA_AXIS_1] * 360.0 + 360.0, 360.0);
        _status->deg[SA_AXIS_2] = (double) _status->position[SA_AXIS_2] / _cpr[SA_AXIS_2] * 360.0;
        skywatcher_get_axis_status(SA_AXIS_1, &_status->axis_status_1);
        skywatcher_get_axis_status(SA_AXIS_2, &_status->axis_status_2);

        if ((_status->axis_status_1.mode != last_status_axis_1.mode) && (last_status_axis_1.mode == SM_GOTO))
        {
            skywatcher_set_motion_mode(SA_AXIS_1, false, false, false, false);
            skywatcher_start_motion(SA_AXIS_1);
        }

        if ((_status->axis_status_2.mode != last_status_axis_2.mode) && (last_status_axis_2.mode == SM_GOTO))
        {
            skywatcher_set_motion_mode(SA_AXIS_2, false, false, false, false);
            skywatcher_start_motion(SA_AXIS_2);
        }

        last_status_axis_1 = _status->axis_status_1;
        last_status_axis_2 = _status->axis_status_2;

        threading_sleep(TTR_MILLI, 100);
    }
    logging_log_message("mount thread stoped.");

    return NULL;
}

private bool skywatcher_check_error(data_t buffer_out, const char* function_name)
{
    bool has_no_error = true;

    if (buffer_out[0] == '!')
    {
        char buffer[256] = {0};

        switch (buffer_out[1])
        {
            case '0': sprintf(buffer, "error: %s, %s", function_name, "unknown command"); break;
            case '1': sprintf(buffer, "error: %s, %s", function_name, "command length"); break;
            case '2': sprintf(buffer, "error: %s, %s", function_name, "Motor not Stopped"); break;
            case '3': sprintf(buffer, "error: %s, %s", function_name, "Invalid Character"); break;
            case '4': sprintf(buffer, "error: %s, %s", function_name, "Not Initialized"); break;
            case '5': sprintf(buffer, "error: %s, %s", function_name, "Driver Sleeping"); break;
        }
    
        has_no_error = false;
        logging_log_message(buffer);
    }

    return has_no_error;
}

private void _skywatcher_get_buffer_out(data_t value, data_t buffer_out)
{
    value[4] = buffer_out[1];
    value[5] = buffer_out[2];
    value[2] = buffer_out[3];
    value[3] = buffer_out[4];
    value[0] = buffer_out[5];
    value[1] = buffer_out[6];
}

private int32_t _skywatcher_convert_data(data_t value)
{
    char* endptr = NULL;
    return strtol(value, &endptr, 16);
}

private void _skywatcher_calculate_siderial_angular_speed_deg_per_s()
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

private double _skywatcher_calculate_max_exposure_time_s(double pixel_size_um, double focal_length_mm)
{
    double phi_pixel_rad = atan2(pixel_size_um * 1.0e-3, focal_length_mm);
    double phi_pixel_deg = phi_pixel_rad / acos(-1) * 180.0;
    return fabs(phi_pixel_deg / _siderial_w_error_2_level_deg_per_s);
}

private bool _skywatcher_initialize_axis(enum skywatcher_axis axis)
{
    data_t buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_INIT, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "_skywatcher_initialize_axis");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

private bool _skywatcher_get_motor_board_version(enum skywatcher_axis axis)
{
    data_t buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_BOARD_VERSION, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "_skywatcher_get_motor_board_version");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

private bool _skywatcher_get_timer_frequency(uint32_t* frequency)
{
    data_t buffer_out = {0};
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_TIMER_FREQUENCY, SA_NONE, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "_skywatcher_get_timer_frequency");
    if (is_ok)
    {
        _skywatcher_get_buffer_out(value, buffer_out);
        *frequency = _skywatcher_convert_data(value);
    }
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

private bool _skywatcher_get_cpr(enum skywatcher_axis axis, uint32_t* cpr)
{
    data_t buffer_out = {0};
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_CPR, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "_skywatcher_get_cpr");
    if (is_ok)
    {
        _skywatcher_get_buffer_out(value, buffer_out);
        *cpr = _skywatcher_convert_data(value);
    }
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

private bool _skywatcher_get_axis_status(enum skywatcher_axis axis, data_t buffer_out)
{
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_STATUS, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "_skywatcher_get_axis_status");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

private void _skywatcher_put_buffer_to_buffer(data_t out, data_t in)
{
    out[3] = toupper(in[4]);
    out[4] = toupper(in[5]);
    out[5] = toupper(in[2]);
    out[6] = toupper(in[3]);
    out[7] = toupper(in[0]);
    out[8] = toupper(in[1]);
}

private int32_t _skywatcher_convert_position_data(data_t value)
{
    return _skywatcher_convert_data(value) - POSITION_OFFSET;
}

void skywatcher_goto_home(enum skywatcher_axis axis, bool instant_stop)
{
    if (instant_stop)
    {
        skywatcher_instant_stop(axis);
    }
    skywatcher_set_goto_target(axis, 0);
    skywatcher_set_motion_mode(axis, false, false, false, false);
    skywatcher_start_motion(axis);
}

void skywatcher_goto_deg(enum skywatcher_axis axis, double degree)
{  
    if (axis == SA_AXIS_1)
    {
        degree = physics_modulo(degree, 360.0);
        degree = degree > 180.0 ? degree - 360.0 : degree;
    }

    int32_t position = (int32_t) (degree *_cpr[axis] / 360.0);
    skywatcher_stop_motion(axis);
    skywatcher_set_goto_target(axis, position);
    skywatcher_set_motion_mode(axis, false, false, false, false);
    skywatcher_start_motion(axis);
    char buffer[256] = {0};
    sprintf(buffer, "axis %d goto: %.2f", axis, degree);
    logging_log_message(buffer);
}

void skywatcher_start_thread(skywatcher_status_t status, void* user_data)
{
    _status = status;
    _thread_handle = threading_create_thread(_skywatcher_mount_thread, user_data);
}

void skywatcher_stop_thread()
{
    _exit_thread = true;
    threading_join_thread(_thread_handle);
}

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
    bool is_ok = false;
    threading_lock_critical_section(&_cs);

    if (_skywatcher_initialize_axis(SA_AXIS_1) && _skywatcher_initialize_axis(SA_AXIS_2))
    {
        if (_skywatcher_get_timer_frequency(&_timer_frequency) && _skywatcher_get_cpr(SA_AXIS_1, &_cpr[SA_AXIS_1]) &&
                _skywatcher_get_cpr(SA_AXIS_2, &_cpr[SA_AXIS_2]))
        {
            _skywatcher_calculate_siderial_angular_speed_deg_per_s();
            _max_exposure_time_s = _skywatcher_calculate_max_exposure_time_s(pixel_size_um, focal_length_mm);
            is_ok = true;
        }
    }

    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_ra_siderial_speed()
{
    bool is_ok = false;

    if (skywatcher_set_speed(SA_AXIS_1, _siderial_w_deg_per_s + _siderial_w_error_1_level_deg_per_s))
    {
        if (skywatcher_set_motion_mode(SA_AXIS_1, true, false, false, false))
        {
            is_ok = skywatcher_start_motion(SA_AXIS_1);
        }
    }

    return is_ok;
}

bool skywatcher_set_ra_speed(double w_deg_per_s)
{
    bool is_ok = false;

    if (skywatcher_set_speed(SA_AXIS_1, fabs(w_deg_per_s)))
    {
        static int32_t last_direction = -1;
        bool direction = w_deg_per_s < 0.0;
        if ((last_direction == -1) || (last_direction != direction ? 1 : 0))
        {
            skywatcher_set_motion_mode(SA_AXIS_1, true, false, direction, false);
            last_direction = direction ? 1 : 0;
        }
        is_ok = skywatcher_start_motion(SA_AXIS_1);
    }
    else
    {
        is_ok = skywatcher_stop_motion(SA_AXIS_1);
    }

    return is_ok;
}

bool skywatcher_set_dec_speed(double w_deg_per_s)
{
    bool is_ok = false;

    if (skywatcher_set_speed(SA_AXIS_2, fabs(w_deg_per_s)))
    {
        static int32_t last_direction = -1;
        bool direction = w_deg_per_s < 0.0;
        if ((last_direction == -1) || (last_direction != direction ? 1 : 0))
        {
            skywatcher_set_motion_mode(SA_AXIS_2, true, false, direction, false);
            last_direction = direction ? 1 : 0;
        }
        is_ok = skywatcher_start_motion(SA_AXIS_2);
    }
    else
    {
        is_ok = skywatcher_stop_motion(SA_AXIS_2);
    }

    return is_ok;
}

bool skywatcher_set_axis_sleep(enum skywatcher_axis axis, bool sleep)
{
    data_t buffer_in = {0}, buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute_with_param_1(CMD_SLEEP, axis, sleep ? SAS_BLOCKED : SAS_NORMAL, buffer_in, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_set_axis_sleep");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_aux(enum skywatcher_axis axis, bool on)
{
    data_t buffer_in = {0}, buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute_with_param_1(CMD_SET_AUX, axis, on ? '1' : '0', buffer_in, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_set_aux");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_start_motion(enum skywatcher_axis axis)
{
    data_t buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_START_MOTION, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_start_motion");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_stop_motion(enum skywatcher_axis axis)
{
    data_t buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_STOP_MOTION, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_stop_motion");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_instant_stop(enum skywatcher_axis axis)
{
    data_t buffer_out = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_INSTANT_STOP, axis, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_instant_stop");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_axis_status(enum skywatcher_axis axis, skywatcher_axis_status_t status)
{
    data_t buffer_out = {0};
    threading_lock_critical_section(&_cs);
    bool is_ok = _skywatcher_get_axis_status(axis, buffer_out);
    char db_1 = buffer_out[1];
    char db_2 = buffer_out[2];
    char db_3 = buffer_out[3];
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
    data_t buffer_in = {0}, buffer_out = {0};
    uint32_t param_1 = BIT_2 | (fast ? BIT_1 : 0) | (tracking ? BIT_0 : 0);
    uint32_t param_2 = (south ? BIT_1 : 0) | (ccw ? BIT_0 : 0);
    threading_lock_critical_section(&_cs);
    _skywatcher_execute_with_param_2(CMD_SET_MOTION_MODE, axis, param_1, param_2, buffer_in, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_set_motion_mode");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_speed(enum skywatcher_axis axis, double* angular_speed_degrees_per_s)
{
    data_t buffer_out = {0};
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_SPEED, axis, buffer_out);
    _skywatcher_get_buffer_out(value, buffer_out);
    char* endptr = NULL;
    uint32_t preset = strtol(value, &endptr, 16);
    double counts_per_s = _timer_frequency / (double) preset;
    *angular_speed_degrees_per_s = counts_per_s * 360.0 / (double) _cpr[axis];
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_get_speed");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_speed(enum skywatcher_axis axis, double angular_speed_degrees_per_s)
{
    data_t buffer_in = {0}, buffer_out = {0};
    bool is_ok = false;
    threading_lock_critical_section(&_cs);
    if (angular_speed_degrees_per_s >= 1.0e-6)
    {
        data_t value = {0}, value_shift = {0};
        double counts_per_s = angular_speed_degrees_per_s * _cpr[axis] / 360.0;
        double preset = _timer_frequency / counts_per_s;
        sprintf(value, "%x", (int32_t) preset);
        
        uint32_t value_lenght = strlen(value);
        memset(value_shift, '0', 6);
        for (uint32_t i = 0; i < value_lenght; ++i)
        {
            value_shift[5 - i] = value[value_lenght - 1 - i];
        }

        sprintf(buffer_in, CMD_SET_SPEED, axis);
        _skywatcher_put_buffer_to_buffer(buffer_in, value_shift);

        if (socket_send(_socket, buffer_in, strlen(buffer_in)))
        {
            socket_receive(_socket, buffer_out, sizeof(data_t));
        }
        
        is_ok = skywatcher_check_error(buffer_out, "skywatcher_set_speed");
    }
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_position(enum skywatcher_axis axis, int32_t* position)
{
    data_t buffer_out = {0};
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_POSITION, axis, buffer_out);
    _skywatcher_get_buffer_out(value, buffer_out);
    *position = _skywatcher_convert_position_data(value);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_get_position");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_position(enum skywatcher_axis axis, int32_t position)
{
    data_t buffer_out = {0};
    data_t value = {0}, buffer = {0};
    sprintf(buffer, CMD_SET_POSITION, axis);
    sprintf(value, "%x", position + POSITION_OFFSET);
    _skywatcher_put_buffer_to_buffer(buffer, value);
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(buffer, SA_NONE, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_set_position");
    if (is_ok)
    {
        char string[256] = {0};
        sprintf(string, "position axis %d: %d", axis, position);
        logging_log_message(string);
    }
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_axis_position(enum skywatcher_axis axis, int32_t* position)
{
    data_t buffer_out = {0};
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_AXIS_POSITION, axis, buffer_out);
    _skywatcher_get_buffer_out(value, buffer_out);
    *position = _skywatcher_convert_position_data(value);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_get_axis_position");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_set_goto_target(enum skywatcher_axis axis, int32_t target)
{
    data_t buffer_out = {0};
    data_t value = {0}, buffer = {0};
    sprintf(buffer, CMD_SET_GOTO_TARGET, axis);
    sprintf(value, "%x", target + POSITION_OFFSET);
    _skywatcher_put_buffer_to_buffer(buffer, value);
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(buffer, SA_NONE, buffer_out);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_set_goto_target");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}

bool skywatcher_get_goto_target(enum skywatcher_axis axis, int32_t* target)
{
    data_t buffer_out = {0};
    data_t value = {0};
    threading_lock_critical_section(&_cs);
    _skywatcher_execute(CMD_GET_GOTO_TARGET, axis, buffer_out);
    _skywatcher_get_buffer_out(value, buffer_out);
    *target = _skywatcher_convert_position_data(value);
    bool is_ok = skywatcher_check_error(buffer_out, "skywatcher_get_goto_target");
    threading_unlock_critical_section(&_cs);
    return is_ok;
}
