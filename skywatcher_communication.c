#include "types.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <api/api.h>
#include <socket/socket.h>
#include <threading/threading.h>
#include <logging/logging.h>

protected_import(threading_critical_section, _cs);

protected socket_handle_t _socket = SOCKET_INVALID_SOCKET;

private void _skywatcher_telegram(data_t buffer_in, data_t buffer_out)
{
    threading_lock_critical_section(&_cs);
    for (uint32_t retry_send = 0; retry_send < 5; ++retry_send)
    {
        bool has_data = false;
        if (socket_send(_socket, buffer_in, strlen(buffer_in)))
        {
            threading_sleep(TTR_MILLI, 10);
            if (socket_receive(_socket, buffer_out, sizeof(data_t)) > 0)
            {
                has_data = true;
                break;
            }
        }
    }
    threading_unlock_critical_section(&_cs);
}

protected void _skywatcher_execute(char* command, enum skywatcher_axis axis, data_t buffer_out)
{
    data_t buffer_in = {0};
    threading_lock_critical_section(&_cs);
    memset(buffer_out, 0, sizeof(data_t));
    if (axis != SA_NONE)
    {
        sprintf(buffer_in, command, axis);
    }
    else
    {
        memcpy(buffer_in, command, strlen(command));
    }
    _skywatcher_telegram(buffer_in, buffer_out);
    threading_unlock_critical_section(&_cs);
}

protected void _skywatcher_execute_with_param_1(char* command, enum skywatcher_axis axis, uint32_t value_1, data_t buffer_in, data_t buffer_out)
{
    threading_lock_critical_section(&_cs);
    memset(buffer_in, 0, sizeof(data_t));
    memset(buffer_out, 0, sizeof(data_t));
    sprintf(buffer_in, command, axis, value_1);
    _skywatcher_telegram(buffer_in, buffer_out);
    threading_unlock_critical_section(&_cs);
}

protected void _skywatcher_execute_with_param_2(char* command, enum skywatcher_axis axis, uint32_t value_1, uint32_t value_2, data_t buffer_in, data_t buffer_out)
{
    threading_lock_critical_section(&_cs);
    memset(buffer_in, 0, sizeof(data_t));
    memset(buffer_out, 0, sizeof(data_t));
    sprintf(buffer_in, command, axis, value_1, value_2);
    _skywatcher_telegram(buffer_in, buffer_out);
    threading_unlock_critical_section(&_cs);
}
