/*
 * serial_win.c —— Windows 串口通信封装（Win32 API 实现）
 * ------------------------------------------------------------
 * 所属模块：通信层（comm）
 * 对外接口：serial_open、serial_close、serial_write、serial_read、
 *           serial_set_timeout、serial_flush、serial_is_open
 * 依赖模块：无（仅系统 API）
 */

#include "comm/serial_win.h"

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct SerialPort {
    HANDLE h;
    COMMTIMEOUTS timeouts;
    char name[64];
};

/* serial_open：打开串口并配置 8N1 与读写超时，失败返回 NULL */
SerialPort *serial_open(const char *port_name, uint32_t baudrate)
{
    SerialPort *port;
    DCB dcb;
    char full_name[72];
    COMMTIMEOUTS timeouts;

    if (port_name == NULL || port_name[0] == '\0') {
        return NULL;
    }

    port = (SerialPort *)calloc(1, sizeof(SerialPort));
    if (port == NULL) {
        return NULL;
    }
    snprintf(port->name, sizeof(port->name), "%s", port_name);

    /* 支持 "COM3" 与 "\\\\.\\COM3" 两种写法 */
    if (strncmp(port_name, "\\\\", 2) == 0 || strncmp(port_name, "\\\\.\\", 4) == 0) {
        snprintf(full_name, sizeof(full_name), "%s", port_name);
    } else {
        snprintf(full_name, sizeof(full_name), "\\\\.\\%s", port_name);
    }

    port->h = CreateFileA(full_name,
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          NULL,
                          OPEN_EXISTING,
                          0,
                          NULL);
    if (port->h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "[串口] 打开失败: %s (错误码 %lu)\n", full_name, (unsigned long)GetLastError());
        free(port);
        return NULL;
    }

    /* 配置 115200 8N1 */
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(port->h, &dcb)) {
        fprintf(stderr, "[串口] GetCommState 失败\n");
        serial_close(port);
        return NULL;
    }
    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(port->h, &dcb)) {
        fprintf(stderr, "[串口] SetCommState 失败\n");
        serial_close(port);
        return NULL;
    }

    /* 读写超时 */
    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 100;
    if (!SetCommTimeouts(port->h, &timeouts)) {
        fprintf(stderr, "[串口] SetCommTimeouts 失败\n");
        serial_close(port);
        return NULL;
    }

    PurgeComm(port->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return port;
}

/* serial_close：关闭串口句柄并释放对象 */
void serial_close(SerialPort *port)
{
    if (port == NULL) {
        return;
    }
    if (port->h != INVALID_HANDLE_VALUE && port->h != NULL) {
        CloseHandle(port->h);
        port->h = INVALID_HANDLE_VALUE;
    }
    free(port);
}

/* serial_write：向串口写入 len 字节，成功返回实际写入数，失败返回 -1 */
int serial_write(SerialPort *port, const uint8_t *data, size_t len)
{
    DWORD written = 0;
    BOOL ok;

    if (port == NULL || port->h == INVALID_HANDLE_VALUE || port->h == NULL) {
        return -1;
    }
    ok = WriteFile(port->h, data, (DWORD)len, &written, NULL);
    return ok ? (int)written : -1;
}

/* serial_read：按 timeout_ms 超时读取数据到 buf，返回实际读取字节数 */
int serial_read(SerialPort *port, uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    DWORD got = 0;
    COMMTIMEOUTS t;

    if (port == NULL || port->h == INVALID_HANDLE_VALUE || port->h == NULL) {
        return -1;
    }
    /* 每次读取前按调用方超时调整 */
    t = port->timeouts;
    t.ReadTotalTimeoutConstant = timeout_ms;
    SetCommTimeouts(port->h, &t);

    if (!ReadFile(port->h, buf, (DWORD)max_len, &got, NULL)) {
        return -1;
    }
    return (int)got;
}

/* serial_set_timeout：设置串口读写超时并生效 */
void serial_set_timeout(SerialPort *port, uint32_t read_ms, uint32_t write_ms)
{
    COMMTIMEOUTS t;
    if (port == NULL) {
        return;
    }
    memset(&t, 0, sizeof(t));
    t.ReadIntervalTimeout = 10;
    t.ReadTotalTimeoutMultiplier = 1;
    t.ReadTotalTimeoutConstant = read_ms;
    t.WriteTotalTimeoutMultiplier = 1;
    t.WriteTotalTimeoutConstant = write_ms;
    port->timeouts = t;
    SetCommTimeouts(port->h, &t);
}

/* serial_flush：清空串口收发缓冲区 */
void serial_flush(SerialPort *port)
{
    if (port == NULL || port->h == INVALID_HANDLE_VALUE || port->h == NULL) {
        return;
    }
    PurgeComm(port->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

/* serial_is_open：判断串口是否已打开，是返回 1 */
int serial_is_open(const SerialPort *port)
{
    return (port != NULL && port->h != INVALID_HANDLE_VALUE && port->h != NULL) ? 1 : 0;
}
