
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
    uint32_t last_read_timeout;
    char name[64];
};

/* 打开串口（本机是 CH340 / CH341SER 驱动）。
 * ⚠️ CH340 注册表里【没有】FTDI 那种 LatencyTimer 项，所以"改设备管理器延迟计时器"
 * 这条路对它根本不存在 —— 当年正是这个误导把排查方向带偏了。 */
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
    dcb.fRtsControl = RTS_CONTROL_TOGGLE;
    if (!SetCommState(port->h, &dcb)) {
        fprintf(stderr, "[串口] SetCommState 失败\n");
        serial_close(port);
        return NULL;
    }

    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = 2;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    if (!SetCommTimeouts(port->h, &timeouts)) {
        fprintf(stderr, "[串口] SetCommTimeouts 失败\n");
        serial_close(port);
        return NULL;
    }
    port->timeouts = timeouts;
    port->last_read_timeout = timeouts.ReadTotalTimeoutConstant;

    PurgeComm(port->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return port;
}

/* 关闭串口。 */
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

/* 写数据到串口。 */
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

/* 从串口读数据。
 * ⚠️ 关键约束：**max_len 必须等于实际会到达的字节数**。
 * 请求长度大于实际长度时 ReadFile 不会完成，只能等 ReadIntervalTimeout，
 * 而 CH341SER 把它挂在 ~15.6ms 节拍上 ⇒ 每笔白等一节拍。
 * 详见 modbus_read_reply()。 */
int serial_read(SerialPort *port, uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    DWORD got = 0;
    BOOL ok;

    if (port == NULL || port->h == INVALID_HANDLE_VALUE || port->h == NULL) {
        return -1;
    }
    if (timeout_ms != port->last_read_timeout) {
        COMMTIMEOUTS t = port->timeouts;
        t.ReadTotalTimeoutConstant = timeout_ms;
        SetCommTimeouts(port->h, &t);
        port->last_read_timeout = timeout_ms;
    }

    ok = ReadFile(port->h, buf, (DWORD)max_len, &got, NULL);
    return ok ? (int)got : -1;
}

/* 设置读/写超时。 */
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
    port->last_read_timeout = read_ms;
    SetCommTimeouts(port->h, &t);
}

/* PurgeComm 清收发缓冲。 */
void serial_flush(SerialPort *port)
{
    if (port == NULL || port->h == INVALID_HANDLE_VALUE || port->h == NULL) {
        return;
    }
    PurgeComm(port->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

/* 串口是否已打开。 */
int serial_is_open(const SerialPort *port)
{
    return (port != NULL && port->h != INVALID_HANDLE_VALUE && port->h != NULL) ? 1 : 0;
}


static SerialPort *g_ops_port = NULL;

static void ops_close(void);

/* CommOps.open 适配（仅本文件用）。 */
static int ops_open(const char *port, uint32_t baud)
{
    if (g_ops_port != NULL) {
        ops_close();
    }
    g_ops_port = serial_open(port, baud);
    return g_ops_port != NULL ? 0 : -1;
}

/* CommOps.close 适配（仅本文件用）。 */
static void ops_close(void)
{
    if (g_ops_port != NULL) {
        serial_close(g_ops_port);
        g_ops_port = NULL;
    }
}

/* CommOps.read_frame 适配（仅本文件用）。 */
static int ops_read_frame(uint8_t *buf, int cap, int timeout_ms)
{
    if (g_ops_port == NULL || buf == NULL || cap <= 0) {
        return -1;
    }
    return serial_read(g_ops_port, buf, (size_t)cap, (uint32_t)timeout_ms);
}

/* CommOps.write_frame 适配（仅本文件用）。 */
static int ops_write_frame(const uint8_t *buf, int len)
{
    if (g_ops_port == NULL || buf == NULL || len <= 0) {
        return -1;
    }
    return serial_write(g_ops_port, buf, (size_t)len);
}

/* CommOps.flush 适配（仅本文件用）。 */
static void ops_flush(void)
{
    if (g_ops_port != NULL) {
        serial_flush(g_ops_port);
    }
}


/* 运行中修改 PC 侧波特率。
 * ⚠️ 驱动器侧改波特率是【写完立即生效】，PC 侧必须同时改，否则当场失联。
 * ⚠️ 六台必须【广播一起写】（逐台写第一台就断）。
 * ⚠️ 不固化（0x00DC=1）则断电回 115200，而 ini 已是新速率 ⇒ 下次上电失联。 */
int serial_set_baud(uint32_t baudrate)
{
    DCB dcb;

    if (g_ops_port == NULL || g_ops_port->h == INVALID_HANDLE_VALUE || g_ops_port->h == NULL) {
        return -1;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(g_ops_port->h, &dcb)) {
        return -1;
    }
    dcb.BaudRate = baudrate;
    if (!SetCommState(g_ops_port->h, &dcb)) {
        return -1;
    }
    PurgeComm(g_ops_port->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return 0;
}

/* 取当前波特率。判读文案必须用它，不要硬编码 MODBUS_BAUDRATE（改 ini 后会自相矛盾）。 */
uint32_t serial_get_baud(void)
{
    DCB dcb;

    if (g_ops_port == NULL || g_ops_port->h == INVALID_HANDLE_VALUE || g_ops_port->h == NULL) {
        return 0;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(g_ops_port->h, &dcb)) {
        return 0;
    }
    return (uint32_t)dcb.BaudRate;
}

/* 设置 RTS 电平（looptest 用）。
 * ⚠️ RTS 停在 ENABLE 会让六轴全部"离线"。 */
int serial_set_rts(int mode)
{
    DCB dcb;

    if (g_ops_port == NULL || g_ops_port->h == INVALID_HANDLE_VALUE || g_ops_port->h == NULL) {
        return -1;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(g_ops_port->h, &dcb)) {
        return -1;
    }
    switch (mode) {
    case 0:  dcb.fRtsControl = RTS_CONTROL_DISABLE; break;
    case 1:  dcb.fRtsControl = RTS_CONTROL_ENABLE;  break;
    default: dcb.fRtsControl = RTS_CONTROL_TOGGLE;  break;
    }
    if (!SetCommState(g_ops_port->h, &dcb)) {
        return -1;
    }
    PurgeComm(g_ops_port->h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return 0;
}

/* 读当前 RTS 电平。 */
int serial_get_rts(void)
{
    DCB dcb;

    if (g_ops_port == NULL || g_ops_port->h == INVALID_HANDLE_VALUE || g_ops_port->h == NULL) {
        return -1;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(g_ops_port->h, &dcb)) {
        return -1;
    }
    switch (dcb.fRtsControl) {
    case RTS_CONTROL_DISABLE: return 0;
    case RTS_CONTROL_ENABLE:  return 1;
    case RTS_CONTROL_TOGGLE:  return 2;
    default:                  return -1;
    }
}

const CommOps serial_comm_ops = {
    ops_open,
    ops_close,
    ops_read_frame,
    ops_write_frame,
    ops_flush,
};
