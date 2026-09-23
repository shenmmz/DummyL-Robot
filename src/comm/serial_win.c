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
    uint32_t last_read_timeout;   /* 上次设置的读超时，避免重复调用 SetCommTimeouts */
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
    /* RTS_CONTROL_TOGGLE: Windows 自动在发送时置高、发送后置低，
     * 用于 RS485 自动方向切换（需转换器硬件支持）。 */
    dcb.fRtsControl = RTS_CONTROL_TOGGLE;
    if (!SetCommState(port->h, &dcb)) {
        fprintf(stderr, "[串口] SetCommState 失败\n");
        serial_close(port);
        return NULL;
    }

    /* 读写超时 */
    memset(&timeouts, 0, sizeof(timeouts));
    /* ReadIntervalTimeout：帧内相邻字节最大间隔，超过即认为一帧结束并让 ReadFile 返回。
     * 115200 下字节间隔约 0.087ms，USB-RS485 以 1ms 粒度上送，取 2ms 足够；
     * 此值越小，读完一帧后返回越快，轮询周期越短。 */
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
    BOOL ok;

    if (port == NULL || port->h == INVALID_HANDLE_VALUE || port->h == NULL) {
        return -1;
    }
    /* 只有超时值变化时才重设，避免每次都走系统调用（USB串口上开销不小） */
    if (timeout_ms != port->last_read_timeout) {
        COMMTIMEOUTS t = port->timeouts;
        t.ReadTotalTimeoutConstant = timeout_ms;
        SetCommTimeouts(port->h, &t);
        port->last_read_timeout = timeout_ms;
    }

    ok = ReadFile(port->h, buf, (DWORD)max_len, &got, NULL);
    return ok ? (int)got : -1;
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
    port->last_read_timeout = read_ms;
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

/* ================= CommOps 适配实现 ================= */

/* 全局静态句柄：CommOps 无句柄参数，适配层自持当前打开的串口 */
static SerialPort *g_ops_port = NULL;

static void ops_close(void);   /* 前向声明：ops_open 在替换旧句柄前关闭 */

/* ops_open：打开串口（固定 8N1，由 serial_open 内部配置） */
static int ops_open(const char *port, uint32_t baud)
{
    if (g_ops_port != NULL) {
        ops_close();
    }
    g_ops_port = serial_open(port, baud);
    return g_ops_port != NULL ? 0 : -1;
}

/* ops_close：关闭当前串口并清空句柄 */
static void ops_close(void)
{
    if (g_ops_port != NULL) {
        serial_close(g_ops_port);
        g_ops_port = NULL;
    }
}

/* ops_read_frame：读一帧，返回实际读取字节数，失败/超时返回 <= 0 */
static int ops_read_frame(uint8_t *buf, int cap, int timeout_ms)
{
    if (g_ops_port == NULL || buf == NULL || cap <= 0) {
        return -1;
    }
    return serial_read(g_ops_port, buf, (size_t)cap, (uint32_t)timeout_ms);
}

/* ops_write_frame：写一帧，成功返回写入字节数，失败返回 -1 */
static int ops_write_frame(const uint8_t *buf, int len)
{
    if (g_ops_port == NULL || buf == NULL || len <= 0) {
        return -1;
    }
    return serial_write(g_ops_port, buf, (size_t)len);
}

/* ops_flush：清空收发缓冲 */
static void ops_flush(void)
{
    if (g_ops_port != NULL) {
        serial_flush(g_ops_port);
    }
}

/* ==================== 回环测试专用 ====================
 * 放在 g_ops_port 定义之后：这两个函数直接操作上层正在用的那一个串口句柄，
 * 不去重新 CreateFile —— Windows 串口独占，重复打开同一端口只会 Access denied。 */

/* serial_set_baud：运行中改 PC 侧波特率（只改 DCB.BaudRate，8N1 等格式不动）。
 * 【只给 looptest 用】改的是 PC 侧，驱动器侧不会跟着变 ⇒ 接电机时调用会立刻失联。
 * 切完顺手 PurgeComm：换速率瞬间的残帧若不清掉，会被当成"回环收到的数据"，
 * 让第一次测量凭空多出几个字节。 */
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

/* serial_get_baud：读回当前 PC 侧波特率，供 looptest 测完恢复 */
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

/* serial_set_rts：回环测试专用 —— 改 DCB.fRtsControl（其余格式不动）。
 * 只动 RTS 的电平控制方式，不改波特率，所以接电机时调用也不会立刻失联
 * （但把 DE 常拉高会挡住从站回帧，表现成"六轴全离线"，测完必须改回来）。 */
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

/* serial_get_rts：读回当前 RTS 控制方式，供 looptest 测完恢复 */
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
    default:                  return -1;   /* HANDSHAKE：程序不用，不归零 */
    }
}

/* 全局 CommOps 实例：上层通过它操作串口总线 */
const CommOps serial_comm_ops = {
    ops_open,
    ops_close,
    ops_read_frame,
    ops_write_frame,
    ops_flush,
};
