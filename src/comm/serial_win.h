#ifndef SERIAL_WIN_H
#define SERIAL_WIN_H

/*
 * Windows 串口封装（CreateFile / SetCommState / ReadFile / WriteFile）
 * 零第三方依赖，仅链接 Windows 系统库。
 * 波特率/数据位/校验/停止位在打开时配置，支持读写超时。
 */

#include <stdint.h>
#include <stddef.h>

typedef struct SerialPort SerialPort;

/* 打开串口，成功返回句柄对象，失败返回 NULL
 * port_name 形如 "COM3"（Windows 10 以上建议 "\\\\.\\COM3" 亦可）
 * baudrate 波特率，如 115200 */
SerialPort *serial_open(const char *port_name, uint32_t baudrate);

/* 关闭并释放串口 */
void serial_close(SerialPort *port);

/* 写数据，成功返回写入字节数，失败返回 -1 */
int serial_write(SerialPort *port, const uint8_t *data, size_t len);

/* 读数据，最多 max_len 字节，timeout_ms 内等待；返回实际读取字节数，失败/超时返回 0 或 -1 */
int serial_read(SerialPort *port, uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/* 设置读写超时（ms），默认读取由调用方传入 */
void serial_set_timeout(SerialPort *port, uint32_t read_ms, uint32_t write_ms);

/* 清空收发缓冲区 */
void serial_flush(SerialPort *port);

/* 返回句柄是否有效 */
int serial_is_open(const SerialPort *port);

#endif /* SERIAL_WIN_H */
