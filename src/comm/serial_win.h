#ifndef SERIAL_WIN_H
#define SERIAL_WIN_H


#include <stdint.h>
#include <stddef.h>

#include "comm/comm_if.h"

typedef struct SerialPort SerialPort;
/* 打开串口（8N1，无流控）。返回句柄，失败返回 NULL。
 * 注意：串口名由 main.c 枚举 SERIALCOMM 得出并覆盖 ini，换机器不用改 ini。 */
SerialPort *serial_open(const char *port_name, uint32_t baudrate);

/* 关闭串口并释放句柄。 */
void serial_close(SerialPort *port);

/* 写。实测推帧 0.15ms/13 字节（只写不等响应）。 */
int serial_write(SerialPort *port, const uint8_t *data, size_t len);

/* 读。⚠️ max_len 必须正好等于期望字节数：请求多了读不满，只能等
 * ReadIntervalTimeout，而 CH341SER 挂在 ~15.6ms 节拍 ⇒ 每笔白等一节拍。 */
int serial_read(SerialPort *port, uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/* 设超时。CH340 无 LatencyTimer（那是 FTDI 才有的选项）。 */
void serial_set_timeout(SerialPort *port, uint32_t read_ms, uint32_t write_ms);

/* 清空收发缓冲（PURGE_RXCLEAR/TXCLEAR）。
 * ⚠️ bus_drain() 里必须先 Sleep(20) 再 flush()，顺序不能反。 */
void serial_flush(SerialPort *port);

/* 串口是否已打开。 */
int serial_is_open(const SerialPort *port);

/* 切【PC 侧】波特率。⚠️ 驱动器侧不跟着改就当场失联。 */
int serial_set_baud(uint32_t baudrate);

/* 取当前 PC 侧波特率（looptest 测完要改回去）。 */
uint32_t serial_get_baud(void);

/* 控 RTS 电平。用于 485 收发方向切换的探针。 */
int serial_set_rts(int mode);

/* 读 RTS 电平。 */
int serial_get_rts(void);

/* 串口后端的 CommOps 实例，启动时交给 modbus_comm_set()。 */
extern const CommOps serial_comm_ops;

#endif
