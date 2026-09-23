#ifndef SERIAL_WIN_H
#define SERIAL_WIN_H

/*
 * Windows 串口封装（CreateFile / SetCommState / ReadFile / WriteFile）
 * 零第三方依赖，仅链接 Windows 系统库。
 * 波特率/数据位/校验/停止位在打开时配置，支持读写超时。
 */

#include <stdint.h>
#include <stddef.h>

#include "comm/comm_if.h"

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

/* ==================== 回环测试专用（looptest） ====================
 * ⚠️ 只作用于【当前已打开的 CommOps 串口】，且改的是【PC 侧】波特率 ——
 * 驱动器侧不会跟着变，所以【只在脱离电机（A/B 接成回环）时用】。
 * 接电机时调用会立刻失联；测完必须用 serial_set_baud 改回原值。
 * 返回 0=成功，非 0=失败（转换器/驱动不支持该波特率时失败）。 */
int serial_set_baud(uint32_t baudrate);

/* 读回当前 PC 侧波特率；未打开或失败返回 0 */
uint32_t serial_get_baud(void);

/* 回环测试专用：直接设定 RTS 电平控制方式。
 * mode: 0=RTS_CONTROL_DISABLE(常低) / 1=RTS_CONTROL_ENABLE(常高) /
 *       2=RTS_CONTROL_TOGGLE(发送时高，程序默认，接电机就靠它)
 * 【为什么要试三种】很多 USB-RS485 模块拿 RTS 当收发方向控制(DE)，
 * 若 RE 与 DE 联动，发送期间接收被关掉 ⇒ A/B 自收自发收不到任何字节。
 * 换一种 RTS 电平可能就让接收重新打开。返回 0=成功。 */
int serial_set_rts(int mode);

/* 读回当前 RTS 控制方式（0/1/2，同 serial_set_rts）；失败返回 -1 */
int serial_get_rts(void);

/* ================= CommOps 适配 =================
 * serial_win 以全局静态句柄实现 CommOps 5 个操作，供上层统一注入。
 * 串口格式固定 8N1（由 serial_open 内部配置）。 */
extern const CommOps serial_comm_ops;

#endif /* SERIAL_WIN_H */
