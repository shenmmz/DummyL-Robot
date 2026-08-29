#ifndef COMM_IF_H
#define COMM_IF_H

/*
 * comm_if.h —— 通信总线抽象接口
 * ------------------------------------------------------------
 * 上层（control）通过 CommOps 函数指针表操作总线，
 * 不直接依赖具体串口实现（serial_win）。
 *
 * 用法：调用方注入 CommOps 实例（如 serial_comm_ops），
 *       之后 modbus_transact 自动通过它收发数据。
 */

#include <stdint.h>

/* 总线操作函数指针表 */
typedef struct {
    /* 打开总线（固定 8N1 格式）
     * port - 设备名，如 "COM3"
     * baud - 波特率，如 115200
     * 返回 0=成功，非 0=失败 */
    int (*open)(const char *port, uint32_t baud);

    /* 关闭总线 */
    void (*close)(void);

    /* 读数据：最多 cap 字节，timeout_ms 内等待
     * 返回实际读取字节数，失败/超时返回 <= 0 */
    int (*read_frame)(uint8_t *buf, int cap, int timeout_ms);

    /* 写数据：len 字节
     * 返回实际写入字节数，失败返回 <= 0 */
    int (*write_frame)(const uint8_t *buf, int len);

    /* 清空收发缓冲 */
    void (*flush)(void);
} CommOps;

#endif /* COMM_IF_H */
