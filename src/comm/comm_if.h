#ifndef COMM_IF_H
#define COMM_IF_H

/*
 * comm_if.h —— 通信层抽象接口（CommOps）
 * ------------------------------------------------------------
 * control 层唯一可见的 comm 头文件。上层通过全局 CommOps 函数指针表
 * 操作总线（打开/关闭/读帧/写帧/清缓冲），不直接依赖具体串口实现
 * （serial_win / modbus_rtu 细节）。
 *
 * 注入方式：调用方构造 CommOps 实例后交给 modbus_comm_set()/总线初始化
 * 函数，模块内部以全局指针持有。
 * 设计约束：纯 C 函数指针表，可离线注入内存假串口进行单测（不接硬件）。
 */

#include <stdint.h>

/* 通信操作函数指针表（纯 C，无对象句柄；实现方自持全局状态） */
typedef struct {
    /* 打开总线。port 为设备名（如 "COM3"），baud 波特率；
     * data_bits/parity/stop_bits 为 8/N/1 等串口参数。
     * 成功返回 0，失败返回非 0。 */
    int (*open)(const char *port, uint32_t baud,
                uint8_t data_bits, char parity, uint8_t stop_bits);

    /* 关闭总线 */
    void (*close)(void);

    /* 读一帧：最多 cap 字节，timeout_ms 内等待；返回实际读取字节数，
     * 失败/超时返回 <= 0。 */
    int (*read_frame)(uint8_t *buf, int cap, int timeout_ms);

    /* 写一帧：len 字节；成功返回实际写入字节数，失败返回 <= 0 */
    int (*write_frame)(const uint8_t *buf, int len);

    /* 清空收发缓冲 */
    void (*flush)(void);
} CommOps;

#endif /* COMM_IF_H */
