#ifndef COMM_IF_H
#define COMM_IF_H


#include <stdint.h>

/* 串口后端接口表（策略模式）：modbus_rtu 只依赖本结构，不依赖 Win32。
 * 换后端（虚拟串口/网络转发）只需换一份 CommOps。 */
typedef struct {
    /* 打开串口并设置波特率。返回 0 成功、非 0 失败。 */
    int (*open)(const char *port, uint32_t baud);

    /* 关闭串口。 */
    void (*close)(void);

    /* 读一帧。⚠️ cap 必须【正好等于】期望字节数，否则读不满、要白等
     * 驱动节拍（CH341SER ~15.6ms）。返回实际读到的字节数，0/负 = 超时。 */
    int (*read_frame)(uint8_t *buf, int cap, int timeout_ms);

    /* 写一帧。返回写出的字节数。 */
    int (*write_frame)(const uint8_t *buf, int len);

    /* 清空收发缓冲。用于 bus_drain() 丢掉没人读走的残帧。 */
    void (*flush)(void);
} CommOps;

#endif
