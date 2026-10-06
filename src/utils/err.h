#ifndef ERR_H
#define ERR_H


/**
 * @enum ErrCode
 * @brief 全局错误码：串口/Modbus/参数/运动各层的失败统一归集到本枚举。
 */
typedef enum {
    ERR_NONE      = 0,   /* 成功 */
    ERR_PORT      = 1,   /* 串口打开或 IO 失败 */
    ERR_TIMEOUT   = 2,   /* 收帧超时 */
    ERR_CRC       = 3,   /* CRC16 校验失败 */
    ERR_EXCEPTION = 4,   /* 驱动器回 Modbus 异常响应（功能码 + 0x80）*/
    ERR_LEN       = 5,   /* 帧长度非法 */
    ERR_ARG       = 6,   /* 参数非法（关节号/角度/寄存器地址越界等）*/
    ERR_OVERFLOW  = 7,   /* 数据越界或缓冲不足 */
    ERR_MASKED    = 8,   /* 关节已屏蔽，操作跳过 */
    ERR_ALARM     = 9,   /* 驱动器报警，运动中断 */
    ERR_SUBDIV    = 10,  /* 每转脉冲数未确认对齐，已拒绝下发运动 */
} ErrCode;

/* 错误码转中文串。ERR_NONE → "OK"。 */
const char *err_str(ErrCode e);

#endif
