#ifndef ERR_H
#define ERR_H

/*
 * err.h —— 全项目统一错误码（ErrCode）与中文描述
 * ------------------------------------------------------------
 * 错误处理单一来源：各层函数失败返回对应 ErrCode，成功返回 ERR_NONE。
 * 调用方不再依赖具体数字（原 -1/0），统一以 err_str() 输出中文提示。
 */

typedef enum {
    ERR_NONE      = 0,  /* 成功 */
    ERR_PORT      = 1,  /* 串口打开/IO 失败 */
    ERR_TIMEOUT   = 2,  /* 收帧超时 */
    ERR_CRC       = 3,  /* CRC 校验失败 */
    ERR_EXCEPTION = 4,  /* Modbus 异常码 01~04 */
    ERR_LEN       = 5,  /* 帧长非法 */
    ERR_ARG       = 6,  /* 参数非法 */
    ERR_OVERFLOW  = 7,  /* 数据越界/缓冲不足 */
    ERR_MASKED    = 8,  /* 关节被屏蔽，操作跳过（非失败） */
} ErrCode;

/* 错误码 -> 中文描述；未知码返回"未知错误" */
const char *err_str(ErrCode e);

#endif /* ERR_H */
