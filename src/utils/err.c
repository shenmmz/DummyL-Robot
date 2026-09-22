/*
 * err.c —— ErrCode 中文描述实现
 * ------------------------------------------------------------
 * 所属模块：utils
 * 对外接口：err_str
 */

#include "utils/err.h"

const char *err_str(ErrCode e)
{
    switch (e) {
    case ERR_NONE:      return "成功";
    case ERR_PORT:      return "串口打开或IO失败";
    case ERR_TIMEOUT:   return "收帧超时";
    case ERR_CRC:       return "CRC校验失败";
    case ERR_EXCEPTION: return "Modbus异常响应";
    case ERR_LEN:       return "帧长非法";
    case ERR_ARG:       return "参数非法";
    case ERR_OVERFLOW:  return "数据越界或缓冲不足";
    case ERR_MASKED:    return "关节已屏蔽，操作跳过";
    case ERR_ALARM:     return "驱动器报警，运动中断";
    case ERR_SUBDIV:    return "每转脉冲数未确认对齐，已拒绝下发运动";
    default:            return "未知错误";
    }
}
