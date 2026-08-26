#ifndef ROBOT_H
#define ROBOT_H

/*
 * DummyL-Robot 高层控制接口
 * ------------------------------------------------------------
 * 封装立三（LEESN）闭环步进驱动器 Modbus RTU 操作（手册 V126）：
 *   robot_init  打开串口并初始化
 *   robot_enable / robot_disable  使能/失能关节（0x00D4：写0使能、写1释放）
 *   robot_movej 单关节绝对运动（角度 -> 脉冲，写 0x00E8~0x00E9）
 *   robot_status 查询关节状态（0x0006~0x0007 位定义）
 * 回零流程见 home.h（堵转/碰撞模式回零）
 * 运动学/轨迹规划由上层（main）调用 kinematics/trajectory 完成，
 * 本层只负责"关节 -> 电机指令"。
 * 注意：Zeta 协议已作废，本文件及实现一律使用立三寄存器地址。
 */

#include <stdint.h>
#include "utils/err.h"

typedef struct Robot Robot;

/* 打开串口并初始化总线；port_name 如 "COM3"，baudrate 默认 115200。
 * 成功返回对象，失败返回 NULL。 */
Robot *robot_init(const char *port_name, uint32_t baudrate);

void robot_close(Robot *robot);

/* 使能/失能关节（1..6）：写 0x00D4，0=马达使能、1=释放马达。
 * 返回 ErrCode：ERR_NONE 成功 / ERR_ARG 参数非法 / 通信错误透传 */
ErrCode robot_enable(Robot *robot, int joint);
ErrCode robot_disable(Robot *robot, int joint);

/* 单关节绝对运动：角度（度）-> 脉冲，写 0x00E8~0x00E9（INT32，
 * 运行中亦可执行）；速度参数写 0x00D8~0x00D9（0.01 rpm）。
 * 返回 ErrCode。 */
ErrCode robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 查询关节在线状态（读 0x0006~0x0007 状态字）：返回 1 在线，0 离线/失败 */
int robot_is_online(Robot *robot, int joint);

/* 读取关节状态字低 16 位（0x0006~0x0007，UINT32）。
 * 成功返回 ERR_NONE 并置 *status；失败返回对应 ErrCode。 */
ErrCode robot_read_status(Robot *robot, int joint, uint16_t *status);

/* 读取关节完整 32 位状态字（0x0006~0x0007，UINT32 低字在前）。
 * 成功返回 ERR_NONE 并置 *status；失败返回对应 ErrCode。
 * 包含 bit8~9 运行 / bit12 到位 / bit13~14 软限位 / bit15 原点 /
 * bit16 使能电平 / bit21 报警（位定义见 robot_internal.h LEESN_STAT_*）。 */
ErrCode robot_read_status32(Robot *robot, int joint, uint32_t *status);

/* 读取关节当前位置（脉冲，0x0004~0x0005），失败返回 0 并置 *ok=0 */
int32_t robot_read_position_steps(Robot *robot, int joint, int *ok);

/* 读取关节当前电流（mA，0x001A），失败返回 -1 */
int robot_read_current_ma(Robot *robot, int joint);

/* 屏蔽/恢复关节（1=屏蔽）：屏蔽后所有操作自动跳过该关节，不发指令、不轮询。
 * 返回 ErrCode：ERR_NONE / ERR_ARG */
ErrCode robot_mask(Robot *robot, int joint);
ErrCode robot_unmask(Robot *robot, int joint);

/* 关节是否被屏蔽：1 屏蔽 / 0 未屏蔽 */
int robot_is_masked(const Robot *robot, int joint);

#endif /* ROBOT_H */
