#ifndef MOTOR_REG_H
#define MOTOR_REG_H

/*
 * motor_reg.h —— 电机寄存器 API（立三 LEESN 485 驱动器）
 * ------------------------------------------------------------
 * 所属模块：API 层（api）
 * 功能：封装常用寄存器读写，屏蔽 Modbus 帧细节，
 *       供 control / monitor 等模块统一调用。
 *
 * 约定：所有函数 joint 参数为 1~6 关节号，
 *       内部自动查表转换为 Modbus 从站地址。
 *       成功返回 ERR_NONE，失败返回对应错误码。
 */

#include "control/robot.h"
#include <stdint.h>

/* ================= 基本寄存器读写 ================= */

/* 写单个 16 位寄存器（功能码 06H）
 * 例：motor_write_u16(robot, 1, 0x00D4, 0) —— 关节1使能 */
ErrCode motor_write_u16(Robot *robot, int joint, uint16_t reg, uint16_t val);

/* 写 32 位寄存器（功能码 10H，低 16 位寄存器在前）
 * 例：motor_write_i32(robot, 1, 0x00D8, 30000) —— 关节1设速度300rpm */
ErrCode motor_write_i32(Robot *robot, int joint, uint16_t reg, int32_t val);

/* 读单个 16 位寄存器（功能码 03H），结果存到 *val */
ErrCode motor_read_u16(Robot *robot, int joint, uint16_t reg, uint16_t *val);

/* 读 32 位寄存器（功能码 03H，低 16 位寄存器在前），结果存到 *val */
ErrCode motor_read_i32(Robot *robot, int joint, uint16_t reg, int32_t *val);

/* ================= 常用电机操作 ================= */

/* 使能电机（0x00D4 = 0），上电后必须先使能才能运动 */
ErrCode motor_enable(Robot *robot, int joint);

/* 释放/失能电机（0x00D4 = 1），电机断电自由转动 */
ErrCode motor_disable(Robot *robot, int joint);

/* 急停（0x00C8 = 0x0100），立即停止，保留使能 */
ErrCode motor_estop(Robot *robot, int joint);

/* 设置运行速度 rpm（0x00D8~0x00D9，范围 ±9999.99 rpm） */
ErrCode motor_set_speed(Robot *robot, int joint, int rpm);

/* 设置加减速时间 ms（同时写 0x0098 加速和 0x0099 减速）
 * 值越大启停越平缓，越小越猛 */
ErrCode motor_set_accel(Robot *robot, int joint, int ms);

/* 清零当前位置（0x00D2 = 0），把当前位置设为坐标原点 */
ErrCode motor_clear_pos(Robot *robot, int joint);

/* 设置限位使能（0x006D，enable=1有效 0=失效）
 * 回零时关闭限位，正常运行时开启 */
ErrCode motor_set_limit(Robot *robot, int joint, int enable);

/* 速度模式运行（0x00C8），dir > 0 正转，dir < 0 反转
 * 注意：先调用 motor_set_speed 设置速度再启动 */
ErrCode motor_run(Robot *robot, int joint, int dir);

/* 读实时电流 mA（0x001A），失败返回 -1 */
int motor_read_current(Robot *robot, int joint);

/* 读实时位置脉冲（0x0004~0x0005），ok=1 成功 0 失败 */
int32_t motor_read_position(Robot *robot, int joint, int *ok);

/* 读状态字低 16 位（0x0006），包含运行状态、到位、限位等标志 */
ErrCode motor_read_status(Robot *robot, int joint, uint16_t *status);

/* ================= 运动控制 ================= */

/* 绝对位置运动（0x00E8~0x00E9，INT32 脉冲）
 * 电机运行到目标绝对位置，运行中也可执行
 * steps - 目标位置脉冲数（由 DEG2STEPS 计算） */
ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps);

/* ================= 状态读取（扩展） ================= */

/* 读实时速度 rpm（0x0019，INT32，单位 0.01rpm），失败返回 -1 */
int motor_read_speed(Robot *robot, int joint);

/* 读报警代码（0x00A3），0=正常，>0=报警代码，失败返回 -1 */
int motor_read_alarm(Robot *robot, int joint);

/* 清除报警（0x00A4 = 0） */
ErrCode motor_clear_alarm(Robot *robot, int joint);

/* 读驱动器地址（0x0066），用于检测电机是否在线，失败返回 -1 */
int motor_read_device_addr(Robot *robot, int joint);

#endif /* MOTOR_REG_H */
