#ifndef MOTOR_REG_H
#define MOTOR_REG_H


#include "control/robot.h"
#include <stdint.h>


/* 写 16 位寄存器（功能码 06H，等响应）。 */
ErrCode motor_write_u16(Robot *robot, int joint, uint16_t reg, uint16_t val);

/* 写 32 位寄存器（功能码 10H，2 个连续寄存器，低字在前）。 */
ErrCode motor_write_i32(Robot *robot, int joint, uint16_t reg, int32_t val);

/* 读 16 位寄存器（功能码 03H）。 */
ErrCode motor_read_u16(Robot *robot, int joint, uint16_t reg, uint16_t *val);

/* 读 32 位寄存器（功能码 03H，2 个连续寄存器，响应低字在前）。 */
ErrCode motor_read_i32(Robot *robot, int joint, uint16_t reg, int32_t *val);


/* 马达使能（0x00D4=0）。 */
ErrCode motor_enable(Robot *robot, int joint);

/* 释放马达 / 脱机（0x00D4=1）。 */
ErrCode motor_disable(Robot *robot, int joint);

/* 急停（0x00C8=0x0100）。 */
ErrCode motor_estop(Robot *robot, int joint);

/* 减速停止（0x00C8=0x0000）。 */
ErrCode motor_stop_slow(Robot *robot, int joint);

/* 写运行速度 0x00D8（寄存器单位 0.01rpm，入参是 rpm）。 */
ErrCode motor_set_speed(Robot *robot, int joint, double rpm);

/* 写连续运行速度 0x009A（整数 rpm）。 */
ErrCode motor_set_speed16(Robot *robot, int joint, int rpm);

/* 一次写加速/减速时间（0x0098 起连续 2 个寄存器，单位 ms）。 */
ErrCode motor_set_profile(Robot *robot, int joint, int accel_ms, int decel_ms);

/* 设置当前电机位置 = 0（0x00D2）。 */
ErrCode motor_clear_pos(Robot *robot, int joint);

/* 断电保存（0x00DC=1，写 0 是恢复出厂）。 */
ErrCode motor_save_params(Robot *robot, int joint);

/* 关掉位置超差报警（0x000B/0x000C 写 0）。 */
ErrCode motor_disable_pos_err_alarm(Robot *robot, int joint);

/* 恢复位置超差报警的出厂阈值（动态 200 / 静态 100，单位 1.8°）。 */
ErrCode motor_restore_pos_err_alarm(Robot *robot, int joint);

/* 写位置偏差预警阈值 0x0010（单位 full step = 1.8°，值域 1~65535，默认 20）。 */
ErrCode motor_set_pos_err_prewarn(Robot *robot, int joint, uint16_t steps);

/* 软件限位失效/有效（0x006D）。 */
ErrCode motor_set_limit(Robot *robot, int joint, int enable);

/* 点动（0x00CA，bit15 方向 / bit14~6 速度 / bit5 停止方式 / bit0 启停）。 */
ErrCode motor_run(Robot *robot, int joint, int dir);

/* 读实时电流 0x001A（mA）；失败返回 -1。 */
int motor_read_current(Robot *robot, int joint);

/* 读实时位置 0x0004（脉冲）；ok=0 表示读失败。 */
int32_t motor_read_position(Robot *robot, int joint, int *ok);

/* 读状态字 0x0006（UINT32，位定义见 LEESN_STAT_*）。 */
ErrCode motor_read_status(Robot *robot, int joint, uint32_t *status);

/* 一次读回「位置 + 状态字」（0x0004 起连续 4 个寄存器）。 */
ErrCode motor_read_pos_status(Robot *robot, int joint, int32_t *pos, uint32_t *status);


/* 运行到绝对位置 0x00E8（等响应）。 */
ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps);
/* 同 motor_move_abs 但不等响应。 */
ErrCode motor_move_abs_noread(Robot *robot, int joint, int32_t steps);

/* 校验绝对位移步数是否超 ROBOT_ABS_MOVE_STEPS_LIMIT。 */
int motor_move_steps_ok(int32_t steps);
/* 通用「写 32 位但不等响应」。 */
ErrCode motor_write_i32_noread(Robot *robot, int joint, uint16_t reg, int32_t val);

/* 广播写 32 位（从站地址 0）。 */
ErrCode motor_write_i32_broadcast(Robot *robot, uint16_t reg, int32_t val);

/* 广播写 16 位（从站地址 0）。 */
ErrCode motor_write_u16_broadcast(Robot *robot, uint16_t reg, uint16_t val);


/* 读实时速度 0x00D6（0.01rpm），转整数 rpm 返回；失败返回 -1。 */
int motor_read_speed(Robot *robot, int joint);

/* 读实时速度 0x00D6 原始值（0.01rpm）；失败返回 0。 */
int32_t motor_read_speed_raw(Robot *robot, int joint);

/* 读细分（每转脉冲数，0x0024）；失败返回 -1。 */
int32_t motor_read_subdivision(Robot *robot, int joint);

/* 写细分 0x0024。 */
ErrCode motor_write_subdivision(Robot *robot, int joint, int32_t per_rev);

/* 读实际位置偏差 0x0011（脉冲）。 */
int motor_read_pos_err(Robot *robot, int joint);

/* 读编码器线数 0x000F（CPR）；失败返回 -1。 */
int motor_read_enc_lines(Robot *robot, int joint);

/* 读报警码 0x00A3 低 4 位（0=正常）；读不到返回 -1。 */
int motor_read_alarm(Robot *robot, int joint);

/* 清报警（0x00A4=0）。 */
ErrCode motor_clear_alarm(Robot *robot, int joint);

/* 读驱动器基地址 0x0066；失败返回 -1。 */
int motor_read_device_addr(Robot *robot, int joint);

/* 力矩模式设定 0x009E（BIT15~8 模式，BIT7~0 力矩等级 0~255）。 */
ErrCode motor_set_torque_mode(Robot *robot, int joint, int mode, int level);

/* 力矩模式执行 0x00CB（BIT15 方向 / BIT14~1 偏移脉冲数 / BIT0 启停）。 */
ErrCode motor_torque_run(Robot *robot, int joint, int dir, int offset, int run);

#endif
