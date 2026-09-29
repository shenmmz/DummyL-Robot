#ifndef ROBOT_H
#define ROBOT_H


#include <stdint.h>
#include "utils/err.h"

typedef struct Robot Robot;

/* 建 Robot + 开串口 + 注入 CommOps + 启动后台监控（须在项目根目录跑，ini 用相对路径）。 */
Robot *robot_init(const char *port_name, uint32_t baudrate);

/* 停监控 + 关串口 + 释放。 */
void robot_close(Robot *robot);

/* 使能某轴。 */
ErrCode robot_enable(Robot *robot, int joint);
/* 释放某轴。 */
ErrCode robot_disable(Robot *robot, int joint);

/* 单轴 MoveJ 到目标机械角（内部加 q0 偏移）。 */
ErrCode robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 判角度是否在软限位内；out_min/out_max 回带该轴限位（可传 NULL）。 */
int robot_angle_in_soft_limit(int joint, double deg, double *out_min, double *out_max);

/* 读回异常检测：六轴同时变越限值 = 通信被打断；命中时回带最坏轴与超出量。 */
int robot_readback_anomaly(const double deg[6], const int ok[6],
                           double big_margin_deg,
                           int *bad_joint, double *bad_excess);

/* 某轴是否在线（读一次状态字探测）。 */
int robot_is_online(Robot *robot, int joint);

/* 读状态字 0x0006。 */
ErrCode robot_read_status(Robot *robot, int joint, uint32_t *status);

/* 读 32 位状态字（0x0006 起 2 个寄存器）。 */
ErrCode robot_read_status32(Robot *robot, int joint, uint32_t *status);

/* 读位置（脉冲）。ok 出参：0 = 读失败，返回值无意义。 */
int32_t robot_read_position_steps(Robot *robot, int joint, int *ok);

/* 读位置并转成机械角（度）。 */
double robot_read_position_deg(Robot *robot, int joint, int *ok);

/* 读电流（mA）；失败返回 -1。 */
int robot_read_current_ma(Robot *robot, int joint);

/* 读实时速度（rpm）；失败返回 -1。 */
int robot_read_speed_rpm(Robot *robot, int joint);

/* 读报警码（0=正常）；读不到返回 -1。 */
int robot_read_alarm(Robot *robot, int joint);

/* 把细分对齐到 ENCODER_STEPS_PER_REV(10000)。 */
void robot_apply_subdivision(Robot *robot);

/* 某轴细分是否已对齐。 */
int robot_subdivision_ok(const Robot *robot, int joint);

/* 屏蔽某轴（后续读写直接跳过，不占总线）。 */
ErrCode robot_mask(Robot *robot, int joint);
/* 解除屏蔽。 */
ErrCode robot_unmask(Robot *robot, int joint);

/* 某轴是否被屏蔽。 */
int robot_is_masked(const Robot *robot, int joint);

#endif
