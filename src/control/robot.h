#ifndef ROBOT_H
#define ROBOT_H

/*
 * DummyL-Robot 高层控制接口
 * ------------------------------------------------------------
 * 封装 Zeta 电机 Modbus RTU 操作：
 *   robot_init  打开串口并初始化
 *   robot_enable / robot_disable  使能/失能关节
 *   robot_movej 单关节绝对运动（角度 -> 步数换算）
 *   robot_home  回零（按 README 分组并行规划，骨架先单关节顺序执行）
 *   robot_status 查询关节状态
 * 运动学/轨迹规划由上层（main）调用 kinematics/trajectory 完成，
 * 本层只负责"关节 -> 电机指令"。
 */

#include <stdint.h>

typedef struct Robot Robot;

/* 打开串口并初始化总线；port_name 如 "COM3"，baudrate 默认 115200。
 * 成功返回对象，失败返回 NULL。 */
Robot *robot_init(const char *port_name, uint32_t baudrate);

void robot_close(Robot *robot);

/* 使能/失能关节（1..6）：写寄存器 0x0006 */
int robot_enable(Robot *robot, int joint);
int robot_disable(Robot *robot, int joint);

/* 单关节绝对运动：角度（度）-> 步数，写位置模式目标步数
 * （寄存器 0x0010/0x0011，可选速度 0x0013）。 */
int robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 回零：顺序执行各关节归零（寄存器 0x001F 写归零速度）。
 * 实机分组并行方案见 README 4.3，待硬件联调确认。 */
int robot_home(Robot *robot);

/* 查询关节在线状态（读寄存器 0x0000 状态）：返回 1 在线，0 离线/失败 */
int robot_is_online(Robot *robot, int joint);

/* 读取关节状态字（寄存器 0x0000），失败返回 -1 */
int robot_read_status(Robot *robot, int joint);

/* 读取关节当前位置（步数），失败返回 0 并置 *ok=0 */
int32_t robot_read_position_steps(Robot *robot, int joint, int *ok);

/* 读取关节当前电流（mA，寄存器 0x0005），失败返回 -1 */
int robot_read_current_ma(Robot *robot, int joint);

/* 屏蔽/恢复关节（1=屏蔽）：屏蔽后所有操作自动跳过该关节，不发指令、不轮询 */
int robot_mask(Robot *robot, int joint);
int robot_unmask(Robot *robot, int joint);

/* 关节是否被屏蔽：1 屏蔽 / 0 未屏蔽 */
int robot_is_masked(const Robot *robot, int joint);

#endif /* ROBOT_H */
