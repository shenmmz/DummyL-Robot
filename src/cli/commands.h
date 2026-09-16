#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* 处理一条已解析命令；返回 1 表示请求退出（exit/quit），否则 0。
 * 支持命令（命名对齐 ABB RAPID）：
 *   home、MoveJ、MoveL、disable、enable、motor、getpos、zero、
 *   zero_save、help、exit
 * enable 不带关节号时使能全部关节，带关节号时仅使能该轴。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

/* 查询电机监控线程是否在运行 */
int cmd_motor_running(void);

/* 显示零点标定数据（当前零点和机械角） */
void cmd_zero(Robot *robot);

/* 直接写入已知电机角零点（vals 为 6 个 q0..q5），内存 + ini 持久化 */
void cmd_zero_save(Robot *robot, const double vals[6]);

/* 读取当前关节角(机械角,度)并通过正运动学计算笛卡尔坐标(X,Y,Z)与姿态(RPY) */
void cmd_getpos(Robot *robot);

/* 笛卡尔【直线】运动（对齐 ABB MoveL）：X,Y,Z in mm; Rx,Ry,Rz in deg。
 * 按步长离散直线段并逐点逆解（分支连续选解），末端沿直线运动；
 * 支持逐段到位(默认)与周期刷新两种下发模式（cmd->stream）。 */
void cmd_movel(Robot *robot, const ParsedCmd *cmd);

#endif /* DUMMYL_CLI_COMMANDS_H */
