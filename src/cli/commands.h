#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* 处理一条已解析命令；返回 1 表示请求退出（exit/quit），否则 0。
 * 支持命令：home、movej、disable、motor、getpos、zero、help、exit */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

/* 查询电机监控线程是否在运行 */
int cmd_motor_running(void);

/* 显示零点标定数据；do_save=1 时保存修正值（须先回零到 home 姿态） */
void cmd_zero(Robot *robot, int do_save);

/* 直接写入已知电机角零点（vals 为 6 个 q0..q5），内存 + ini 持久化 */
void cmd_zero_set(Robot *robot, const double vals[6]);

/* 读取当前关节角(机械角,度)并通过正运动学计算笛卡尔坐标(X,Y,Z)与姿态(RPY) */
void cmd_getpos(Robot *robot);

#endif /* DUMMYL_CLI_COMMANDS_H */
