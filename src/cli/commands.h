#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* 处理一条已解析命令；返回 1 表示请求退出（exit/quit），否则 0。
 * 支持命令：home、movej、disable、motor、help、exit */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

/* 查询电机监控线程是否在运行 */
int cmd_motor_running(void);

/* 显示零点标定数据；do_save=1 时保存修正值 */
void cmd_zero(Robot *robot, int do_save);

#endif /* DUMMYL_CLI_COMMANDS_H */
