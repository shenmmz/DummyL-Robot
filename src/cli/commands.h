#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* 处理一条已解析命令；返回 1 表示请求退出（exit/quit），否则 0。
 *
 * 所有命令的实现（home / homej / movej / enable / disable / status / scan /
 * diag / torque / mask / unmask / calib / help）集中于此模块；
 * main.c 仅负责初始化与命令循环，不再内联任何命令逻辑。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

#endif /* DUMMYL_CLI_COMMANDS_H */
