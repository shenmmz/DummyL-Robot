#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

int cmd_pose_check(Robot *robot);

void cmd_pose_unlock(Robot *robot);

int cmd_motor_running(void);

void cmd_zero(Robot *robot);

void cmd_zero_save(Robot *robot, const double vals[6]);

void cmd_getpos(Robot *robot);

void cmd_fk(const ParsedCmd *cmd);
void cmd_diag(Robot *robot, const ParsedCmd *cmd);

void cmd_bcast(Robot *robot);

void cmd_nrtest(Robot *robot);

void cmd_pipe(Robot *robot, const ParsedCmd *cmd);

void cmd_movel(Robot *robot, const ParsedCmd *cmd);

#endif
