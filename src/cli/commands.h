#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* CLI 命令总分发：按 cmd->type 调对应 cmd_xxx。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

/* 位姿可信度闸门：越软限位/零点可疑时返回 0 并打印警告。 */
int cmd_pose_check(Robot *robot);

/* 解除位姿闸门（`poseok` 命令的人工解锁路径）。 */
void cmd_pose_unlock(Robot *robot);

/* 是否有运动指令正在进行中（供监控线程/急停路径查询）。 */
int cmd_motor_running(void);

/* `zero`：把当前机械位形标定为关节零点。 */
void cmd_zero(Robot *robot);

/* `zero:save`：把六轴零点写进 ini [joint_zero]。 */
void cmd_zero_save(Robot *robot, const double vals[6]);

/* `getpos`：打印六轴机械角 + 法兰位姿 + 状态/电流/报警。 */
void cmd_getpos(Robot *robot);

/* `fk`：正解——给六轴机械角算法兰位姿（不碰总线，纯算）。 */
void cmd_fk(const ParsedCmd *cmd);
/* `diag`：总线/巡检诊断，逐轮追踪 + 末段给时间账本。 */
void cmd_diag(Robot *robot, const ParsedCmd *cmd);

/* `bcast`：广播帧验证（地址 0 写速度再逐轴读回，看几轴响应）。 */
void cmd_bcast(Robot *robot);

/* `nrtest`：扫帧间延迟找安全下发间隔。 */
void cmd_nrtest(Robot *robot);

/* `pipe`：流水线批量读探针（只读、不动臂）。 */
void cmd_pipe(Robot *robot, const ParsedCmd *cmd);

/* `movel`：笛卡尔直线。默认 interp（贴直线+逐段保护）；显式加 ,smooth 走流畅（不插补）。 */
void cmd_movel(Robot *robot, const ParsedCmd *cmd);

#endif
