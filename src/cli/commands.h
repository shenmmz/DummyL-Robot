#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* CLI 命令总分发：按 cmd->type 调对应 cmd_xxx。main.c 的 fgets 循环里唯一入口。
 * ⚠️ 只看 type 不看 cmd_parse 的返回值 ⇒ 新增命令必须保证失败时 type 不复位错。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

/* 位姿可信度闸门：越软限位/零点可疑时返回 0 并打印警告。
 * 返回 0 ⇒ MoveJ/MoveL/tabtest/curtest(带轴) 全部锁定，必须先 home。 */
int cmd_pose_check(Robot *robot);

/* 解除位姿闸门（`poseok` 命令的人工解锁路径）。 */
void cmd_pose_unlock(Robot *robot);

/* 是否有运动指令正在进行中（供监控线程/急停路径查询）。 */
int cmd_motor_running(void);

/* `zero`：把当前机械位形标定为关节零点。⚠️ 先人工对齐各关节凹槽再敲。 */
void cmd_zero(Robot *robot);

/* `zero:save`：把六轴零点写进 ini [joint_zero]。 */
void cmd_zero_save(Robot *robot, const double vals[6]);

/* `getpos`：打印六轴机械角 + 法兰位姿 + 状态/电流/报警。
 * 判据：必须 enable 之后读；失联时会打假数 0,0,90,0,0,0。 */
void cmd_getpos(Robot *robot);

/* `fk`：正解——给六轴机械角算法兰位姿（不碰总线，纯算）。 */
void cmd_fk(const ParsedCmd *cmd);
/* `diag`：总线/巡检诊断。逐轮追踪 + 末段给时间账本。
 * ★ N=10（曾为 30）：σ_单事务≈0.053ms ⇒ SEM ±1.0%，判据阈值 5%。
 * ★ 判据：`后台巡检停住` 预期 0.1~30ms（旧版是固定盲等 320ms）；
 *   `后台巡检累计 N 轮` 隔 5 秒敲两次应涨约 15（不涨 ⇒ 监控被永久挂起）。 */
void cmd_diag(Robot *robot, const ParsedCmd *cmd);

/* `bcast`：广播帧验证（地址 0 写速度再逐轴读回，看几轴响应）。 */
void cmd_bcast(Robot *robot);

/* `nrtest`：扫帧间延迟找安全间隔。实测结论：2ms 安全（0ms 撞车），
 * 且 noread 已无优势（那个 ini 键已随 sync 模式于 2026-09-23 一并移除）。 */
void cmd_nrtest(Robot *robot);

/* `pipe`：流水线批量读探针（只读、不动臂）。先连发 6 请求再收 6 响应。
 * 用途：验证「周转能否与收发重叠」⇒ 理论 2.84ms/352Hz（3.6 倍，未验证）。 */
void cmd_pipe(Robot *robot, const ParsedCmd *cmd);

/* `movel`：笛卡尔直线。默认 smooth 模式（关节空间直线、全程不查过流）。
 * ⚠️ 姿态参数必须抄【当前 getpos】原值，否则画斜线。 */
void cmd_movel(Robot *robot, const ParsedCmd *cmd);

#endif
