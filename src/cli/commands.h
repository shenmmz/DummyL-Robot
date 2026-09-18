#ifndef DUMMYL_CLI_COMMANDS_H
#define DUMMYL_CLI_COMMANDS_H

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/cmd_parser.h"

/* 处理一条已解析命令；返回 1 表示请求退出（exit/quit），否则 0。
 * 支持命令（命名对齐 ABB RAPID）：
 *   home、MoveJ、MoveL、disable、enable、motor、getpos、fk、
 *   zero、zero_save、help、exit
 * enable 不带关节号时使能全部关节，带关节号时仅使能该轴。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd);

/* 位姿合法性检查：扫六轴机械角，越软限位返回该关节号（1..6）并打印告警，
 * 正常返回 0。用于识别"驱动器掉电 ⇒ 0x00D2 RAM 零点丢失 ⇒ 机械角是假值"。
 * main.c 启动后调用一次；越限时 cmd_dispatch 会锁住基于位姿的运动命令。 */
int cmd_pose_check(Robot *robot);

/* 人工解除位姿闸门（poseok 命令）：再扫一次并如实打印当前是否仍越限 */
void cmd_pose_unlock(Robot *robot);

/* 查询电机监控线程是否在运行 */
int cmd_motor_running(void);

/* 显示零点标定数据（当前零点和机械角） */
void cmd_zero(Robot *robot);

/* 直接写入已知电机角零点（vals 为 6 个 q0..q5），内存 + ini 持久化 */
void cmd_zero_save(Robot *robot, const double vals[6]);

/* 读取当前关节角(机械角,度)并通过正运动学计算笛卡尔坐标(X,Y,Z)与姿态(RPY) */
void cmd_getpos(Robot *robot);

/* 离线正解预览：按当前 DH 表计算指定 6 个关节角对应的末端位姿、法兰倾角与臂形。
 * 不读硬件、不下发、不动臂；用于和 cmd_getpos 的真机读数逐项对照。 */
void cmd_fk(const ParsedCmd *cmd);
void cmd_diag(Robot *robot, const ParsedCmd *cmd);

/* 广播帧验证：向地址 0 写一次 0x00D8(运行速度)，再逐轴读回，
 * 判断该型号从站是否【执行】广播写。决定"6 路独立总线 + 广播下发"路线是否成立。 */
void cmd_bcast(Robot *robot);

/* noread 连发帧完整性测试：反复下发当前位置（原地不动），
 * 测出"连发不撞车"所需的帧间延迟与每轮真实耗时。 */
void cmd_nrtest(Robot *robot);

/* 笛卡尔【直线】运动（对齐 ABB MoveL）：X,Y,Z in mm; Rx,Ry,Rz in deg。
 * 按步长离散直线段并逐点逆解（分支连续选解），末端沿直线运动；
 * 下发模式见 MovlMode：sync 单发同步(默认) / step 逐段到位 / stream 周期刷新。 */
void cmd_movel(Robot *robot, const ParsedCmd *cmd);

#endif /* DUMMYL_CLI_COMMANDS_H */
