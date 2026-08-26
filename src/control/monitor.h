#ifndef MONITOR_H
#define MONITOR_H

/*
 * 在线检测与堵转报警（LEESN 立三体系）
 * ------------------------------------------------------------
 * 周期轮询各关节状态寄存器（0x0006~0x0007，UINT32）与电流（0x001A），
 * 对照堵转电流阈值（TODO-待用户确认）触发报警回调。
 * 状态判定使用 robot_internal.h 中 LEESN_STAT_* 位定义
 * （立三无碰撞停/光电停状态字，堵转靠电流超阈值判定）。
 */

#include "control/robot.h"

typedef struct Monitor Monitor;

/* 创建监控器。stall_threshold_ma 传 0 表示使用全局配置（TODO 默认禁用）。 */
Monitor *monitor_create(Robot *robot, int stall_threshold_ma);

void monitor_destroy(Monitor *m);

/* 轮询一次全部关节：更新在线状态，返回在线数量 */
int monitor_poll(Monitor *m);

/* 检查指定关节是否堵转（电流超阈值）；堵转返回 1，正常/失败返回 0 */
int monitor_check_stall(Monitor *m, int joint);

/* 返回最近一次轮询中在线关节数 */
int monitor_online_count(const Monitor *m);

#endif /* MONITOR_H */
