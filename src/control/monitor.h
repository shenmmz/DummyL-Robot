#ifndef MONITOR_H
#define MONITOR_H

/*
 * 在线检测与堵转报警
 * ------------------------------------------------------------
 * 周期轮询各关节状态寄存器（0x0000）与电流（0x0005），
 * 对照堵转电流阈值（TODO-待用户确认）触发报警回调。
 */

#include "control/robot.h"

#define MONITOR_STATUS_IDLE       0x0000  /* 待机或到达位置 */
#define MONITOR_STATUS_RUNNING    0x0001  /* 运行中 */
#define MONITOR_STATUS_COLLISION  0x0002  /* 碰撞停 */
#define MONITOR_STATUS_PHOTO_POS  0x0003  /* 正光电停 */
#define MONITOR_STATUS_PHOTO_NEG  0x0004  /* 反光电停 */

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
