#ifndef MONITOR_H
#define MONITOR_H


#include "control/robot.h"
#include <stdint.h>

typedef struct Monitor Monitor;

#define MONITOR_DEFAULT_INTERVAL_MS 300u

#define MONITOR_PARK_TIMEOUT_MS 500u

/* 创建后台巡检对象（不启动线程）。stall_threshold_ma 为六轴堵转电流阈值。 */
Monitor *monitor_create(Robot *robot, const int stall_threshold_ma[6]);

/* 判电流是否达到堵转阈值。 */
int monitor_stall_hit(int cur_ma, int threshold_ma);

/* 改某轴堵转阈值（`stall` 命令）。 */
void monitor_set_stall_threshold(Monitor *m, int joint, int ma);
/* 读某轴堵转阈值。 */
int  monitor_get_stall_threshold(const Monitor *m, int joint);

/* 停线程 + 释放。 */
void monitor_destroy(Monitor *m);

/* 启动巡检线程（interval_ms<=0 用默认）；内部复位 paused/parked。 */
int monitor_start(Monitor *m, int interval_ms);

/* 停巡检线程（join）。 */
void monitor_stop(Monitor *m);

/* 挂起/恢复指定实例（只置标志，不等待）。 */
void monitor_pause(Monitor *m, int on);
/* 对全局当前实例挂起/恢复。 */
void monitor_pause_active(int on);

/* 挂起 + 等巡检真正停住再返回（超时上限 MONITOR_PARK_TIMEOUT_MS）。 */
void monitor_park(void);
/* 同 monitor_park 但自定义超时；返回 1=已停住 / 0=超时。 */
int  monitor_park_wait(int timeout_ms);

/* 已完成巡检轮数（可判断监控是否停摆）。 */
long monitor_poll_count(void);

/* 跑一轮巡检（六轴各读位置+电流，有报警再查），返回本轮在线轴数。 */
int monitor_poll(Monitor *m);

typedef struct MonitorSnapshot {
    int      online;
    uint32_t status;
    int      current_ma;
    int      alarm_code;
} MonitorSnapshot;

/* 取某轴的快照（在线/状态字/电流/报警码）。读快照不占总线。 */
ErrCode monitor_snapshot(Monitor *m, int joint, MonitorSnapshot *out);

#endif
