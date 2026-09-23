#ifndef MONITOR_H
#define MONITOR_H


#include "control/robot.h"
#include <stdint.h>

typedef struct Monitor Monitor;

#define MONITOR_DEFAULT_INTERVAL_MS 300u

#define MONITOR_PARK_TIMEOUT_MS 500u

/* 创建后台巡检对象（不启动线程）。stall_threshold_ma 为六轴堵转电流阈值。 */
Monitor *monitor_create(Robot *robot, const int stall_threshold_ma[6]);

/* 判电流是否达到堵转阈值。抽出来是为了让测试能离线跑（不碰硬件）。 */
int monitor_stall_hit(int cur_ma, int threshold_ma);

/* 改某轴堵转阈值（`stall` 命令）。 */
void monitor_set_stall_threshold(Monitor *m, int joint, int ma);
/* 读某轴堵转阈值。 */
int  monitor_get_stall_threshold(const Monitor *m, int joint);

/* 停线程 + 释放。 */
void monitor_destroy(Monitor *m);

/* 启动巡检线程。interval_ms<=0 用 MONITOR_DEFAULT_INTERVAL_MS。
 * ⚠️ 内部会复位 paused=0 / parked=0 —— 否则 home 的 stop→start 若残留
 * paused=1，监控会永久停摆。 */
int monitor_start(Monitor *m, int interval_ms);

/* 停巡检线程（join）。 */
void monitor_stop(Monitor *m);

/* 线程是否在跑。 */
int monitor_is_running(const Monitor *m);

/* 挂起/恢复指定实例（只置标志，不等待）。 */
void monitor_pause(Monitor *m, int on);
/* 对【全局当前实例】挂起/恢复。恢复一律用这个（11 处，含提前 return 的路径）。 */
void monitor_pause_active(int on);

/* 挂起 + 等巡检【真正停住】再返回 —— 旧 `pause_active(1)+Sleep(320)` 的替代品。
 * 实测：后台在睡则 0.01ms 返回；正在巡检则等它跑完这一轮（≈20.4ms，有报警查询
 * 最多约 30ms）。超时上限 MONITOR_PARK_TIMEOUT_MS(500ms) ⇒ 兜底等同旧行为。 */
void monitor_park(void);
/* 同上但自定义超时。返回 1 = 已停住，0 = 超时（调用方自行决定是否继续）。 */
int  monitor_park_wait(int timeout_ms);

/* 已完成巡检轮数。用途：隔 5 秒敲两次 diag，这个数应涨约 15
 * （周期 ≈ 20ms 巡检 + 300ms 等待）；不涨 ⇒ 监控被永久挂起。 */
long monitor_poll_count(void);

/* 跑一轮巡检：六轴各读 2 笔事务（位置+状态/电流），有报警时再查报警。
 * 实测一轮 = 6 轴 x 2 笔 x 1.70ms ≈ 20.4ms（有报警查询时最多约 30ms）。
 * 返回本轮在线轴数。 */
int monitor_poll(Monitor *m);

/* 查某轴是否堵转（含启动掩码期与报警排除）。 */
int monitor_check_stall(Monitor *m, int joint);

/* 上一轮在线的轴数。 */
int monitor_online_count(const Monitor *m);

typedef struct MonitorSnapshot {
    int      online;
    uint32_t status;
    int      current_ma;
    int      alarm_code;
} MonitorSnapshot;

/* 取某轴的快照（在线/状态字/电流/报警码）。读快照不占总线。 */
ErrCode monitor_snapshot(Monitor *m, int joint, MonitorSnapshot *out);

#endif
