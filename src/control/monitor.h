#ifndef MONITOR_H
#define MONITOR_H

/*
 * 在线检测与后台状态监控（LEESN 立三体系）
 * ------------------------------------------------------------
 * 周期轮询各关节状态寄存器（0x0006~0x0007，UINT32）与电流（0x001A），
 * 状态判定使用 robot_internal.h 中 LEESN_STAT_* 位定义
 * （立三无碰撞停/光电停状态字，堵转靠电流超阈值判定）。
 *
 * 线程模型（Windows）：
 *   monitor_start() 创建独立后台线程，按 interval_ms 周期巡检全部关节，
 *   更新在线状态 / 快照，并对「状态翻转」打事件日志（掉线、恢复、报警、
 *   超差、堵转），正常运行不刷屏。
 *   巡检产生的 Modbus 帧与 CLI 主线程共用同一串口，总线互斥由 robot.c
 *   的 robot_request / robot_request_noread 临界区保证，本模块无需重复加锁。
 *   【注意】临界区只保证"帧不撞车"，不保证"带宽够用"：并发线程仍会互相
 *   排队等锁，所以耗时敏感的总线测量（diag）必须先用 monitor_pause_active
 *   请巡检让出总线，否则测到的是含排队等待的数字，且巡检会超时刷假掉线。
 *
 * 注意：线程运行期间不要调用 monitor_destroy，必须先 monitor_stop。
 */

#include "control/robot.h"
#include <stdint.h>

typedef struct Monitor Monitor;

/* 后台线程默认巡检周期（ms）
 * 【此处由 50 改为 300】巡检一轮 = 六轴 ×(状态 1 笔 + 电流 1 笔) = 12 笔事务；
 * 50ms 周期 ⇒ 需求 240 事务/秒，而单总线 115200 实测只有 ~65 事务/秒，
 * 会把总线永久打满（控制周期掉到 6Hz、巡检自身超时刷假掉线）。
 * 300ms ⇒ 40 事务/秒，约占 60% 余量。只是状态显示与事件日志用，300ms 完全够。
 * 【接线现状（2026-09-18 已改）】monitor_pause_active 现在接了
 * diag / bcast / **movej / movel** 四条路径。
 * 此前把"运动期间是否挂起"标注为"安全取舍，待用户决定"——**那个判断是错的**：
 * main.c 用 monitor_create(robot, 0)，堵转检测根本没开，巡检只做状态日志，
 * 运动期间挂起没有任何安全损失，不该拿"安全"当挡箭牌一直不做。
 * 实现放在【派发层 cmd_dispatch】而不是 cmd_movel 内部：内部有多个提前
 * return 分支，在那里挂起极易漏恢复、把巡检永久停住。 */
#define MONITOR_DEFAULT_INTERVAL_MS 300u

/* 创建监控器。stall_threshold_ma[6] 为【逐轴】堵转电流阈值(mA，下标0=关节1)，
 * 某轴 <=0 表示该轴不启用电流堵转事件检测。传 NULL 等价于六轴全 0（全关）。
 *
 * 【为什么改成逐轴 —— 2026-09-18】
 * 旧接口是单个 int（全局阈值），而六轴机座大小不同、额定电流能差好几倍：
 * 一个阈值必然要么大轴一动就误报、要么小轴撞死都不报。
 * 配合 ini [stall] j1..j6 逐轴配置使用。 */
Monitor *monitor_create(Robot *robot, const int stall_threshold_ma[6]);

/* 纯判定函数（不访问总线、不需要 Monitor 实例，便于离线单测）：
 * 电流 cur_ma 是否算"超阈堵转"。阈值<=0（未配置）或读数无效(cur<0)一律返回 0，
 * 保证"没配阈值"和"读不到"都不会误报 —— 漏报可以接受，误报会打断正常运动。 */
int monitor_stall_hit(int cur_ma, int threshold_ma);

/* 运行时读写单轴阈值（mA）。joint 1..6；set 传 <=0 表示关闭该轴检测。
 * 供 CLI 的 stall:N:MA 命令使用，免得每调一次阈值都要重启程序。 */
void monitor_set_stall_threshold(Monitor *m, int joint, int ma);
int  monitor_get_stall_threshold(const Monitor *m, int joint);

/* 释放监控对象。若后台线程仍在运行会先内部 stop（最多等待线程退出 2s） */
void monitor_destroy(Monitor *m);

/* 启动后台巡检线程（周期 interval_ms；<=0 用 MONITOR_DEFAULT_INTERVAL_MS）。
 * 返回 1 成功 / 0 失败（已运行、线程创建失败等）。 */
int monitor_start(Monitor *m, int interval_ms);

/* 停止后台巡检线程并回收句柄；线程不在运行则直接返回 */
void monitor_stop(Monitor *m);

/* 后台线程是否在运行 */
int monitor_is_running(const Monitor *m);

/* 运动期间挂起/恢复后台巡检（on=1 挂起）。
 * 巡检一轮 12 笔事务、周期 50ms ⇒ 需求 240 事务/秒，远超单总线 115200 的
 * 实测能力 65 事务/秒；不挂起会把控制周期拖到 6Hz 并刷假掉线报警。
 * monitor_pause_active 供 cli 层在无 Monitor* 句柄处调用。 */
void monitor_pause(Monitor *m, int on);
void monitor_pause_active(int on);

/* 巡检一次全部关节（线程内部与手动触发共用）：更新在线状态/快照/事件日志，
 * 返回在线数量 */
int monitor_poll(Monitor *m);

/* 检查指定关节是否堵转（电流超阈值，立即读总线）；堵转返回 1，正常/失败返回 0 */
int monitor_check_stall(Monitor *m, int joint);

/* 返回最近一次巡检中在线关节数 */
int monitor_online_count(const Monitor *m);

/* 单关节快照（后台线程最新缓存，不访问总线）。 */
typedef struct MonitorSnapshot {
    int      online;      /* 1=在线 0=离线 -1=未巡检/屏蔽 */
    uint32_t status;      /* 最新 32 位状态字（0x0006~0x0007） */
    int      current_ma;  /* 最新电流 mA；<0 表示未读到 */
    int      alarm_code;  /* 最新报警代码；<0 表示未知 */
} MonitorSnapshot;

/* 读取单关节快照。成功返回 ERR_NONE；参数非法返回 ERR_ARG */
ErrCode monitor_snapshot(Monitor *m, int joint, MonitorSnapshot *out);

#endif /* MONITOR_H */
