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
 *   的 robot_request 临界区保证，本模块无需重复加锁。
 *
 * 注意：线程运行期间不要调用 monitor_destroy，必须先 monitor_stop。
 */

#include "control/robot.h"
#include <stdint.h>

typedef struct Monitor Monitor;

/* 后台线程默认巡检周期（ms） */
#define MONITOR_DEFAULT_INTERVAL_MS 50u

/* 创建监控器。stall_threshold_ma <= 0 表示不启用电流堵转事件检测（默认） */
Monitor *monitor_create(Robot *robot, int stall_threshold_ma);

/* 释放监控对象。若后台线程仍在运行会先内部 stop（最多等待线程退出 2s） */
void monitor_destroy(Monitor *m);

/* 启动后台巡检线程（周期 interval_ms；<=0 用 MONITOR_DEFAULT_INTERVAL_MS）。
 * 返回 1 成功 / 0 失败（已运行、线程创建失败等）。 */
int monitor_start(Monitor *m, int interval_ms);

/* 停止后台巡检线程并回收句柄；线程不在运行则直接返回 */
void monitor_stop(Monitor *m);

/* 后台线程是否在运行 */
int monitor_is_running(const Monitor *m);

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
