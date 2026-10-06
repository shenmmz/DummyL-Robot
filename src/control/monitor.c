
#include <stdio.h>
#include "control/monitor.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "config/robot_config.h"

#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#endif

/**
 * @struct SnapJoint
 * @brief  单轴快照：巡检线程写入，外部通过 monitor_snapshot 无锁读取。
 */
typedef struct {
    int      online;      /* 1=在线 0=掉线 -1=未知/已屏蔽 */
    int      masked;      /* 1=该轴被屏蔽，快照不刷新 */
    uint32_t status;      /* 状态字 0x0006（仅在线时更新） */
    int      current_ma;  /* 实时电流 mA，-1=读失败 */
    int      alarm_code;  /* 报警码 0x00A3，0=正常 -1=未读/读失败 */
} SnapJoint;

/**
 * @struct Monitor
 * @brief  后台巡检实例：封装监控线程、事件、快照与上一轮 latch 状态。对外为 opaque 指针。*/
struct Monitor {
    Robot *robot;
    int stall_threshold_ma[ROBOT_JOINT_COUNT];
    int interval_ms;
    volatile LONG running;
    volatile LONG paused;
    volatile LONG parked;
    volatile LONG poll_count;
    HANDLE thread;
    HANDLE wake_ev;
    HANDLE parked_ev;
    CRITICAL_SECTION snap_lock;
    int online[ROBOT_JOINT_COUNT];
    int online_count;
    int prev_online[ROBOT_JOINT_COUNT];
    int prev_alarm[ROBOT_JOINT_COUNT];
    int prev_overrun[ROBOT_JOINT_COUNT];
    int prev_neg[ROBOT_JOINT_COUNT];
    int prev_pos[ROBOT_JOINT_COUNT];
    int prev_stall[ROBOT_JOINT_COUNT];
    SnapJoint snap[ROBOT_JOINT_COUNT];
};

static Monitor *g_active_monitor = NULL;

/**
 * @brief 堵转判据：电流 > 阈值。
 * @param cur_ma       实测电流 mA（<0 = 读失败）
 * @param threshold_ma 阈值 mA（<=0 = 未配置）
 * @return 1=命中堵转 / 0=未命中或无数据
 */
int monitor_stall_hit(int cur_ma, int threshold_ma)
{
    if (threshold_ma <= 0) return 0;
    if (cur_ma < 0)        return 0;
    return (cur_ma > threshold_ma) ? 1 : 0;
}

/**
 * @brief 设置某轴堵转阈值（mA），传 <=0 则禁用该轴判断。
 * @param m     Monitor 实例（NULL 安全）
 * @param joint 关节号 1~6
 * @param ma    新阈值 mA
 */
void monitor_set_stall_threshold(Monitor *m, int joint, int ma)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return;
    EnterCriticalSection(&m->snap_lock);
    m->stall_threshold_ma[joint - 1] = ma;
    if (ma <= 0) m->prev_stall[joint - 1] = 0;
    LeaveCriticalSection(&m->snap_lock);
}

/**
 * @brief 取某轴当前堵转阈值。
 * @return 阈值 mA；实例 NULL 或关节号非法时返回 0
 */
int monitor_get_stall_threshold(const Monitor *m, int joint)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return 0;
    return m->stall_threshold_ma[joint - 1];
}

/**
 * @brief 创建监控对象（建快照锁 + 两个自动复位事件，不启动线程）。
 * @param robot              已初始化的 Robot 实例
 * @param stall_threshold_ma 六轴堵转电流阈值数组（NULL ⇒ 全 0，不判堵转）
 * @return Monitor 指针；失败时 NULL
 */
Monitor *monitor_create(Robot *robot, const int stall_threshold_ma[6])
{
    Monitor *m;
    int i;
    if (robot == NULL) {
        return NULL;
    }
    m = (Monitor *)calloc(1, sizeof(Monitor));
    if (m == NULL) {
        return NULL;
    }
    m->robot = robot;
    if (stall_threshold_ma != NULL) {
        for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
            m->stall_threshold_ma[i] = stall_threshold_ma[i];
        }
    }
    m->interval_ms = (int)MONITOR_DEFAULT_INTERVAL_MS;
    m->thread = NULL;
    m->running = 0;
    m->paused = 0;
    m->parked = 0;
    m->wake_ev = CreateEvent(NULL, FALSE, FALSE, NULL);
    m->parked_ev = CreateEvent(NULL, FALSE, FALSE, NULL);
    g_active_monitor = m;
    InitializeCriticalSection(&m->snap_lock);
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        m->online[i] = 0;
        m->prev_online[i] = -1;
        m->prev_alarm[i] = -1;
        m->prev_overrun[i] = -1;
        m->prev_neg[i] = -1;
        m->prev_pos[i] = -1;
        m->prev_stall[i] = 0;
        m->snap[i].online = -1;
        m->snap[i].masked = robot_is_masked(robot, i + 1) ? 1 : 0;
        m->snap[i].status = 0;
        m->snap[i].current_ma = -1;
        m->snap[i].alarm_code = -1;
    }
    return m;
}

/**
 * @brief 停监控线程 + 关闭事件 + 销毁临界区 + 释放。
 * @param m Monitor 实例（NULL 安全）
 */
void monitor_destroy(Monitor *m)
{
    if (m == NULL) {
        return;
    }
    monitor_stop(m);
    if (g_active_monitor == m) g_active_monitor = NULL;
    if (m->wake_ev != NULL) CloseHandle(m->wake_ev);
    if (m->parked_ev != NULL) CloseHandle(m->parked_ev);
    DeleteCriticalSection(&m->snap_lock);
    free(m);
}

/**
 * @brief 巡检单轴，仅在状态发生跳变时报一次（掉线/恢复、报警、超差、软限位、堵转）。
 * @param m Monitor 实例
 * @param j 关节号 1~6
 */
static void monitor_scan_joint(Monitor *m, int j)
{
    ErrCode rc;
    uint32_t st = 0;
    int cur = -1;
    int idx = j - 1;
    int now_online;
    int need_code = 0;

    if (robot_is_masked(m->robot, j)) {
        EnterCriticalSection(&m->snap_lock);
        m->snap[idx].masked = 1;
        m->snap[idx].online = -1;
        LeaveCriticalSection(&m->snap_lock);
        return;
    }

    rc = robot_read_status(m->robot, j, &st);
    now_online = (rc == ERR_NONE) ? 1 : 0;
    if (now_online) {
        cur = robot_read_current_ma(m->robot, j);
    } else {
        cur = -1;
    }

    EnterCriticalSection(&m->snap_lock);
    m->snap[idx].masked = 0;
    m->snap[idx].online = now_online;
    m->snap[idx].current_ma = cur;
    m->online[idx] = now_online;
    if (now_online) {
        m->snap[idx].status = st;
    }

    if (m->prev_online[idx] != -1 && m->prev_online[idx] != now_online) {
        if (now_online) {
            printf("关节%d 恢复在线\n", j);
        } else {
            printf("[警告] 关节%d 掉线（状态读取无响应）\n", j);
        }
    }
    m->prev_online[idx] = now_online;

    if (now_online) {
        int alarm   = (st & LEESN_STAT_ALARM)   ? 1 : 0;
        int overrun = (st & LEESN_STAT_OVERRUN) ? 1 : 0;
        int neg     = (st & LEESN_STAT_SOFT_NEG) ? 1 : 0;
        int pos     = (st & LEESN_STAT_SOFT_POS) ? 1 : 0;
        int latch   = 0;

        if (m->prev_alarm[idx] != -1 && m->prev_alarm[idx] != alarm) {
            if (alarm) {
                printf("[错误] 关节%d 驱动器报警置位（状态字 bit21）\n", j);
                need_code = 1;
            } else {
                printf("关节%d 驱动器报警已清除\n", j);
                m->snap[idx].alarm_code = 0;
            }
        } else if (m->prev_alarm[idx] == -1 && alarm) {
            need_code = 1;
        }
        m->prev_alarm[idx] = alarm;

        if (m->prev_overrun[idx] != -1 && m->prev_overrun[idx] != overrun && overrun) {
            printf("[警告] 关节%d 位置超差（状态字 bit10）\n", j);
        }
        m->prev_overrun[idx] = overrun;

        if (m->prev_neg[idx] != -1 && m->prev_neg[idx] != neg && neg) {
            printf("[警告] 关节%d 到达软件负限位\n", j);
        }
        if (m->prev_pos[idx] != -1 && m->prev_pos[idx] != pos && pos) {
            printf("[警告] 关节%d 到达软件正限位\n", j);
        }
        m->prev_neg[idx] = neg;
        m->prev_pos[idx] = pos;

        latch = monitor_stall_hit(cur, m->stall_threshold_ma[idx]);
        if (!m->prev_stall[idx] && latch) {
            printf("[错误] 关节%d 堵转报警：电流 %d mA 超阈值 %d mA\n",
                      j, cur, m->stall_threshold_ma[idx]);
        }
        m->prev_stall[idx] = latch;
    }
    LeaveCriticalSection(&m->snap_lock);

    if (need_code) {
        int code = motor_read_alarm(m->robot, j);
        EnterCriticalSection(&m->snap_lock);
        m->snap[idx].alarm_code = code;
        if (code > 0) {
            printf("[错误] 关节%d 驱动器报警：%s（代码 %d）\n",
                      j, leesn_alarm_text(code), code);
        } else if (code < 0) {
            printf("[警告] 关节%d 驱动器报警，报警代码读取失败\n", j);
        }
        LeaveCriticalSection(&m->snap_lock);
    }
}

/**
 * @brief 巡检一轮（六轴），写快照 + 报跳变事件。
 * @return 本轮在线轴数（不包含被屏蔽的轴）
 */
int monitor_poll(Monitor *m)
{
    int j;
    int cnt = 0;

    if (m == NULL || m->robot == NULL) {
        return 0;
    }
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        monitor_scan_joint(m, j);
    }
    EnterCriticalSection(&m->snap_lock);
    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        if (m->online[j]) {
            cnt++;
        }
    }
    m->online_count = cnt;
    LeaveCriticalSection(&m->snap_lock);
    InterlockedIncrement(&m->poll_count);
    return cnt;
}


/**
 * @brief 监控线程主循环：周期唤醒，paused=0 时跑 monitor_poll，paused=1 时置 parked 并发事件。
 * @return 线程退出码（固定 0）
 */
static DWORD WINAPI monitor_thread_main(LPVOID arg)
{
    Monitor *m = (Monitor *)arg;
    DWORD wait_ms;

    wait_ms = (DWORD)(m->interval_ms > 0
                          ? m->interval_ms
                          : (int)MONITOR_DEFAULT_INTERVAL_MS);

    while (InterlockedCompareExchange(&m->running, 1, 1) != 0) {
        if (InterlockedCompareExchange(&m->paused, 0, 0) == 0) {
            InterlockedExchange(&m->parked, 0);
            monitor_poll(m);
        } else {
            InterlockedExchange(&m->parked, 1);
            SetEvent(m->parked_ev);
        }
        if (m->wake_ev != NULL) {
            WaitForSingleObject(m->wake_ev, wait_ms);
        } else {
            Sleep(wait_ms);
        }
    }
    InterlockedExchange(&m->running, 0);
    return 0;
}

/**
 * @brief 置/清挂起标志，并 SetEvent(wake_ev) 唤醒线程让它立刻看到变化。
 * @note  不等待实际停住，需要确认时用 monitor_park。
 */
void monitor_pause(Monitor *m, int on)
{
    if (m == NULL) return;
    InterlockedExchange(&m->paused, on ? 1 : 0);
    if (m->wake_ev != NULL) {
        SetEvent(m->wake_ev);
    }
}

/**
 * @brief 对全局当前活动 Monitor 实例置/清挂起标志（典型场景：CLI 开运动前临时让总线）。
 */
void monitor_pause_active(int on)
{
    monitor_pause(g_active_monitor, on);
}

/**
 * @brief 等后台巡检真正停住（需先已 pause）。
 * @param timeout_ms 等待上限；<=0 时使用 MONITOR_PARK_TIMEOUT_MS
 * @return 1=已停住 / 0=超时（可能卡在读总线）
 */
int monitor_park_wait(int timeout_ms)
{
    Monitor *m = g_active_monitor;
    DWORD t0, el;

    if (m == NULL || m->thread == NULL || m->parked_ev == NULL) {
        return 1;
    }
    if (timeout_ms <= 0) {
        timeout_ms = (int)MONITOR_PARK_TIMEOUT_MS;
    }

    t0 = GetTickCount();
    for (;;) {
        if (InterlockedCompareExchange(&m->parked, 0, 0) != 0) {
            return 1;
        }
        el = GetTickCount() - t0;
        if ((int)el >= timeout_ms) {
            printf("[警告] 后台巡检 %d ms 内没停住（可能卡在读总线），按旧行为继续\n",
                   timeout_ms);
            return 0;
        }
        WaitForSingleObject(m->parked_ev, (DWORD)(timeout_ms - (int)el));
    }
}

/**
 * @brief 挂起后台巡检 + 等它真正停住（一次性接口，默认超时 MONITOR_PARK_TIMEOUT_MS）。
 */
void monitor_park(void)
{
    monitor_pause_active(1);
    monitor_park_wait(0);
}

/**
 * @brief 返回已完成巡检的轮数（可用于判断监控是否停摆）。
 * @return 轮数；无活动实例时 -1
 */
long monitor_poll_count(void)
{
    Monitor *m = g_active_monitor;

    if (m == NULL) {
        return -1;
    }
    return (long)InterlockedCompareExchange(&m->poll_count, 0, 0);
}

/**
 * @brief 启动巡检线程（先复位 paused/parked/running）。
 * @param interval_ms 巡检周期；<=0 时使用 MONITOR_DEFAULT_INTERVAL_MS
 * @return 1=启动成功 / 0=实例无效或已在跑或 CreateThread 失败
 */
int monitor_start(Monitor *m, int interval_ms)
{
    if (m == NULL || m->robot == NULL) {
        return 0;
    }
    if (m->thread != NULL) {
        return 0;
    }
    m->interval_ms = (interval_ms > 0) ? interval_ms
                                       : (int)MONITOR_DEFAULT_INTERVAL_MS;
    InterlockedExchange(&m->paused, 0);
    InterlockedExchange(&m->parked, 0);
    InterlockedExchange(&m->running, 1);
    m->thread = CreateThread(NULL, 0, monitor_thread_main, m, 0, NULL);
    if (m->thread == NULL) {
        InterlockedExchange(&m->running, 0);
        return 0;
    }
    return 1;
}

/**
 * @brief 停监控线程并 join。
 * @note  先置 running=0 再 SetEvent(wake_ev) 把循环从 Sleep 里拉出来，避免干等 2 秒超时。
 */
void monitor_stop(Monitor *m)
{
    if (m == NULL) {
        return;
    }
    if (m->thread != NULL) {
        InterlockedExchange(&m->running, 0);
        if (m->wake_ev != NULL) {
            SetEvent(m->wake_ev);
        }
        WaitForSingleObject(m->thread, 2000);
        CloseHandle(m->thread);
        m->thread = NULL;
    }
}

/**
 * @brief 取某轴上一轮巡检的快照（不占用总线）。
 * @param m     Monitor 实例
 * @param joint 关节号 1~6
 * @param out   写入目标
 * @return ERR_NONE / ERR_ARG
 */
ErrCode monitor_snapshot(Monitor *m, int joint, MonitorSnapshot *out)
{
    int idx;

    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT || out == NULL) {
        return ERR_ARG;
    }
    idx = joint - 1;
    EnterCriticalSection(&m->snap_lock);
    out->online = m->snap[idx].online;
    out->status = m->snap[idx].status;
    out->current_ma = m->snap[idx].current_ma;
    out->alarm_code = m->snap[idx].alarm_code;
    LeaveCriticalSection(&m->snap_lock);
    return ERR_NONE;
}
