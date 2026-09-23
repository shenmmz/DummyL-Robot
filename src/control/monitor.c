
#include <stdio.h>
#include "control/monitor.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "config/robot_config.h"

#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#endif

typedef struct {
    int      online;
    int      masked;
    uint32_t status;
    int      current_ma;
    int      alarm_code;
} SnapJoint;

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

/* 堵转判据：电流 > 阈值。阈值 <=0（未配置）或电流 <0（读失败）一律不算堵转。 */
int monitor_stall_hit(int cur_ma, int threshold_ma)
{
    if (threshold_ma <= 0) return 0;
    if (cur_ma < 0)        return 0;
    return (cur_ma > threshold_ma) ? 1 : 0;
}

/* 设置某轴堵转阈值（mA）。传 <=0 表示禁用该轴堵转判断。 */
void monitor_set_stall_threshold(Monitor *m, int joint, int ma)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return;
    EnterCriticalSection(&m->snap_lock);
    m->stall_threshold_ma[joint - 1] = ma;
    if (ma <= 0) m->prev_stall[joint - 1] = 0;
    LeaveCriticalSection(&m->snap_lock);
}

/* 取某轴堵转阈值。 */
int monitor_get_stall_threshold(const Monitor *m, int joint)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return 0;
    return m->stall_threshold_ma[joint - 1];
}

/* 创建监控对象：建快照锁，并建两个自动复位事件 ——
 * wake_ev（打断线程等待）与 parked_ev（通知调用方"已让出总线"）。 */
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

/* 停止线程、关闭两个事件、销毁临界区。 */
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

/* 巡检单个关节（仅本文件用）：读状态 + 电流，并做【边沿检测】——
 * 掉线/恢复、报警置位/清除、位置超差、软限位、堵转，都只在跳变时报一次，
 * 否则每 300ms 刷一行会把控制台淹掉。 */
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

/* 巡检一轮（6 个关节），返回在线轴数，并把 poll_count 加一。
 * 实测一轮 = 6 轴 x 2 笔事务 = 12 笔 x 1.70ms ≈ 20.4ms（有报警查询时最多约 30ms）。 */
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


/* 立即查某轴是否堵转（不等下一个巡检周期）。 */
int monitor_check_stall(Monitor *m, int joint)
{
    int cur;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    if (m->stall_threshold_ma[joint - 1] <= 0) {
        return 0;
    }
    cur = robot_read_current_ma(m->robot, joint);
    if (!monitor_stall_hit(cur, m->stall_threshold_ma[joint - 1])) {
        return 0;
    }
    printf("[错误] 关节%d 堵转报警：电流 %d mA 超阈值 %d mA\n",
              joint, cur, m->stall_threshold_ma[joint - 1]);
    return 1;
}

/* 取最近一轮巡检的在线轴数。 */
int monitor_online_count(const Monitor *m)
{
    return (m != NULL) ? m->online_count : 0;
}


/* 监控线程主循环（仅本文件用）。
 * ★ 用 WaitForSingleObject(wake_ev, interval) 代替 Sleep(interval)：
 *   这样"挂起请求"能【立即】打断它的等待，而不是等它睡满一整个周期。
 * 挂起时置 parked=1 并 SetEvent(parked_ev)，告诉调用方"总线已经让出来了"。
 * 未挂起时先清 parked=0 再巡检。 */
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

/* 置/清挂起标志，并唤醒线程让它立刻看到这个变化。 */
void monitor_pause(Monitor *m, int on)
{
    if (m == NULL) return;
    InterlockedExchange(&m->paused, on ? 1 : 0);
    if (m->wake_ev != NULL) {
        SetEvent(m->wake_ev);
    }
}

/* 对全局活动监控对象置/清挂起标志。 */
void monitor_pause_active(int on)
{
    monitor_pause(g_active_monitor, on);
}

/* 等后台巡检【真正停住】，返回 1=已停 / 0=超时（打警告后按旧行为继续）。
 * 实现上轮询 parked 标志而不是只信事件：自动复位事件可能残留旧信号，
 * 被伪唤醒就再检查一遍。超时上限 MONITOR_PARK_TIMEOUT_MS(500ms)
 * ⇒ 兜底路径等价于旧的盲等，零回归风险。 */
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

/* = 挂起 + 等后台真停住。
 * ★ 这是原来 11 处 `monitor_pause_active(1); Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);`
 * 的替代品。旧写法必须盲等 320ms（因为挂起标志只有到线程下一轮循环开头才被看到）；
 * 现在典型只要 0.1~30ms —— 后台在睡则几乎立刻返回，正在巡检则等它跑完这一轮。 */
void monitor_park(void)
{
    monitor_pause_active(1);
    monitor_park_wait(0);
}

/* 已完成巡检轮数。用途：隔 5 秒敲两次 diag，这个数应涨约 15（周期 ≈ 20ms 巡检 + 300ms 等待）。
 * 若完全不变 ⇒ 监控被永久挂起（某处漏了恢复），必须立刻排查。 */
long monitor_poll_count(void)
{
    Monitor *m = g_active_monitor;

    if (m == NULL) {
        return -1;
    }
    return (long)InterlockedCompareExchange(&m->poll_count, 0, 0);
}

/* 起监控线程。
 * ⚠️ 这里会复位 paused / parked：home 是 stop → 回零 → start，
 * 若残留 paused=1，重启后巡检会【永远不跑】—— 这是安全监控，静默停摆最危险。 */
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

/* 停监控线程并等它退出。先 SetEvent(wake_ev) 唤醒它，不必干等 2 秒超时。 */
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

/* 监控线程是否在跑。 */
int monitor_is_running(const Monitor *m)
{
    return (m != NULL && m->thread != NULL) ? 1 : 0;
}

/* 取某轴的快照（在线 / 状态字 / 电流 / 报警码），加锁读。 */
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
