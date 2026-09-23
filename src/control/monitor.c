
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
    HANDLE thread;
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

int monitor_stall_hit(int cur_ma, int threshold_ma)
{
    if (threshold_ma <= 0) return 0;
    if (cur_ma < 0)        return 0;
    return (cur_ma > threshold_ma) ? 1 : 0;
}

void monitor_set_stall_threshold(Monitor *m, int joint, int ma)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return;
    EnterCriticalSection(&m->snap_lock);
    m->stall_threshold_ma[joint - 1] = ma;
    if (ma <= 0) m->prev_stall[joint - 1] = 0;
    LeaveCriticalSection(&m->snap_lock);
}

int monitor_get_stall_threshold(const Monitor *m, int joint)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return 0;
    return m->stall_threshold_ma[joint - 1];
}

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

void monitor_destroy(Monitor *m)
{
    if (m == NULL) {
        return;
    }
    monitor_stop(m);
    if (g_active_monitor == m) g_active_monitor = NULL;
    DeleteCriticalSection(&m->snap_lock);
    free(m);
}

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
    return cnt;
}


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

int monitor_online_count(const Monitor *m)
{
    return (m != NULL) ? m->online_count : 0;
}


static DWORD WINAPI monitor_thread_main(LPVOID arg)
{
    Monitor *m = (Monitor *)arg;

    while (InterlockedCompareExchange(&m->running, 1, 1) != 0) {
        if (InterlockedCompareExchange(&m->paused, 0, 0) == 0) {
            monitor_poll(m);
        }
        Sleep((DWORD)(m->interval_ms > 0
                          ? m->interval_ms
                          : (int)MONITOR_DEFAULT_INTERVAL_MS));
    }
    InterlockedExchange(&m->running, 0);
    return 0;
}

void monitor_pause(Monitor *m, int on)
{
    if (m == NULL) return;
    InterlockedExchange(&m->paused, on ? 1 : 0);
}

void monitor_pause_active(int on)
{
    monitor_pause(g_active_monitor, on);
}

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
    InterlockedExchange(&m->running, 1);
    m->thread = CreateThread(NULL, 0, monitor_thread_main, m, 0, NULL);
    if (m->thread == NULL) {
        InterlockedExchange(&m->running, 0);
        return 0;
    }
    return 1;
}

void monitor_stop(Monitor *m)
{
    if (m == NULL) {
        return;
    }
    if (m->thread != NULL) {
        InterlockedExchange(&m->running, 0);
        WaitForSingleObject(m->thread, 2000);
        CloseHandle(m->thread);
        m->thread = NULL;
    }
}

int monitor_is_running(const Monitor *m)
{
    return (m != NULL && m->thread != NULL) ? 1 : 0;
}

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
