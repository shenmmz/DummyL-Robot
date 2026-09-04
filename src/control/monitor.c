/*
 * monitor.c —— 关节在线检测与堵转报警监控
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：monitor_create、monitor_destroy、monitor_poll、
 *           monitor_check_stall、monitor_online_count
 * 依赖模块：control/robot、config/robot_config、utils/logger
 */

#include "control/monitor.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#endif

/* 快照单轴缓存 */
typedef struct {
    int      online;
    int      masked;
    uint32_t status;
    int      current_ma;
    int      alarm_code;
} SnapJoint;

struct Monitor {
    Robot *robot;
    int stall_threshold_ma; /* 堵转电流阈值；<=0 表示不检测 */
    int interval_ms;        /* 巡检周期 ms */
    volatile LONG running;  /* 线程运行标志（Interlocked 访问） */
    HANDLE thread;          /* 后台巡检线程句柄，NULL=未启动 */
    CRITICAL_SECTION snap_lock; /* 保护 online/snap 快照与事件缓存 */
    int online[ROBOT_JOINT_COUNT];
    int online_count;
    int prev_online[ROBOT_JOINT_COUNT];  /* -1=首轮未定 */
    int prev_alarm[ROBOT_JOINT_COUNT];   /* -1=首轮未定 */
    int prev_overrun[ROBOT_JOINT_COUNT]; /* -1=首轮未定 */
    int prev_neg[ROBOT_JOINT_COUNT];     /* -1=首轮未定 */
    int prev_pos[ROBOT_JOINT_COUNT];     /* -1=首轮未定 */
    int prev_stall[ROBOT_JOINT_COUNT];   /* 0=正常 1=堵转闩锁 */
    SnapJoint snap[ROBOT_JOINT_COUNT];
};

/* monitor_create：创建监控对象，stall_threshold_ma<=0 表示不检测堵转 */
Monitor *monitor_create(Robot *robot, int stall_threshold_ma)
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
    m->stall_threshold_ma = stall_threshold_ma;
    m->interval_ms = (int)MONITOR_DEFAULT_INTERVAL_MS;
    m->thread = NULL;
    m->running = 0;
    InitializeCriticalSection(&m->snap_lock);
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        m->online[i] = 0;
        m->prev_online[i] = -1;   /* 首轮只建档不报警 */
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

/* monitor_destroy：释放监控对象（先停线程） */
void monitor_destroy(Monitor *m)
{
    if (m == NULL) {
        return;
    }
    monitor_stop(m);
    DeleteCriticalSection(&m->snap_lock);
    free(m);
}

/* monitor_scan_joint：巡检单关节（总线读在锁外，快照/事件更新在锁内）。
 * 读取 32 位状态字（0x0006~0x0007）与实时电流（0x001A），
 * 按翻转检测打事件日志：掉线/恢复、报警置位/清除（报警代码补读 0x00A3）、
 * 位置超差、软件限位、堵转电流闩锁；正常无变化时不打日志。 */
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
        cur = -1;   /* 离线关节跳过电流读，避免每轮叠加一次超时 */
    }

    EnterCriticalSection(&m->snap_lock);
    m->snap[idx].masked = 0;
    m->snap[idx].online = now_online;
    m->snap[idx].current_ma = cur;
    m->online[idx] = now_online;
    if (now_online) {
        m->snap[idx].status = st;
    }

    /* 在线状态翻转 */
    if (m->prev_online[idx] != -1 && m->prev_online[idx] != now_online) {
        if (now_online) {
            LOG_INFO("关节%d 恢复在线", j);
        } else {
            LOG_WARN("关节%d 掉线（状态读取无响应）", j);
        }
    }
    m->prev_online[idx] = now_online;

    if (now_online) {
        int alarm   = (st & LEESN_STAT_ALARM)   ? 1 : 0;
        int overrun = (st & LEESN_STAT_OVERRUN) ? 1 : 0;
        int neg     = (st & LEESN_STAT_SOFT_NEG) ? 1 : 0;
        int pos     = (st & LEESN_STAT_SOFT_POS) ? 1 : 0;
        int latch   = 0;

        /* 报警置位/清除翻转 */
        if (m->prev_alarm[idx] != -1 && m->prev_alarm[idx] != alarm) {
            if (alarm) {
                LOG_ERROR("关节%d 驱动器报警置位（状态字 bit21）", j);
                need_code = 1;              /* 锁外补读报警代码 */
            } else {
                LOG_INFO("关节%d 驱动器报警已清除", j);
                m->snap[idx].alarm_code = 0;
            }
        } else if (m->prev_alarm[idx] == -1 && alarm) {
            need_code = 1;                  /* 首轮建档，只读代码不打印 */
        }
        m->prev_alarm[idx] = alarm;

        /* 位置超差翻转（仅报置位） */
        if (m->prev_overrun[idx] != -1 && m->prev_overrun[idx] != overrun && overrun) {
            LOG_WARN("关节%d 位置超差（状态字 bit10）", j);
        }
        m->prev_overrun[idx] = overrun;

        /* 软件限位翻转（仅报置位） */
        if (m->prev_neg[idx] != -1 && m->prev_neg[idx] != neg && neg) {
            LOG_WARN("关节%d 到达软件负限位", j);
        }
        if (m->prev_pos[idx] != -1 && m->prev_pos[idx] != pos && pos) {
            LOG_WARN("关节%d 到达软件正限位", j);
        }
        m->prev_neg[idx] = neg;
        m->prev_pos[idx] = pos;

        /* 堵转电流闩锁：阈值>0 才启用；电流越阈值报一次，回落不刷屏 */
        if (m->stall_threshold_ma > 0 && cur >= 0) {
            latch = (cur > m->stall_threshold_ma) ? 1 : 0;
            if (!m->prev_stall[idx] && latch) {
                LOG_ERROR("关节%d 堵转报警：电流 %d mA 超阈值 %d mA",
                          j, cur, m->stall_threshold_ma);
            }
            m->prev_stall[idx] = latch;
        }
    }
    LeaveCriticalSection(&m->snap_lock);

    if (need_code) {
        int code = motor_read_alarm(m->robot, j);
        EnterCriticalSection(&m->snap_lock);
        m->snap[idx].alarm_code = code;
        if (code > 0) {
            LOG_ERROR("关节%d 驱动器报警：%s（代码 %d）",
                      j, leesn_alarm_text(code), code);
        } else if (code < 0) {
            LOG_WARN("关节%d 驱动器报警，报警代码读取失败", j);
        }
        /* code == 0：置位翻转后代码已被清除（如主流程已清警），非失败，静默 */
        LeaveCriticalSection(&m->snap_lock);
    }
}

/* monitor_poll：巡检全部关节（线程循环与手动触发共用），返回在线数量。
 * 注意：勿在多线程中同时手动调用本函数（与后台线程循环互斥由调用方保证）。 */
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

/* monitor_check_alarm：读 0x00A3 报警状态，有报警时打印代码并返回报警代码（0=正常） */
int monitor_check_alarm(Monitor *m, int joint)
{
    int code;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    code = motor_read_alarm(m->robot, joint);
    if (code < 0) {
        return 0;
    }
    if (code != 0) {
        LOG_ERROR("关节%d 驱动器报警：%s（代码 %d）", joint, leesn_alarm_text(code), code);
    }
    return code;
}

/* monitor_clear_alarm：写 0x00A4 = 0 清除报警，返回 ERR_NONE 成功 */
ErrCode monitor_clear_alarm(Monitor *m, int joint)
{
    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    return motor_clear_alarm(m->robot, joint);
}

/* monitor_check_stall：单关节电流超阈值检测（立三无堵转寄存器，碰撞/堵转
 * 判定唯一依据为 0x001A 实时电流），超阈值报警返回 1 */
int monitor_check_stall(Monitor *m, int joint)
{
    int cur;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    if (m->stall_threshold_ma <= 0) {
        return 0; /* 阈值未配置，跳过检测 */
    }
    cur = robot_read_current_ma(m->robot, joint);
    if (cur < 0) {
        return 0;
    }
    if (cur > m->stall_threshold_ma) {
        LOG_ERROR("关节%d 堵转报警：电流 %d mA 超阈值 %d mA",
                  joint, cur, m->stall_threshold_ma);
        return 1;
    }
    return 0;
}

/* monitor_online_count：返回最近一次轮询的在线关节数 */
int monitor_online_count(const Monitor *m)
{
    return (m != NULL) ? m->online_count : 0;
}

/* ===================== 后台巡检线程 ===================== */

/* monitor_thread_main：后台巡检线程主循环，running=0 时退出 */
static DWORD WINAPI monitor_thread_main(LPVOID arg)
{
    Monitor *m = (Monitor *)arg;

    while (InterlockedCompareExchange(&m->running, 1, 1) != 0) {
        monitor_poll(m);
        Sleep((DWORD)(m->interval_ms > 0
                          ? m->interval_ms
                          : (int)MONITOR_DEFAULT_INTERVAL_MS));
    }
    InterlockedExchange(&m->running, 0);
    return 0;
}

/* monitor_start：创建后台巡检线程；interval_ms<=0 用默认周期。
 * 已在运行返回 0；线程创建失败返回 0 并复位标志。 */
int monitor_start(Monitor *m, int interval_ms)
{
    if (m == NULL || m->robot == NULL) {
        return 0;
    }
    if (m->thread != NULL) {
        return 0; /* 已在运行 */
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

/* monitor_stop：请求退出并等待线程结束（最多 2s），回收句柄 */
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

/* monitor_is_running：后台线程是否在运行 */
int monitor_is_running(const Monitor *m)
{
    return (m != NULL && m->thread != NULL) ? 1 : 0;
}

/* monitor_snapshot：读取单关节最新快照（不访问总线） */
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
