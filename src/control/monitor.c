/*
 * monitor.c —— 关节在线检测与堵转报警监控
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：monitor_create、monitor_destroy、monitor_poll、
 *           monitor_check_stall、monitor_online_count
 * 依赖模块：control/robot、config/robot_config、utils/logger
 */

#include <stdio.h>
#include "control/monitor.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "config/robot_config.h"

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
    int stall_threshold_ma[ROBOT_JOINT_COUNT]; /* 逐轴堵转电流阈值；<=0 表示不检测 */
    int interval_ms;        /* 巡检周期 ms */
    volatile LONG running;  /* 线程运行标志（Interlocked 访问） */
    volatile LONG paused;   /* >0 = 运动期间挂起巡检，一个字节都不上总线 */
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

/* 当前活跃监控器：供 cli 层在没有 Monitor* 句柄的地方挂起巡检（monitor_pause_active）。
 * 【必须声明在 monitor_create 之前】monitor_create / monitor_destroy 会注册与注销它。
 * 此前这个定义被放在文件中部（monitor_pause_active 附近），导致 monitor_create
 * 里的赋值出现"g_active_monitor 未声明"，整个文件编译失败——而 build/ 里留着一份
 * 旧版本编译出来的 .obj，把这个问题掩盖了很久。 */
static Monitor *g_active_monitor = NULL;

/* monitor_stall_hit：纯判定（不访问总线）。见 monitor.h 注释。
 * 抽成独立函数是为了能离线单测 —— 堵转判定是安全逻辑，不能只在真机上试。 */
int monitor_stall_hit(int cur_ma, int threshold_ma)
{
    if (threshold_ma <= 0) return 0;   /* 该轴未配置阈值 ⇒ 不检测 */
    if (cur_ma < 0)        return 0;   /* 读数失败 ⇒ 不误报 */
    return (cur_ma > threshold_ma) ? 1 : 0;
}

/* monitor_set_stall_threshold / monitor_get_stall_threshold：运行时改单轴阈值 */
void monitor_set_stall_threshold(Monitor *m, int joint, int ma)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return;
    EnterCriticalSection(&m->snap_lock);
    m->stall_threshold_ma[joint - 1] = ma;
    if (ma <= 0) m->prev_stall[joint - 1] = 0;   /* 关检测时清闩锁，避免恢复后漏报 */
    LeaveCriticalSection(&m->snap_lock);
}

int monitor_get_stall_threshold(const Monitor *m, int joint)
{
    if (m == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) return 0;
    return m->stall_threshold_ma[joint - 1];
}

/* monitor_create：创建监控对象，th[6] 为逐轴阈值（NULL = 全 0 = 全关） */
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
    if (g_active_monitor == m) g_active_monitor = NULL;
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

        /* 报警置位/清除翻转 */
        if (m->prev_alarm[idx] != -1 && m->prev_alarm[idx] != alarm) {
            if (alarm) {
                printf("[错误] 关节%d 驱动器报警置位（状态字 bit21）\n", j);
                need_code = 1;              /* 锁外补读报警代码 */
            } else {
                printf("关节%d 驱动器报警已清除\n", j);
                m->snap[idx].alarm_code = 0;
            }
        } else if (m->prev_alarm[idx] == -1 && alarm) {
            need_code = 1;                  /* 首轮建档，只读代码不打印 */
        }
        m->prev_alarm[idx] = alarm;

        /* 位置超差翻转（仅报置位） */
        if (m->prev_overrun[idx] != -1 && m->prev_overrun[idx] != overrun && overrun) {
            printf("[警告] 关节%d 位置超差（状态字 bit10）\n", j);
        }
        m->prev_overrun[idx] = overrun;

        /* 软件限位翻转（仅报置位） */
        if (m->prev_neg[idx] != -1 && m->prev_neg[idx] != neg && neg) {
            printf("[警告] 关节%d 到达软件负限位\n", j);
        }
        if (m->prev_pos[idx] != -1 && m->prev_pos[idx] != pos && pos) {
            printf("[警告] 关节%d 到达软件正限位\n", j);
        }
        m->prev_neg[idx] = neg;
        m->prev_pos[idx] = pos;

        /* 堵转电流闩锁：逐轴阈值，>0 的轴才启用；电流越阈值报一次，回落不刷屏。
         * 阈值<=0 或读数失败时 monitor_stall_hit 恒返回 0，故这里无需再判。 */
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

/* 注：此处原有两个零调用的函数 monitor_check_alarm / monitor_clear_alarm，
 * 2026-09-21 清理时删除（monitor.h 从未声明，全项目零引用）。
 * 报警读取已由 monitor_poll 内联、清除已由 home.c 直接调 motor_clear_alarm 承担。
 * 需要时 git show HEAD:src/control/monitor.c 可取回。 */

/* monitor_check_stall：单关节电流超阈值检测（立三无堵转寄存器，碰撞/堵转
 * 判定唯一依据为 0x001A 实时电流），超阈值报警返回 1 */
int monitor_check_stall(Monitor *m, int joint)
{
    int cur;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    if (m->stall_threshold_ma[joint - 1] <= 0) {
        return 0; /* 该轴阈值未配置，跳过检测 */
    }
    cur = robot_read_current_ma(m->robot, joint);
    if (!monitor_stall_hit(cur, m->stall_threshold_ma[joint - 1])) {
        return 0;
    }
    printf("[错误] 关节%d 堵转报警：电流 %d mA 超阈值 %d mA\n",
              joint, cur, m->stall_threshold_ma[joint - 1]);
    return 1;
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
        /* 运动期间挂起：见 monitor_pause 说明。挂起时【完全不碰总线】，
         * 否则每 50ms 的 12 笔巡检会和运动指令抢总线，把控制周期拖到 6Hz。 */
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

/* monitor_pause：运动期间挂起后台巡检。
 *
 * 【为什么必须挂起】现场实测（2026-09-18 diag）：
 *   - 巡检一轮 = 六轴 ×(状态字 1 笔 + 电流 1 笔) = 12 笔事务
 *   - 周期 50ms ⇒ 需求 240 事务/秒
 *   - 而 115200 单总线实测能力 ≈ 65 事务/秒（15.5ms/笔）
 *   ⇒ 后台巡检一个就要吃掉总线能力的 3.7 倍，把总线永久打满。
 * 后果：运动指令排在巡检后面（控制周期掉到 6Hz、段间一顿一顿），
 * 巡检自己也超时，刷出"关节N 掉线（状态读取无响应）"的假报警。
 *
 * 运动期间整条命令独占总线，结束后恢复巡检。 */
void monitor_pause(Monitor *m, int on)
{
    if (m == NULL) return;
    InterlockedExchange(&m->paused, on ? 1 : 0);
}

/* monitor_pause_active：cli 层用的便捷入口，作用于 monitor_create 注册的对象 */
void monitor_pause_active(int on)
{
    monitor_pause(g_active_monitor, on);
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
