
#include "cli/commands.h"
#include "control/robot.h"
#include "control/home.h"
#include "control/monitor.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "utils/err.h"
#include "utils/ini_rw.h"
#include "utils/telemetry.h"
#include "kinematics/joint_zero.h"
#include "comm/modbus_rtu.h"
#include "comm/serial_win.h"
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"
#include "config/robot_config.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <windows.h>


#define MOVEJ_POLL_MS      10
#define MOVEJ_INPOS_TOL   100
#define MOVEJ_TIMEOUT_MS  60000
#define MOVEJ_STATUS_MS   200
#define MOVEJ_STATUS_W    78
#define MOVEJ_MIN_RPM       1.0
#define MOVL_ACC_FLOOR_MS   60
#define MOVL_SYNC_BOW_MM   2.0
#define MOVL_MIN_STEP_MM   2.0

#define MOVL_TIP_WARN_DEG  5.0
static int    g_tx_written = 0;
#define MOVL_STEP_MM        1.0
#define MOVL_STREAM_MIN_MS  20
#define MOVL_STREAM_BEAT_S  0.10
#define MOVL_SEG_RAMP_RATIO 3.0
#define MOVL_BOW_SAMPLES    24
#define MOVL_BOW_WARN_MM    1.0
#define MOVL_STREAM_PROBE_MS 150
#define MOVL_WRIST_SINGULAR_DEG 5.0
#define MOVL_JUMP_WARN_DEG     15.0
#define MOVEJ_MAX_STEP_DEG     720.0
#define MOVL_JUMP_MAX_DEG      30.0

#define MOVEJ_ANOM_MARGIN_DEG  15.0
#define MOVEJ_ANOM_MIN_POLLS   3

static double movl_max_step_deg(void);
static double movl_max_jump_deg(void);
#define MOVL_EPS_MM         0.05
#define MOVL_EPS_DEG        0.05

#define MOVEJ_NR_GAP_MS    4

static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                          const double angles[6], double speed,
                          int accel_ms, int decel_ms,
                          const double *line_a, const double *line_b);
static void movej_multi(Robot *robot, const ParsedCmd *cmd);

static void cmd_tabtest(Robot *robot, const ParsedCmd *cmd);
static void cmd_busrate(Robot *robot, const ParsedCmd *cmd);

static void bus_drain(void)
{
    const CommOps *ops = modbus_comm_get();
    if (ops != NULL && ops->flush != NULL) {
        ops->flush();
    }
}
static void cmd_accel(Robot *robot, const ParsedCmd *cmd);
static void cmd_alarm(Robot *robot, const ParsedCmd *cmd);
static void cmd_looptest(Robot *robot, const ParsedCmd *cmd);
static void cmd_drvbaud(Robot *robot, const ParsedCmd *cmd);

static double movl_point_dev_mm(const double x[3], const double a[3], const double b[3])
{
    double ab[3], ab2 = 0.0, t = 0.0, d2 = 0.0;
    int i;
    for (i = 0; i < 3; i++) { ab[i] = b[i] - a[i]; ab2 += ab[i] * ab[i]; }
    if (ab2 < 1e-9) return 0.0;
    for (i = 0; i < 3; i++) t += (x[i] - a[i]) * ab[i];
    t /= ab2;
    if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    for (i = 0; i < 3; i++) {
        double e = (x[i] - a[i]) - t * ab[i];
        d2 += e * e;
    }
    return sqrt(d2);
}


typedef struct {
    Robot *robot;
    volatile LONG running;
    HANDLE thread;
} MotorMonitor;

static MotorMonitor *g_motor_mon = NULL;

static Monitor *g_mon = NULL;

static DWORD WINAPI motor_monitor_thread(LPVOID arg)
{
    MotorMonitor *mm = (MotorMonitor *)arg;

    while (InterlockedCompareExchange(&mm->running, 1, 1) != 0) {
        char ts[32];
        {
            SYSTEMTIME lt;
            GetLocalTime(&lt);
            snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
                     lt.wYear, lt.wMonth, lt.wDay,
                     lt.wHour, lt.wMinute, lt.wSecond);
        }

        for (int j = 1; j <= 6; j++) {
            int pos, cur, spd;
            double deg;
            int ok = 0;
            pos = motor_read_position(mm->robot, j, NULL);
            cur = motor_read_current(mm->robot, j);
            spd = motor_read_speed(mm->robot, j);
            deg = robot_read_position_deg(mm->robot, j, &ok);
            printf("%s  关节%d: 脉冲=%d  机械角=%.2f°  速度=%d rpm  电流=%d mA\n",
                   ts, j, pos, deg, spd, cur);
        }
        printf("\n");
        fflush(stdout);

        for (int i = 0; i < 2; i++) {
            if (mm->running == 0) break;
            Sleep(500);
        }
    }
    return 0;
}

static MotorMonitor *motor_monitor_create(Robot *robot)
{
    MotorMonitor *mm = (MotorMonitor *)calloc(1, sizeof(MotorMonitor));
    if (mm == NULL) return NULL;
    mm->robot = robot;
    mm->running = 0;
    mm->thread = NULL;
    return mm;
}

static int motor_monitor_start(MotorMonitor *mm)
{
    if (mm == NULL || mm->thread != NULL) return 0;
    InterlockedExchange(&mm->running, 1);
    mm->thread = CreateThread(NULL, 0, motor_monitor_thread, mm, 0, NULL);
    return mm->thread != NULL;
}

static void motor_monitor_stop(MotorMonitor *mm)
{
    if (mm == NULL || mm->thread == NULL) return;
    InterlockedExchange(&mm->running, 0);
    WaitForSingleObject(mm->thread, 2000);
    CloseHandle(mm->thread);
    mm->thread = NULL;
}

static void motor_monitor_destroy(MotorMonitor *mm)
{
    if (mm == NULL) return;
    motor_monitor_stop(mm);
    free(mm);
}

static int motor_monitor_is_running(MotorMonitor *mm)
{
    return (mm != NULL && mm->thread != NULL) ? 1 : 0;
}

int cmd_motor_running(void)
{
    return motor_monitor_is_running(g_motor_mon);
}


typedef struct {
    int    n;
    int    min;
    int    max;
    double sum;
} CurStat;

static void curstat_reset(CurStat *s)
{
    s->n = 0; s->min = 0; s->max = 0; s->sum = 0.0;
}

static void curstat_add(CurStat *s, int v)
{
    if (v < 0) return;
    if (s->n == 0) {
        s->min = v; s->max = v;
    } else {
        if (v < s->min) s->min = v;
        if (v > s->max) s->max = v;
    }
    s->sum += (double)v;
    s->n++;
}

static double curstat_mean(const CurStat *s)
{
    return (s->n > 0) ? (s->sum / (double)s->n) : 0.0;
}

#define CURTEST_IDLE_ROUNDS   20
#define CURTEST_IDLE_BASE     12
#define CURTEST_START_MASK_MS 250
#define CURTEST_MIN_SAMPLES   8
#define CURTEST_MOVE_TIMEOUT  15000u

static int curtest_sample_move(Robot *robot, int j, double target_deg,
                               double rpm, CurStat *mv)
{
    uint32_t t0;
    int k = 0;

    if (robot_movej(robot, j, target_deg, rpm) != ERR_NONE) return -1;
    t0 = GetTickCount();
    while ((GetTickCount() - t0) < CURTEST_MOVE_TIMEOUT) {
        curstat_add(mv, robot_read_current_ma(robot, j));
        if ((++k % 3) == 0 && (GetTickCount() - t0) > CURTEST_START_MASK_MS) {
            uint32_t st = 0;
            if (robot_read_status(robot, j, &st) == ERR_NONE &&
                (st & LEESN_STAT_INPOS) && mv->n >= CURTEST_MIN_SAMPLES) {
                return 0;
            }
        }
    }
    return 1;
}

static void cmd_curtest(Robot *robot, const ParsedCmd *cmd)
{
    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    if (cmd->joint == 0) {
        CurStat st[ROBOT_JOINT_COUNT];
        int j, r;

        for (j = 0; j < ROBOT_JOINT_COUNT; j++) curstat_reset(&st[j]);
        printf("curtest 静止采样：六轴【使能保持电流】（%d 轮 ≈ %d ms，一动不动）\n",
               CURTEST_IDLE_ROUNDS, CURTEST_IDLE_ROUNDS * 90);
        for (r = 0; r < CURTEST_IDLE_ROUNDS; r++) {
            for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
                if (robot_is_masked(robot, j)) continue;
                curstat_add(&st[j - 1], robot_read_current_ma(robot, j));
            }
        }
        printf("\n%-8s %10s %10s %10s %8s\n", "关节", "最小mA", "均值mA", "最大mA", "样本");
        for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
            if (robot_is_masked(robot, j)) {
                printf("关节%-4d （已屏蔽，跳过）\n", j);
                continue;
            }
            if (st[j - 1].n == 0) {
                printf("关节%-4d 读取全部失败（离线？）\n", j);
                continue;
            }
            printf("关节%-4d %10d %10.0f %10d %8d\n", j, st[j - 1].min,
                   curstat_mean(&st[j - 1]), st[j - 1].max, st[j - 1].n);
        }
        {
            int any = 0;
            for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
                int code;
                if (robot_is_masked(robot, j)) continue;
                code = motor_read_alarm(robot, j);
                if (code > 0) {
                    printf("[警告] 关节%d 报警代码 %d = %s\n",
                           j, code, leesn_alarm_text(code));
                    any++;
                }
            }
            if (any == 0) printf("（六轴当前均无报警）\n");
        }
        printf("\n下一步：curtest:N 逐轴量【运动峰值】（会小幅摆动，先确认末端已抬起）\n");
        monitor_pause_active(0);
        return;
    }

    {
        int j = cmd->joint;
        int ok = 0, rc;
        int moved_ok = 0, alarm_after = -1;
        double q0, amp = cmd->angle_deg, rpm = cmd->speed_rpm, target, dir = 1.0;
        const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
        const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
        CurStat base, mv;

        if (robot_is_masked(robot, j)) {
            printf("关节%d 已被屏蔽，无法测量\n", j);
            monitor_pause_active(0);
            return;
        }
        q0 = robot_read_position_deg(robot, j, &ok);
        if (!ok) {
            printf("[错误] 关节%d 读取当前位置失败，无法做摆动测量\n", j);
            monitor_pause_active(0);
            return;
        }
        if (q0 + amp > lmax[j - 1]) dir = -1.0;
        else if (q0 - amp < lmin[j - 1]) dir = 1.0;
        target = q0 + dir * amp;
        if (target > lmax[j - 1] || target < lmin[j - 1]) {
            printf("[错误] 关节%d 当前 %.2f°，两个方向摆 %.2f° 都会出软限位"
                   "（%.1f~%.1f），请减小摆幅\n", j, q0, amp, lmin[j - 1], lmax[j - 1]);
            monitor_pause_active(0);
            return;
        }

        curstat_reset(&base);
        curstat_reset(&mv);
        printf("curtest 关节%d：%.2f° → %.2f° → 回到 %.2f°，转速 %.0f rpm\n",
               j, q0, target, q0, rpm);
        printf("（摆幅上限 10° 由解析器强制；越软限位会自动反向）\n");

        for (int i = 0; i < CURTEST_IDLE_BASE; i++)
            curstat_add(&base, robot_read_current_ma(robot, j));

        rc = curtest_sample_move(robot, j, target, rpm, &mv);
        if (rc < 0) {
            printf("[错误] 关节%d 运动指令下发失败\n", j);
            monitor_pause_active(0);
            return;
        }
        {
            int okr = 0;
            double reached = robot_read_position_deg(robot, j, &okr);
            moved_ok = (okr && fabs(reached - target) <= 0.05) ? 1 : 0;
            if (!moved_ok) {
                printf("[错误] 关节%d 没有走到目标：目标 %.2f°，实际 %.2f°"
                       "（指令未被执行）\n", j, target, reached);
            }
            alarm_after = motor_read_alarm(robot, j);
        }
        rc = curtest_sample_move(robot, j, q0, rpm, &mv);

        printf("\n关节%d 电流实测（0x001A）：\n", j);
        printf("  静止保持：最小 %d / 均值 %.0f / 最大 %d mA（%d 样本）\n",
               base.min, curstat_mean(&base), base.max, base.n);
        printf("  运动中  ：最小 %d / 均值 %.0f / 最大 %d mA（%d 样本）%s\n",
               mv.min, curstat_mean(&mv), mv.max, mv.n,
               (rc == 1) ? "  ← 有段没判到到位（超时），峰值可能被截断" : "");
        if (!moved_ok) {
            printf("  ⚠ 本次测量【作废】：轴根本没动，上面的电流是驱动器切断输出后的读数。\n");
            if (alarm_after > 0)
                printf("  ⚠ 关节%d 报警代码 %d = %s（先排除这个再看电流）\n",
                       j, alarm_after, leesn_alarm_text(alarm_after));
            else
                printf("  ⚠ 无报警代码，检查使能/屏蔽/软限位\n");
            monitor_pause_active(0);
            return;
        }
        if (alarm_after > 0) {
            printf("  [警告] 关节%d 报警代码 %d = %s\n",
                   j, alarm_after, leesn_alarm_text(alarm_after));
        }
        if (mv.n > 0) {
            printf("  建议阈值区间：运动峰值 %d × 1.5 ~ × 2.0 = %d ~ %d mA\n",
                   mv.max, (int)(mv.max * 1.5), (int)(mv.max * 2.0));
            printf("  【只是参考，最终由你定】填到 ini [stall] j%d 并重启；0 = 该轴不启用保护。\n", j);
        } else {
            printf("  [警告] 没采到任何运动样本，检查该轴是否使能/在线\n");
        }
    }
    monitor_pause_active(0);
}

static void cmd_stall(Monitor *mon, const ParsedCmd *cmd)
{
    int j;

    if (mon == NULL) {
        printf("[错误] 监控器未创建，无法查看/设置堵转阈值\n");
        return;
    }

    if (cmd->joint >= 1) {
        int jt = cmd->joint;
        if (cmd->param >= 0.0) {
            int ma = (int)(cmd->param + 0.5);
            monitor_set_stall_threshold(mon, jt, ma);
            printf("关节%d 堵转阈值 = %d mA%s\n", jt, ma,
                   (ma <= 0) ? "（该轴检测已关闭）" : "");
            printf("（只改本次运行；要持久化请写 ini [stall] j%d 后重启）\n", jt);
        }
        {
            MonitorSnapshot snap;
            int th = monitor_get_stall_threshold(mon, jt);
            printf("关节%d：阈值 %d mA（%s）", jt, th, (th > 0) ? "启用" : "关闭");
            if (monitor_snapshot(mon, jt, &snap) == ERR_NONE && snap.current_ma >= 0)
                printf("，最新电流 %d mA\n", snap.current_ma);
            else
                printf("，最新电流未知（尚未巡检/离线）\n");
        }
        return;
    }

    printf("逐轴堵转阈值（mA，0 = 该轴不检测）：\n");
    printf("%-8s %10s %12s %10s\n", "关节", "阈值mA", "最新电流mA", "状态");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        MonitorSnapshot snap;
        int th = monitor_get_stall_threshold(mon, j);
        char curbuf[24];
        if (monitor_snapshot(mon, j, &snap) == ERR_NONE && snap.current_ma >= 0)
            snprintf(curbuf, sizeof(curbuf), "%d", snap.current_ma);
        else
            snprintf(curbuf, sizeof(curbuf), "-");
        printf("关节%-4d %10d %12s %10s\n", j, th, curbuf, (th > 0) ? "启用" : "关闭");
    }
    if (monitor_get_stall_threshold(mon, 1) <= 0 &&
        monitor_get_stall_threshold(mon, 2) <= 0 &&
        monitor_get_stall_threshold(mon, 3) <= 0 &&
        monitor_get_stall_threshold(mon, 4) <= 0 &&
        monitor_get_stall_threshold(mon, 5) <= 0 &&
        monitor_get_stall_threshold(mon, 6) <= 0) {
        printf("\n[提示] 六轴阈值全为 0 ⇒ 碰撞保护【未启用】。\n"
               "       先 curtest 量出各轴电流，再 stall:N:MA 试填，稳定后写进 ini [stall]。\n");
    }
}

#define POSE_LIMIT_TOL_DEG  1.0

static int g_pose_invalid = 0;

static int pose_scan_bad(Robot *robot)
{
    const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
    int j;

    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        int ok = 0;
        double q;
        if (robot_is_masked(robot, j)) continue;
        q = robot_read_position_deg(robot, j, &ok);
        if (!ok) continue;
        if (q < lmin[j - 1] - POSE_LIMIT_TOL_DEG ||
            q > lmax[j - 1] + POSE_LIMIT_TOL_DEG) {
            return j;
        }
    }
    return 0;
}

int cmd_pose_check(Robot *robot)
{
    int bad = pose_scan_bad(robot);

    g_pose_invalid = bad;
    if (bad > 0) {
        printf("[警告] 关节%d 当前机械角越软限位 —— 零点【很可能已丢失】\n"
               "       （0x00D2 是 RAM 无记忆寄存器，驱动器掉电即清零，须重新回零）。\n"
               "       位姿读数不可信，MoveL / MoveJ / curtest 已锁定。请先执行 home。\n",
               bad);
    }
    return bad;
}

void cmd_pose_unlock(Robot *robot)
{
    int bad = pose_scan_bad(robot);

    if (bad > 0) {
        printf("[警告] 关节%d 仍然越软限位，但按你的要求强制解锁运动命令。\n"
               "       如果零点真的丢了，接下来所有位姿与运动都是错的 —— 后果自负。\n",
               bad);
    } else {
        printf("六轴机械角均在软限位内，闸门解锁。\n");
    }
    g_pose_invalid = 0;
}


int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd)
{
    int j;

    g_mon = mon;

    if (g_pose_invalid &&
        (cmd->type == CMD_MOVEJ ||
         cmd->type == CMD_MOVEL ||
         (cmd->type == CMD_CURTEST && cmd->joint != 0) ||
         cmd->type == CMD_TABTEST)) {
        printf("[错误] 位姿不可信：关节%d 机械角越软限位，零点可能已丢失 ⇒ 命令已拒绝。\n"
               "       先执行 home 回零；确认是误判可用 poseok 解锁（不推荐）。\n",
               g_pose_invalid);
        return 0;
    }

    switch (cmd->type) {
    case CMD_HOME: {
        motor_monitor_stop(g_motor_mon);
        monitor_stop(mon);
        ErrCode rc = (cmd->joint >= 1) ? robot_home_single(robot, cmd->joint)
                                       : robot_home(robot);
        if (rc != ERR_NONE) printf("[错误] 回零失败：%s\n", err_str(rc));
        if (cmd_pose_check(robot) == 0) {
            printf("回零后六轴机械角均在软限位内，位姿闸门已解除。\n");
        }
        if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
            printf("[警告] 回零后监控线程重启失败\n");
        break;
    }
    case CMD_MOVEJ: {
        monitor_pause_active(1);
        Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);
        if (cmd->num_joints > 1) {
            movej_multi(robot, cmd);
        } else {
            int can_move = 1;
            double target = cmd->angle_deg;
            if (cmd->rel) {
                int ok = 0;
                double cur = robot_read_position_deg(robot, cmd->joint, &ok);
                if (!ok) {
                    printf("[错误] 关节%d 读取当前位置失败，相对运动无法执行\n", cmd->joint);
                    can_move = 0;
                } else {
                    target = cur + cmd->angle_deg;
                }
            }
            if (can_move) {
                double lmin = 0.0, lmax = 0.0;
                int r = robot_angle_in_soft_limit(cmd->joint, target, &lmin, &lmax);
                if (r < 0) {
                    printf("[错误] 关节号 %d 非法（应为 1..6），已拒绝下发\n", cmd->joint);
                    can_move = 0;
                } else if (r == 0) {
                    printf("[错误] 关节%d 目标 %.2f° 超出软限位 [%.0f, %.0f]°，已拒绝下发。\n"
                           "       越限位的目标必然让电机一路顶到机械极限并触发堵转/过流，\n"
                           "       是一次纯粹的无效冲撞。请改到限位内的角度。\n",
                           cmd->joint, target, lmin, lmax);
                    can_move = 0;
                }
            }
            if (can_move) {
                ErrCode rc = robot_movej(robot, cmd->joint, target, cmd->speed_rpm);
                if (rc != ERR_NONE) printf("[错误] 运动指令失败：%s\n", err_str(rc));
            }
        }
        monitor_pause_active(0);
        break;
    }
    case CMD_MOVEL: {
        monitor_pause_active(1);
        Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);
        cmd_movel(robot, cmd);
        monitor_pause_active(0);
        break;
    }
    case CMD_DISABLE: {
        motor_monitor_stop(g_motor_mon);
        if (cmd->joint >= 1) {
            ErrCode rc = robot_disable(robot, cmd->joint);
            if (rc != ERR_NONE) printf("[错误] 关节%d 失能失败：%s\n", cmd->joint, err_str(rc));
            else printf("关节%d 已泄力(失能)\n", cmd->joint);
        } else {
            for (j = 1; j <= 6; j++) {
                ErrCode rc = robot_disable(robot, j);
                if (rc != ERR_NONE) printf("[错误] 关节%d 失能失败：%s\n", j, err_str(rc));
            }
        }
        break;
    }
    case CMD_ENABLE: {
        int failed = 0;
        int first = (cmd->joint >= 1) ? cmd->joint : 1;
        int last = (cmd->joint >= 1) ? cmd->joint : 6;
        for (j = first; j <= last; j++) {
            ErrCode rc = robot_enable(robot, j);
            if (rc != ERR_NONE) {
                printf("[错误] 关节%d 使能失败：%s\n", j, err_str(rc));
                failed++;
            }
        }
        if (failed == 0) {
            if (cmd->joint >= 1) printf("关节%d 已使能\n", cmd->joint);
            else printf("全部关节已恢复使能\n");
        } else {
            printf("[错误] 有 %d 个关节使能失败\n", failed);
        }
        break;
    }
    case CMD_MOTOR: {
        if (g_motor_mon == NULL) {
            g_motor_mon = motor_monitor_create(robot);
            if (g_motor_mon == NULL || !motor_monitor_start(g_motor_mon)) {
                printf("[错误] 电机监控启动失败\n");
                motor_monitor_destroy(g_motor_mon);
                g_motor_mon = NULL;
            } else {
               printf("电机监控已启动（每1秒刷新，按 q 停止）\n");
            }
        } else {
            motor_monitor_stop(g_motor_mon);
            motor_monitor_destroy(g_motor_mon);
            g_motor_mon = NULL;
            printf("电机监控已停止\n");
        }
        break;
    }
    case CMD_ZERO:
        cmd_zero(robot);
        break;
    case CMD_ZERO_SAVE:
        cmd_zero_save(robot, cmd->zero_vals);
        break;
    case CMD_GETPOS:
        cmd_getpos(robot);
        break;
    case CMD_FK:
        cmd_fk(cmd);
        break;
    case CMD_DIAG:
        cmd_diag(robot, cmd);
        break;
    case CMD_BCAST:
        cmd_bcast(robot);
        break;
    case CMD_NRTEST:
        cmd_nrtest(robot);
        break;
    case CMD_CURTEST:
        cmd_curtest(robot, cmd);
        break;
    case CMD_STALL:
        cmd_stall(mon, cmd);
        break;
    case CMD_POSEOK:
        cmd_pose_unlock(robot);
        break;
    case CMD_TABTEST:
        cmd_tabtest(robot, cmd);
        break;
    case CMD_BUSRATE:
        cmd_busrate(robot, cmd);
        break;
    case CMD_ACCEL:
        cmd_accel(robot, cmd);
        break;
    case CMD_ALARM:
        cmd_alarm(robot, cmd);
        break;
    case CMD_LOOPTEST:
        cmd_looptest(robot, cmd);
        break;
    case CMD_DRVBAUD:
        cmd_drvbaud(robot, cmd);
        break;
    case CMD_HELP:
        cmd_print_help();
        break;
    case CMD_EXIT:
        motor_monitor_stop(g_motor_mon);
        motor_monitor_destroy(g_motor_mon);
        g_motor_mon = NULL;
        return 1;
    case CMD_EMPTY:
        break;
    default:
        printf("[警告] 未知命令，输入 help 查看帮助\n");
        break;
    }
    return 0;
}

void cmd_zero(Robot *robot)
{
    const double *zero = joint_zero_get();
    const double target[6] = {0, 0, 90, 0, 0, 0};
    double reading[6];
    double corrected[6];
    int ok[6];
    int all_ok = 1;

    for (int i = 0; i < 6; i++) {
        reading[i] = robot_read_position_deg(robot, i + 1, &ok[i]);
        if (!ok[i]) all_ok = 0;
        corrected[i] = zero[i] + (reading[i] - target[i]);
    }

    printf("当前零点:     {");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", zero[i]);
    printf("}\n");
    printf("当前角度:     {");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", reading[i]);
    printf("}°\n");
    printf("目前零点:     {");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", corrected[i]);
    printf("}\n");

    if (!all_ok) printf("[警告] 部分关节读取失败，显示值仅供参考\n");
}

void cmd_zero_save(Robot *robot, const double vals[6])
{
    (void)robot;
    joint_zero_save(vals);
    if (ini_write_joint_zero(INI_PATH, vals)) {
        printf("零点标定已保存（内存 + ini：%s）：\n", INI_PATH);
    } else {
        printf("零点标定已保存(内存)，但写入 ini 失败\n");
    }
    printf("zero_save:");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", vals[i]);
    printf("\n");
}

static double g_tlm_mech[6] = {0};
static int    g_tlm_mech_ok = 0;

static int movej_issue(Robot *robot, int num_joints, const int joints[6],
                       const double angles[6], const double *ref_angles,
                       double speed, int accel_ms, int decel_ms, int set_profile,
                       int32_t tgt[7], uint8_t pend[7])
{
    static double last_spd[7] = {0};
    static int    last_spd_ok[7] = {0};
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    double dist[7] = {0};
    double max_dist = 0;
    int i, j, remain = 0;

    if (speed <= 0.0) {
        printf("[警告] 多关节运动速度须大于 0 rpm，已拒绝执行\n");
        return 0;
    }

    for (i = 0; i < num_joints; i++) {
        if (!isfinite(angles[i])) {
            printf("[错误] 关节%d 的目标角不是有限数（%s），已拒绝下发。\n"
                   "       下发 NaN/Inf 会被转成 ±2147483647 附近的步数写进驱动器，\n"
                   "       电机会朝天文数字目标猛冲并超时急停（曾实测甩出 34mm）。\n"
                   "       请检查目标位姿是否落在万向锁（Ry≈±90°）等退化姿态上。\n",
                   joints[i], isnan(angles[i]) ? "NaN" : "Inf");
            return 0;
        }
    }

    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        tgt[j] = DEG2STEPS(angles[i] + zero[j - 1], reductions[j - 1]);
        if (ref_angles != NULL) {
            int32_t ref_steps = DEG2STEPS(ref_angles[i] + zero[j - 1], reductions[j - 1]);
            dist[j] = (double)abs(tgt[j] - ref_steps);
        } else {
            int pos_ok = 0;
            int32_t pos = motor_read_position(robot, j, &pos_ok);
            dist[j] = pos_ok ? (double)abs(tgt[j] - pos) : 0.0;
        }
        if (dist[j] > max_dist) max_dist = dist[j];
    }

    for (i = 0; i < num_joints; i++) {
        double lmin = 0.0, lmax = 0.0;
        int r = robot_angle_in_soft_limit(joints[i], angles[i], &lmin, &lmax);
        if (r < 0) {
            printf("[错误] 关节号 %d 非法（应为 1..6），已拒绝下发\n", joints[i]);
            return 0;
        }
        if (r == 0) {
            printf("[错误] 关节%d 目标 %.2f° 超出软限位 [%.0f, %.0f]°，已拒绝下发。\n"
                   "       越限位的目标必然让电机一路顶到机械极限并触发堵转/过流，\n"
                   "       是一次纯粹的无效冲撞。请改到限位内的角度。\n"
                   "       （限位值在 config/robot_config.h 的 ROBOT_JOINT_LIMIT_MIN/MAX_DEG）\n",
                   joints[i], angles[i], lmin, lmax);
            return 0;
        }
    }

    for (i = 0; i < num_joints; i++) {
        double max_steps;
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        max_steps = (double)DEG2STEPS(movl_max_step_deg(), reductions[j - 1]);
        if (max_steps < 0) max_steps = -max_steps;
        if (dist[j] > max_steps) {
            printf("[错误] 关节%d 本次位移 %.1f° 超过单步上限 %.0f°，已拒绝下发。\n"
                   "       目标离当前位置这么远，通常是单位/符号/坐标系搞错了，\n"
                   "       真发出去就是电机猛冲 + 超时急停。确认无误就调大\n"
                   "       ini [safety] max_step_deg（当前 %.0f）。\n",
                   j, dist[j] / (double)DEG2STEPS(1.0, reductions[j - 1]),
                   movl_max_step_deg(), movl_max_step_deg());
            return 0;
        }
    }

    double spd[7] = {0};
    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        if (max_dist > 0) {
            spd[j] = dist[j] / max_dist * speed;
            if (spd[j] < MOVEJ_MIN_RPM) spd[j] = MOVEJ_MIN_RPM;
        } else {
            spd[j] = speed;
        }
    }

    if (set_profile) {
        for (i = 0; i < num_joints; i++) {
            j = joints[i];
            if (robot_is_masked(robot, j)) continue;
            g_tx_written++;
            if (motor_set_profile(robot, j, accel_ms, decel_ms) != ERR_NONE)
                printf("[警告] 关节%d 加减速设置失败\n", j);
        }
    }
    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        {
            double s = spd[j];
            double ref = fabs(last_spd[j]);
            if (!set_profile && last_spd_ok[j] &&
                fabs(s - last_spd[j]) <= 0.02 * ((ref > 1e-9) ? ref : 1.0))
                continue;
            g_tx_written++;
            if (motor_set_speed(robot, j, s) != ERR_NONE)
                printf("[警告] 关节%d 速度设置失败\n", j);
            else {
                last_spd[j] = s;
                last_spd_ok[j] = 1;
            }
        }
    }
    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        g_tx_written++;
        if (motor_move_abs(robot, j, tgt[j]) != ERR_NONE) {
            printf("[警告] 关节%d 多关节运动发指令失败\n", j);
            continue;
        }
        pend[j] = 1;
        remain++;
    }

    if (num_joints == 6) {
        for (i = 0; i < 6; i++) g_tlm_mech[i] = angles[i];
        g_tlm_mech_ok = 1;
        telemetry_send(angles, "cmd");
    }

    return remain;
}

static double g_dev_peak = 0.0;
static int    g_dev_seen = 0;

static void dev_reset(void) { g_dev_peak = 0.0; g_dev_seen = 0; }

static void dev_report(const char *tag)
{
    if (!g_dev_seen) return;
    printf("  %s实测最大偏差 %.2f mm（全程峰值）\n",
           (tag != NULL) ? tag : "", g_dev_peak);
}

static void sync_finish(uint32_t t0, double total_dt)
{
    double actual = (double)(GetTickCount() - t0) / 1000.0;
    dev_report("sync ");
    if (total_dt > 1e-6)
        printf("  实际耗时 %.2f s（预计 %.2f s）比值 %.2f "
               "—— 超出 1 的部分就是段间归零吃掉的\n",
               actual, total_dt, actual / total_dt);
}

static void status_line(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("\r%-*s", MOVEJ_STATUS_W, buf);
    fflush(stdout);
}

static void status_clear(void)
{
    printf("\r%*s\r", MOVEJ_STATUS_W, "");
    fflush(stdout);
}

static void movej_wait(Robot *robot, const int joints[6], const int32_t tgt[7],
                       uint8_t pend[7], int remain,
                       const double *line_a, const double *line_b)
{
    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    uint32_t start_ms = GetTickCount();
    uint32_t last_print = 0;
    int j;

    int32_t pos[7] = {0};
    int      ok[7] = {0};
    int      anom_n = 0;

    (void)joints;
    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= MOVEJ_TIMEOUT_MS) {
            const uint16_t red_to[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
            status_clear();
            for (j = 1; j <= 6; j++) {
                int pos_ok = 0;
                int32_t pos;
                ErrCode rc;
                if (!pend[j]) continue;
                pos = motor_read_position(robot, j, &pos_ok);
                rc = motor_estop(robot, j);
                printf("[警告] 关节%d 多关节运动超时，已急停%s", j,
                       (rc == ERR_NONE) ? "" : "（急停指令下发失败）");
                if (pos_ok) {
                    double per_deg = (double)DEG2STEPS(1.0, red_to[j - 1]);
                    printf("（还差 %.2f° = %ld 步）",
                           (double)(tgt[j] - pos) / per_deg, (long)(tgt[j] - pos));
                }
                printf("\n");
                pend[j] = 0; remain--;
            }
            break;
        }
        double   motor[6], mech[6];
        double   dev = 0.0;
        for (j = 1; j <= 6; j++) {
            if (!pend[j]) continue;
            pos[j] = motor_read_position(robot, j, &ok[j]);
        }
        for (j = 0; j < 6; j++)
            motor[j] = ok[j + 1] ? STEPS2DEG(pos[j + 1], red[j]) : 0.0;
        joint_zero_motor_to_mech(motor, mech);

        if (g_tlm_mech_ok) {
            for (j = 0; j < 6; j++)
                if (ok[j + 1]) g_tlm_mech[j] = mech[j];
            telemetry_send(g_tlm_mech, "act");
        }

        {
            int ok6[6], bad_j = 0;
            double bad_ex = 0.0;
            for (j = 0; j < 6; j++) ok6[j] = ok[j + 1];
            if (robot_readback_anomaly(mech, ok6, MOVEJ_ANOM_MARGIN_DEG,
                                       &bad_j, &bad_ex)) {
                if (++anom_n >= MOVEJ_ANOM_MIN_POLLS) {
                    status_clear();
                    printf("[危险] 关节%d 读回 %.2f° 越软限位 %.1f°，且连续 %d 轮如此 ——\n"
                           "       这不是真实位置，是总线/读回异常（2026-09-19 实测：六轴\n"
                           "       读回同时变成越限值，末端偏差冻结在 258.92mm 不动）。\n"
                           "       不再等 %d ms 超时，立即中止并尽力急停。\n"
                           "       【请检查】USB-RS485 适配器与驱动器供电，别急着改软件。\n",
                           bad_j, mech[bad_j - 1], bad_ex, anom_n, MOVEJ_TIMEOUT_MS);
                    for (j = 1; j <= 6; j++) {
                        ErrCode rc;
                        if (!pend[j]) continue;
                        rc = motor_estop(robot, j);
                        if (rc != ERR_NONE) {
                            printf("[警告] 关节%d 急停指令下发失败（总线确实不通了）\n", j);
                        }
                        pend[j] = 0; remain--;
                    }
                    break;
                }
            } else {
                anom_n = 0;
            }
        }

        if (line_a != NULL && line_b != NULL) {
            double m[4][4], xyz[3];
            dh_forward(DH_TABLE, mech, m);
            for (j = 0; j < 3; j++) xyz[j] = m[j][3];
            dev = movl_point_dev_mm(xyz, line_a, line_b);
            if (!g_dev_seen || dev > g_dev_peak) g_dev_peak = dev;
            g_dev_seen = 1;
        }
        for (j = 1; j <= 6; j++) {
            if (!pend[j]) continue;
            if (ok[j] && pos[j] >= tgt[j] - MOVEJ_INPOS_TOL &&
                pos[j] <= tgt[j] + MOVEJ_INPOS_TOL) {
                pend[j] = 0; remain--;
            }
        }
        if ((GetTickCount() - last_print) >= MOVEJ_STATUS_MS || remain == 0) {
            last_print = GetTickCount();
            if (line_a != NULL && line_b != NULL)
                status_line("J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f  偏差%6.2f mm",
                            mech[0], mech[1], mech[2], mech[3], mech[4], mech[5], dev);
            else
                status_line("J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f",
                            mech[0], mech[1], mech[2], mech[3], mech[4], mech[5]);
        }
        if (remain > 0) Sleep(MOVEJ_POLL_MS);
    }
    status_clear();
}

static void movl_dev_sample(Robot *robot, const double *line_a, const double *line_b)
{
    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    double motor[6], mech[6], m[4][4], xyz[3];
    int32_t pos[7];
    int ok[7], j;

    if (line_a == NULL || line_b == NULL) return;
    for (j = 1; j <= 6; j++) {
        pos[j] = motor_read_position(robot, j, &ok[j]);
        if (!ok[j]) return;
    }
    for (j = 0; j < 6; j++) motor[j] = STEPS2DEG(pos[j + 1], red[j]);
    joint_zero_motor_to_mech(motor, mech);
    dh_forward(DH_TABLE, mech, m);
    for (j = 0; j < 3; j++) xyz[j] = m[j][3];
    {
        double dev = movl_point_dev_mm(xyz, line_a, line_b);
        if (!g_dev_seen || dev > g_dev_peak) g_dev_peak = dev;
        g_dev_seen = 1;
    }
}

static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                         const double angles[6], double speed,
                         int accel_ms, int decel_ms,
                         const double *line_a, const double *line_b)
{
    int32_t tgt[7] = {0};
    uint8_t pend[7] = {0};
    int remain = movej_issue(robot, num_joints, joints, angles, NULL,
                             speed, accel_ms, decel_ms, 1, tgt, pend);
    if (remain <= 0) return;
    movej_wait(robot, joints, tgt, pend, remain, line_a, line_b);
}

static void movej_issue_noread(Robot *robot, const int joints[6], int32_t tgt[7],
                               int gap_ms)
{
    int j;
    for (j = 0; j < 6; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        if (gap_ms > 0) {
            motor_move_abs_noread(robot, joints[j], tgt[joints[j]]);
            Sleep((DWORD)gap_ms);
        } else {
            motor_move_abs(robot, joints[j], tgt[joints[j]]);
        }
    }
}

static void movl_set_seg_speed(Robot *robot, const int joints[6],
                               const double qa[6], const double qb[6],
                               const uint16_t red[6], double seg_rpm)
{
    double seg[7] = {0}, smax = 0.0, spd;
    int j;

    for (j = 0; j < 6; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        seg[joints[j]] = fabs(qb[j] - qa[j]) * (double)red[j];
        if (seg[joints[j]] > smax) smax = seg[joints[j]];
    }
    if (smax <= 0.0) return;
    for (j = 0; j < 6; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        spd = seg[joints[j]] / smax * seg_rpm;
        if (spd < MOVEJ_MIN_RPM) spd = MOVEJ_MIN_RPM;
        motor_set_speed(robot, joints[j], spd);
    }
}

static int movl_acc_floor_ms(void)
{
    double v;
    if (ini_read_positive_double(INI_PATH, "movel", "acc_floor_ms", &v))
        return (int)v;
    return MOVL_ACC_FLOOR_MS;
}

static double movl_seg_ramp_ratio(void)
{
    double v;
    if (ini_read_positive_double(INI_PATH, "movel", "seg_ramp_ratio", &v))
        return v;
    return MOVL_SEG_RAMP_RATIO;
}

static double movl_stream_beat_s(void)
{
    double v;
    if (ini_read_positive_double(INI_PATH, "movel", "stream_beat_s", &v))
        return v;
    return MOVL_STREAM_BEAT_S;
}

static void movej_multi(Robot *robot, const ParsedCmd *cmd)
{
    int joints[6], idx = 0;
    double angles[6];
    int acc, dec;
    for (int i = 0; i < cmd->num_joints; i++) {
        joints[idx] = cmd->joints[i];
        angles[idx] = cmd->angles[i];
        idx++;
    }
    {
        int floor_ms = movl_acc_floor_ms();
        acc = (cmd->accel_ms[0] >= floor_ms) ? cmd->accel_ms[0] : floor_ms;
        dec = (cmd->decel_ms[0] >= floor_ms) ? cmd->decel_ms[0] : floor_ms;
        if ((cmd->accel_ms[0] > 0 && cmd->accel_ms[0] < floor_ms) ||
            (cmd->decel_ms[0] > 0 && cmd->decel_ms[0] < floor_ms))
            printf("[提示] 加/减速 %d/%d ms 低于安全下限，已抬到 %d/%d ms\n",
                   cmd->accel_ms[0], cmd->decel_ms[0], acc, dec);
    }
    movej_joints(robot, cmd->num_joints, joints, angles,
                 cmd->speeds[0], acc, dec, NULL, NULL);
}

static int movl_noread_gap_ms(void)
{
    FILE *f;
    char line[256];
    int in_movel = 0;

    f = fopen(INI_PATH, "r");
    if (f == NULL) return MOVEJ_NR_GAP_MS;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_movel = (strncmp(p, "[movel]", 7) == 0) ? 1 : 0;
            continue;
        }
        if (!in_movel) continue;
        if (strncmp(p, "noread_gap_ms", 13) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            int v = atoi(eq + 1);
            return (v >= 0) ? v : MOVEJ_NR_GAP_MS;
        }
    }
    fclose(f);
    return MOVEJ_NR_GAP_MS;
}

static void movl_stall_thresholds(int th[6])
{
    const int def[ROBOT_JOINT_COUNT] = ROBOT_STALL_CURRENT_MA_TABLE;
    int i;

    if (g_mon != NULL) {
        for (i = 0; i < ROBOT_JOINT_COUNT; i++)
            th[i] = monitor_get_stall_threshold(g_mon, i + 1);
        return;
    }
    if (ini_read_stall_current(INI_PATH, th)) return;
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) th[i] = def[i];
}

static int movl_stall_on(const int th[6])
{
    int i;
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        if (th[i] > 0) return 1;
    }
    return 0;
}

static int movl_stall_guard(Robot *robot, const int th[6])
{
    double worst = 0.0;
    int worst_j = 0, worst_cur = 0;
    int j;

    if (!movl_stall_on(th)) return 0;

    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        int cur;
        double r;
        if (th[j] <= 0) continue;
        if (robot_is_masked(robot, j + 1)) continue;
        cur = robot_read_current_ma(robot, j + 1);
        if (cur <= 0) continue;
        r = (double)cur / (double)th[j];
        if (r > worst) { worst = r; worst_j = j + 1; worst_cur = cur; }
    }
    if (worst >= 1.0) {
        printf("[过流保护] 关节%d 实时电流 %d mA 超阈值 %d mA（%.0f%%），急停全部关节\n",
               worst_j, worst_cur, th[worst_j - 1], worst * 100.0);
        for (j = 0; j < ROBOT_JOINT_COUNT; j++)
            if (!robot_is_masked(robot, j + 1)) motor_estop(robot, j + 1);
        return 1;
    }
    return 0;
}

static int movl_plan(const double start_pose[6], const double end_pose[6],
                     const double q_start[6], const JointLimit *limits,
                     double dist_mm, double step_mm, const double vmax[6],
                     double q_seq[LINE_MAX_POINTS][6], double seg_dt[LINE_MAX_SEGS],
                     int *out_count, double *out_total_dt, int *out_fail_idx,
                     char *fail_reason)
{
    LinePath path;
    int count = line_count_for_distance(dist_mm, step_mm);
    int i;

    if (line_plan(start_pose, end_pose, count, &path) != 0) return -1;
    if (line_solve(&path, DH_TABLE, limits, q_start, q_seq, out_fail_idx, fail_reason) != 0) return -2;
    if (line_time_table(q_seq, count, vmax, seg_dt, out_total_dt) != 0) return -3;
    *out_count = count;

    int warn_j = 0, warn_seg = 0;
    double warn_deg;

    warn_deg = line_max_joint_jump(q_seq, count, &warn_j, &warn_seg);

    if (warn_deg > movl_max_jump_deg()) {
        printf("[拒绝] 规划路径第 %d/%d 段：关节%d 单段要转 %.1f°,\n"
               "       超过单段跳变上限 %.0f°（ini [safety] max_jump_deg）⇒ 整条 MoveL 不下发。\n"
               "       【实测后果】时间表按关节限速自动把这一段拉长到十几秒 ——\n"
               "       这段时间里末端只挪 1mm，而 J4 转过 90°：臂会以一种你\n"
               "       完全没预料到的大幅度慢慢扫过去，肘部/法兰扫过一大片空间。\n"
               "       不是「猛冲」，但撞到周围东西的概率极高，而且看起来根本不像直线。\n"
               "       成因通常是路径擦过腕部奇异位形（J5≈0）或逆解分支翻转。\n"
               "       【怎么绕开】先用 MoveJ:J1..J6 把姿态挪开（尤其把 J5 移出 ±5°），\n"
               "       再从新姿态走笛卡尔直线。确认要强走就把 max_jump_deg 调大,\n"
               "       但那等于自己承担甩臂风险。\n",
               warn_seg, count, warn_j, warn_deg, movl_max_jump_deg());
        return -4;
    }
    if (warn_deg > MOVL_JUMP_WARN_DEG) {
        printf("[警告] 规划路径第 %d/%d 段：关节%d 单段要转 %.1f°（告警阈值 %.0f°）。\n"
               "       末端位姿是精确的，但关节会走得很凶 —— 时间表会把这一段\n"
               "       按关节限速拉长，期间末端几乎不动而某个关节转过一大片角度，\n"
               "       臂会以一种你没预料到的姿态扫过去。留意周围有没有东西可撞。\n"
               "       成因通常是路径擦过奇异位形/逆解分支翻转,\n"
               "       建议先用 MoveJ:J1..J6 把姿态挪开再走笛卡尔直线。\n",
               warn_seg, count, warn_j, warn_deg, MOVL_JUMP_WARN_DEG);
    }

    for (i = 0; i < count; i++) {
        if (fabs(q_seq[i][4]) < MOVL_WRIST_SINGULAR_DEG) {
            printf("[警告] 规划路径第 %d/%d 点进入腕部奇异区（J5 = %.2f° ≈ 0）：\n"
                   "       此处 θ4 与 θ6 同轴、关节分配不唯一。IK 已做退化处理\n"
                   "       （保留真实 θ5、θ4 锚定上一点），末端位姿仍然精确；\n"
                   "       但离开奇异点时 θ4 会被位姿唯一锁死，可能要求 J4 大角度转动。\n"
                   "       【实测】home 位形出发沿 +Y 走 30mm：第 1 段就要 J4 转 89.98°。\n"
                   "       建议先用 MoveJ:J1..J6 把 J5 移出 ±%.0f° 再做笛卡尔直线。\n"
                   "       注：2026-09-18 那次「末端偏差 34mm、臂乱甩」的真因不是它，\n"
                   "       而是万向锁处 asin 越界产生 NaN（见 test_line_nan.c），已修。\n",
                   i, count, q_seq[i][4], MOVL_WRIST_SINGULAR_DEG);
            break;
        }
    }
    return 0;
}

static double movl_bow_mm(const double q0[6], const double q1[6],
                          const double a[3], const double b[3])
{
    double ab[3], ab2 = 0.0, worst = 0.0;
    int k, i;

    for (i = 0; i < 3; i++) { ab[i] = b[i] - a[i]; ab2 += ab[i] * ab[i]; }
    if (ab2 < 1e-9) return 0.0;

    for (k = 1; k < MOVL_BOW_SAMPLES; k++) {
        double s = (double)k / (double)MOVL_BOW_SAMPLES;
        double q[6], m[4][4], t = 0.0, d2 = 0.0;
        for (i = 0; i < 6; i++) q[i] = q0[i] + s * (q1[i] - q0[i]);
        dh_forward(DH_TABLE, q, m);
        for (i = 0; i < 3; i++) {
            double v = m[i][3] - a[i];
            t += v * ab[i];
        }
        t /= ab2;
        if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
        for (i = 0; i < 3; i++) {
            double e = (m[i][3] - a[i]) - t * ab[i];
            d2 += e * e;
        }
        if (d2 > worst) worst = d2;
    }
    return sqrt(worst);
}


static double movl_max_step_deg(void)
{
    double v;
    if (ini_read_max_step_deg(INI_PATH, &v)) return v;
    return MOVEJ_MAX_STEP_DEG;
}

static double movl_max_jump_deg(void)
{
    double v;
    if (ini_read_max_jump_deg(INI_PATH, &v)) return v;
    return MOVL_JUMP_MAX_DEG;
}

static double movl_bow_budget(void)
{
    FILE *f;
    char line[256];
    int in_movel = 0;

    f = fopen(INI_PATH, "r");
    if (f == NULL) {
        return MOVL_SYNC_BOW_MM;
    }
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_movel = (strncmp(p, "[movel]", 7) == 0) ? 1 : 0;
            continue;
        }
        if (!in_movel) continue;
        if (strncmp(p, "bow_mm", 6) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            double v = atof(eq + 1);
            return (v > 0.0) ? v : MOVL_SYNC_BOW_MM;
        }
    }
    fclose(f);
    return MOVL_SYNC_BOW_MM;
}

static double ang_delta_deg(double a, double b)
{
    double d = fmod(a - b, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

static double movl_tip_warn_deg(void)
{
    FILE *f;
    char line[256];
    int in_movel = 0;

    f = fopen(INI_PATH, "r");
    if (f == NULL) return MOVL_TIP_WARN_DEG;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_movel = (strncmp(p, "[movel]", 7) == 0) ? 1 : 0;
            continue;
        }
        if (!in_movel) continue;
        if (strncmp(p, "tip_warn_deg", 12) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            double v = atof(eq + 1);
            return (v > 0.0) ? v : MOVL_TIP_WARN_DEG;
        }
    }
    fclose(f);
    return MOVL_TIP_WARN_DEG;
}

static void movl_pose_warn(const double start_pose[6], const double end_pose[6])
{
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double pen_mm = 0.0;
    double drx, dry, drz, worst, warn;
    double m0[4][4], m1[4][4];
    double n0[3], n1[3], dot, tilt, swing;

    if (!ini_read_pen_length(INI_PATH, &pen_mm) || pen_mm <= 0.0) {
        pen_mm = 0.0;
        if (ini_read_tool_length(INI_PATH, &pen_mm) && pen_mm <= 0.0) pen_mm = 0.0;
    }

    drx = fabs(ang_delta_deg(end_pose[3], start_pose[3]));
    dry = fabs(ang_delta_deg(end_pose[4], start_pose[4]));
    drz = fabs(ang_delta_deg(end_pose[5], start_pose[5]));
    worst = drx;
    if (dry > worst) worst = dry;
    if (drz > worst) worst = drz;
    warn = movl_tip_warn_deg();

    line_pose_to_matrix(start_pose, m0);
    line_pose_to_matrix(end_pose, m1);
    n0[0] = m0[0][2]; n0[1] = m0[1][2]; n0[2] = m0[2][2];
    n1[0] = m1[0][2]; n1[1] = m1[1][2]; n1[2] = m1[2][2];
    dot = n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2];
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;
    tilt = acos(dot) * RAD2DEG;
    swing = pen_mm * 2.0 * sin(tilt / 2.0 / RAD2DEG);

    if (worst <= warn) return;

    printf("[警告] 目标姿态与【当前】姿态不一致：ΔRx=%.2f° ΔRy=%.2f° ΔRz=%.2f°"
           "（法兰倾角变化 %.2f°）\n", drx, dry, drz, tilt);
    printf("       MoveL 会用 SLERP 把姿态【从起点一路拧到目标】，中途姿态一直在变。\n");
    printf("       当前姿态是 Rx=%.2f Ry=%.2f Rz=%.2f —— 就抄 getpos 打印的这三个。\n",
           start_pose[3], start_pose[4], start_pose[5]);
    if (pen_mm > 0.0) {
        printf("       笔长 %.2fmm ⇒ 笔尖绕法兰摆 %.2f×2sin(%.2f°/2) = %.2f mm（空间摆幅）。\n",
               pen_mm, pen_mm, tilt, swing);
        printf("       注意：摆幅大多花在【把笔从朝下抬起来】上，落到纸面的横向偏离"
               "通常远小于它\n       （88.9° 那次空间摆 57.6mm，纸上只有 7.71mm）。\n");
    } else {
        printf("       笔长未配置 ⇒ 算不出毫米数。ini [tool] pen_length 填上法兰面到笔尖的"
               "mm 就能换算。\n");
    }
    printf("       ⇒ 法兰仍走直线，但【笔尖画的是弧/斜线，落点也会偏】。\n");
    printf("       要笔尖走直线：把 Rx,Ry,Rz 抄成 getpos 打印的原值（含负号、含 -0.00）。\n");
    printf("       （告警阈值 %.1f°，ini [movel] tip_warn_deg 可调）\n", warn);
}

void cmd_movel(Robot *robot, const ParsedCmd *cmd)
{
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double q_start[6];
    double start_pose[6];
    double end_pose[6];
    double q_seq[LINE_MAX_POINTS][6];
    double seg_dt[LINE_MAX_SEGS];
    double vmax[6];
    JointLimit limits[ROBOT_JOINT_COUNT];
    int joints[6] = {1, 2, 3, 4, 5, 6};
    int count, fail_idx = -1, i, j;
    char fail_reason[64] = {0};
    double total_dt = 0.0;
    double base_rpm = (cmd->speeds[0] > 0) ? cmd->speeds[0] : 60.0;
    double speed_rpm = base_rpm;
    int acc, dec;
    {
        int floor_ms = movl_acc_floor_ms();
        acc = (cmd->accel_ms[0] >= floor_ms) ? cmd->accel_ms[0] : floor_ms;
        dec = (cmd->decel_ms[0] >= floor_ms) ? cmd->decel_ms[0] : floor_ms;
        if ((cmd->accel_ms[0] > 0 && cmd->accel_ms[0] < floor_ms) ||
            (cmd->decel_ms[0] > 0 && cmd->decel_ms[0] < floor_ms))
            printf("[提示] 加/减速 %d/%d ms 低于安全下限 %d，已抬到 %d/%d ms"
                   "（过短的减速会丢步，表现为走到一半卡住）\n",
                   cmd->accel_ms[0], cmd->decel_ms[0], floor_ms, acc, dec);
    }
    int stall_th[ROBOT_JOINT_COUNT];
    movl_stall_thresholds(stall_th);

    for (j = 0; j < 6; j++) q_start[j] = robot_read_position_deg(robot, j + 1, NULL);
    {
        double m[4][4], rpy[3];
        dh_forward(DH_TABLE, q_start, m);
        dh_pose_to_xyz_rpy(m, start_pose, rpy);
        start_pose[3] = rpy[0] * RAD2DEG;
        start_pose[4] = rpy[1] * RAD2DEG;
        start_pose[5] = rpy[2] * RAD2DEG;
    }
    for (j = 0; j < 6; j++) end_pose[j] = cmd->cartesian[j];
    if (cmd->keep_pose) {
        for (j = 0; j < 3; j++) end_pose[3 + j] = start_pose[3 + j];
        printf("MoveL: 姿态保持当前不变（Rx=%.2f Ry=%.2f Rz=%.2f，取自当前位姿）\n",
               end_pose[3], end_pose[4], end_pose[5]);
    }

    double dx = end_pose[0] - start_pose[0];
    double dy = end_pose[1] - start_pose[1];
    double dz = end_pose[2] - start_pose[2];
    double dist = sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < MOVL_EPS_MM &&
        fabs(ang_delta_deg(end_pose[3], start_pose[3])) < MOVL_EPS_DEG &&
        fabs(ang_delta_deg(end_pose[4], start_pose[4])) < MOVL_EPS_DEG &&
        fabs(ang_delta_deg(end_pose[5], start_pose[5])) < MOVL_EPS_DEG) {
        printf("MoveL: 已在目标位姿（位移 %.2f mm），无需运动\n", dist);
        return;
    }

    movl_pose_warn(start_pose, end_pose);

    {
        const double lmin[6] = ROBOT_JOINT_LIMIT_MIN_DEG;
        const double lmax[6] = ROBOT_JOINT_LIMIT_MAX_DEG;
        for (j = 0; j < 6; j++) {
            limits[j].min_deg = lmin[j];
            limits[j].max_deg = lmax[j];
        }
    }
    {
        const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
        for (j = 0; j < 6; j++) vmax[j] = speed_rpm * 6.0 / (double)red[j];
    }

    double step_mm = MOVL_STEP_MM;
    int rc = 0;

    rc = movl_plan(start_pose, end_pose, q_start, limits, dist, step_mm, vmax,
                   q_seq, seg_dt, &count, &total_dt, &fail_idx, fail_reason);
    if (rc != 0) {
        if (rc != -4) {
            printf("[错误] MoveL 第 %d 个插补点逆解失败/越软限位：%s\n",
                   fail_idx, fail_reason);
        }
        return;
    }

    if (cmd->movl_mode == MOVL_MODE_STREAM && count > 2) {
        double beat_s = movl_stream_beat_s();
        double ramp_s = movl_seg_ramp_ratio() * (double)(acc + dec) / 1000.0;
        int n_seg;
        if (ramp_s > beat_s) beat_s = ramp_s;
        n_seg = (int)ceil(total_dt / beat_s);
        if (n_seg < 1) n_seg = 1;
        if (n_seg < count - 1) {
            step_mm = dist / (double)n_seg;
            rc = movl_plan(start_pose, end_pose, q_start, limits, dist, step_mm, vmax,
                           q_seq, seg_dt, &count, &total_dt, &fail_idx, fail_reason);
            if (rc != 0) {
                if (rc != -4) {
                    printf("[错误] MoveL 第 %d 个插补点逆解失败/越软限位：%s\n",
                           fail_idx, fail_reason);
                }
                return;
            }
        }
    }

    dev_reset();

    if (cmd->movl_mode == MOVL_MODE_SYNC ||
        cmd->movl_mode == MOVL_MODE_SMOOTH) {
        const double *q_end = q_seq[count - 1];
        const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
        const double *zero = joint_zero_get();
        const double bow_budget = movl_bow_budget();
        double dmax = 0.0, sync_rpm, sync_bow = 0.0, bow_full = 0.0, dist_axis[7] = {0};
        int32_t tgt[7] = {0};
        uint8_t pend[7] = {0};
        int nr_gap = MOVEJ_NR_GAP_MS;
        uint32_t sync_t0 = GetTickCount();
        int remain = 0, stride, n_pts, last_i = 0, n_inj = 0;

        for (j = 0; j < 6; j++) {
            dist_axis[joints[j]] = fabs(q_end[j] - q_start[j]) * (double)red[j];
            if (dist_axis[joints[j]] > dmax) dmax = dist_axis[joints[j]];
        }
        sync_rpm = (total_dt > 1e-6) ? (dmax / total_dt / 6.0) : base_rpm;
        if (sync_rpm > base_rpm) sync_rpm = base_rpm;
        if (sync_rpm < 1.0) sync_rpm = 1.0;

        n_pts = count - 1;
        bow_full = movl_bow_mm(q_seq[0], q_seq[count - 1], start_pose, end_pose);
        {
            int target_n;
            if (cmd->movl_mode == MOVL_MODE_SMOOTH) {
                target_n = 1;
            } else if (bow_full <= bow_budget || dist < 1e-9) {
                target_n = 1;
            } else {
                double L = dist * sqrt(bow_budget / bow_full);
                if (L < MOVL_MIN_STEP_MM) L = MOVL_MIN_STEP_MM;
                target_n = (int)ceil(dist / L);
            }
            if (target_n < 1) target_n = 1;
            if (target_n > n_pts) target_n = n_pts;
            stride = (n_pts + target_n - 1) / target_n;
        }
        n_inj = (n_pts + stride - 1) / stride;

        for (i = 0; i < count - 1; i += stride) {
            int k = i + stride;
            double d;
            if (k > count - 1) k = count - 1;
            d = movl_bow_mm(q_seq[i], q_seq[k], start_pose, end_pose);
            if (d > sync_bow) sync_bow = d;
        }

        if (cmd->movl_mode == MOVL_MODE_SMOOTH) {
            printf("MoveL: 流畅(不分段), 位移 %.1f mm, 速度 %.1f rpm, 预计 %.2f s, "
                   "整段一次下发由驱动器自己规划 ⇒ 段间零停顿\n"
                   "        代价：走关节空间直线，会偏离笛卡尔直线 %.2f mm"
                   "（弓高 ∝ 长度²，线段越短越看不出来）\n",
                   dist, sync_rpm, total_dt, bow_full);
        } else {
            printf("MoveL: 同步, %d 航点(步长 %.1fmm), 位移 %.1f mm, 速度 %.1f rpm, "
                   "预计 %.2f s, 逐航点等到位, 几何弓高 ≤ %.2f mm"
                   "（整段不分段弯 %.2f mm > 预算 %.2f mm 才分段；过流保护 %s）\n",
                   n_inj, dist / (double)n_inj, dist, sync_rpm, total_dt,
                   sync_bow, bow_full, bow_budget,
                   !movl_stall_on(stall_th) ? "关闭（六轴阈值均为 0）"
                       : (n_inj > 1) ? "开启（每段下发前检查）"
                       : "本段不检查（整段一次下发，无分段点）");
        }

        if (count <= 2 || n_inj <= 1) {
            movej_joints(robot, 6, joints, q_end, sync_rpm, acc, dec,
                         start_pose, end_pose);
            sync_finish(sync_t0, total_dt);
            return;
        }

        nr_gap = movl_noread_gap_ms();

        for (j = 0; j < 6; j++) {
            if (robot_is_masked(robot, joints[j])) continue;
            motor_set_profile(robot, joints[j], acc, dec);
        }
        (void)dist_axis;

        for (i = 1; i < count; i += stride) {
            if (movl_stall_guard(robot, stall_th)) {
                sync_finish(sync_t0, total_dt);
                return;
            }
            for (j = 0; j < 6; j++) {
                if (robot_is_masked(robot, joints[j])) continue;
                tgt[joints[j]] = DEG2STEPS(q_seq[i][j] + zero[joints[j] - 1],
                                           red[joints[j] - 1]);
            }
            movl_set_seg_speed(robot, joints, q_seq[last_i], q_seq[i], red, sync_rpm);
            movej_issue_noread(robot, joints, tgt, nr_gap);
            last_i = i;
            remain = 0;
            for (j = 0; j < 6; j++) {
                if (robot_is_masked(robot, joints[j])) continue;
                pend[joints[j]] = 1;
                remain++;
            }
            if (remain > 0)
                movej_wait(robot, joints, tgt, pend, remain, start_pose, end_pose);
        }
        if (last_i != count - 1) {
            for (j = 0; j < 6; j++) {
                if (robot_is_masked(robot, joints[j])) continue;
                tgt[joints[j]] = DEG2STEPS(q_end[j] + zero[joints[j] - 1],
                                           red[joints[j] - 1]);
            }
            movl_set_seg_speed(robot, joints, q_seq[last_i], q_end, red, sync_rpm);
            movej_issue_noread(robot, joints, tgt, nr_gap);
        }

        remain = 0;
        for (j = 0; j < 6; j++) {
            if (robot_is_masked(robot, joints[j])) continue;
            pend[joints[j]] = 1;
            remain++;
        }
        if (remain > 0)
            movej_wait(robot, joints, tgt, pend, remain, start_pose, end_pose);
        sync_finish(sync_t0, total_dt);

        if (sync_bow > 0.0 && g_dev_peak > 3.0 * sync_bow) {
            printf("        [注意] 实测偏差是几何弓高的 %.1f 倍 ⇒ 不是「分段不够密」。\n"
                   "               按段速度归一化修好之后（2026-09-18），这条线在本机上\n"
                   "               实测已经≈几何弓高（4 段：0.26mm vs 预测 0.14mm）。\n"
                   "               现在还这么大，先怀疑这三条：\n"
                   "               ① 某轴没同时到位 —— 看上面逐行采样里有没有某个 J 提前\n"
                   "                  停住不动了（旧 bug 的典型症状，若复现请查\n"
                   "                  movl_set_seg_speed 是否真的在每段都被调用到）；\n"
                   "               ② 回差/齿隙 —— 正反向各跑一次，差值若稳定在 0.1mm 量级\n"
                   "                  且方向相关，就是它，调分段无效；\n"
                   "               ③ 负载/皮带打滑 —— 换低速（第 7 参数调小）复测。\n",
                   g_dev_peak / sync_bow);
        }

        return;
    }

    {
        double bow = 0.0;
        for (i = 1; i < count; i++) {
            double d = movl_bow_mm(q_seq[i - 1], q_seq[i], start_pose, end_pose);
            if (d > bow) bow = d;
        }
        printf("MoveL: %d 段, 步长 %.1f mm, 位移 %.1f mm, 节拍 %.3f s, 预计 %.2f s, "
               "模式 %s, 几何弓高 ≤ %.2f mm（过流保护 %s）",
               count - 1, step_mm, dist, (count > 1) ? seg_dt[0] : 0.0, total_dt,
               (cmd->movl_mode == MOVL_MODE_STREAM) ? "stream"
                   : (cmd->movl_mode == MOVL_MODE_SMOOTH) ? "smooth"
                   : (cmd->movl_mode == MOVL_MODE_SYNC) ? "sync" : "step", bow,
               movl_stall_on(stall_th) ? "开启" : "关闭（六轴阈值均为 0）");
        {
            const double bow_budget = movl_bow_budget();
            if (bow > bow_budget * 1.2 + 0.05)
                printf("\n[警告] 弓高 %.2f mm 超出预算 %.2f mm（段数被节拍压到 %d 段）："
                       "降速或调大 ACC/DEC 可把节拍拉长、段数变多；"
                       "也可调大 bow_mm 接受这条弧", bow, bow_budget, count - 1);
        }
        if (count > 1 && seg_dt[0] * 1000.0 < (double)(acc + dec)) {
            static double hint_seg_ms = -1.0;
            static int    hint_ramp   = -1;
            double seg_ms = seg_dt[0] * 1000.0;
            if (fabs(seg_ms - hint_seg_ms) > 0.5 || (acc + dec) != hint_ramp) {
                hint_seg_ms = seg_ms;
                hint_ramp   = acc + dec;
                printf("\n[提示] 段时长 %.0f ms 短于加减速时间之和 %d ms：每段都爬不完坡就要刹车，"
                       "实际耗时会明显长于预计。想又快又顺就调小 ACC/DEC，"
                       "想更少停顿就调大 ACC/DEC（但会压低段数、弓高变大）",
                       seg_ms, acc + dec);
            }
        }
        printf("\n");
    }

    int32_t s_tgt[7] = {0};
    uint8_t s_pend[7] = {0};
    int s_remain = 0;
    uint32_t mv_t0 = GetTickCount();
    uint32_t iss_sum = 0, iss_max = 0, iss_n = 0;
    uint32_t iss_tx = 0;
    int probe_n = 0;
    for (i = 1; i < count; i++) {
        if (movl_stall_guard(robot, stall_th)) return;
        if (movl_stall_on(stall_th)) {
            double worst = 0.0;
            for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
                int cur;
                double r;
                if (stall_th[j] <= 0) continue;
                if (robot_is_masked(robot, j + 1)) continue;
                cur = robot_read_current_ma(robot, j + 1);
                if (cur <= 0) continue;
                r = (double)cur / (double)stall_th[j];
                if (r > worst) worst = r;
            }
            if (worst > 0.6) {
                double factor = 0.8 / worst;
                if (factor > 1.0) factor = 1.0;
                speed_rpm = base_rpm * factor;
                if (speed_rpm < 1.0) speed_rpm = 1.0;
                {
                    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
                    for (j = 0; j < 6; j++) vmax[j] = speed_rpm * 6.0 / (double)red[j];
                }
                if ((count - i) >= 2)
                    line_time_table(&q_seq[i], count - i, vmax, &seg_dt[i], NULL);
            }
        }

        double seg_speed = 0.0;
        {
            const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
            double t_eff = seg_dt[i - 1] - (double)(acc + dec) / 2000.0;
            double t_floor = 0.25 * seg_dt[i - 1];
            if (t_eff < t_floor) t_eff = t_floor;
            for (j = 0; j < 6; j++) {
                double d = fabs(q_seq[i][j] - q_seq[i - 1][j]) * (double)red[j];
                double rpm = d / t_eff / 6.0;
                if (rpm > seg_speed) seg_speed = rpm;
            }
        }
        if (seg_speed < 1.0) seg_speed = 1.0;

        if (cmd->movl_mode == MOVL_MODE_STREAM) {
            uint32_t t0 = GetTickCount();
            int tx_before = g_tx_written;
            int r = movej_issue(robot, 6, joints, q_seq[i], q_seq[i - 1],
                                seg_speed, acc, dec, (i == 1) ? 1 : 0, s_tgt, s_pend);
            if (r > 0) s_remain = r;
            iss_tx += (uint32_t)(g_tx_written - tx_before);
            uint32_t period_ms = (uint32_t)(seg_dt[i - 1] * 1000.0);
            if (period_ms < MOVL_STREAM_MIN_MS) period_ms = MOVL_STREAM_MIN_MS;
            uint32_t used = GetTickCount() - t0;
            iss_sum += used; iss_n++;
            if (used > iss_max) iss_max = used;

            if (used + MOVL_STREAM_PROBE_MS <= period_ms) {
                movl_dev_sample(robot, start_pose, end_pose);
                probe_n++;
                used = GetTickCount() - t0;
            }
            if (used < period_ms) Sleep(period_ms - used);
            status_line("MoveL stream %d/%d 段", i, count - 1);
        } else {
            movej_joints(robot, 6, joints, q_seq[i], seg_speed, acc, dec,
                         start_pose, end_pose);
        }
    }

    status_clear();
    if (cmd->movl_mode == MOVL_MODE_STREAM) {
        if (s_remain > 0) movej_wait(robot, joints, s_tgt, s_pend, s_remain,
                                     start_pose, end_pose);
        printf("MoveL stream %d 段完成，实际耗时 %.2f s（预计 %.2f s），"
               "段时长 %.0f ms（ACC/DEC = %d/%d ms）\n",
               count - 1, (GetTickCount() - mv_t0) / 1000.0, total_dt,
               (count > 1) ? seg_dt[0] * 1000.0 : 0.0, acc, dec);
        if (probe_n > 0)
            printf("    （偏差为全程峰值：段内采样 %d 次 + 收尾等待）\n", probe_n);
        else
            printf("    [注意] 节拍太短，段内一次都没采到 ⇒ 下面的偏差只覆盖最后\n"
                   "           一段的收尾，不代表全程，不要拿它和 sync 比。\n"
                   "           想采到就放慢速度或调大 ACC/DEC，把节拍拉长到 %d ms 以上。\n",
                   MOVL_STREAM_PROBE_MS);
        dev_report("stream ");
        if (iss_n > 0) {
            double per = (double)iss_sum / (double)iss_n;
            double per_tx = (double)iss_sum / (double)iss_tx;
            double seg_ms = (count > 1) ? seg_dt[0] * 1000.0 : 0.0;
            printf("    下发：每节拍 %.0f ms（峰值 %.0f ms），单事务 %.1f ms",
                   per, (double)iss_max, per_tx);
            if (seg_ms > 0.0)
                printf("，占段时长 %.0f%%", 100.0 * per / seg_ms);
            printf("\n");
            if (seg_ms > 0.0 && per > 0.25 * seg_ms) {
                static int beat_hint_done = 0;
                if (!beat_hint_done) {
                    beat_hint_done = 1;
                    printf("    [提示] 节拍超过段时长的 25%%，六轴起步严重不同步，末端偏差会远大于弓高。\n"
                           "           优先降低单事务耗时（换低延迟转换器 / 提高波特率 / 广播下发），"
                           "其次加长段时长（调大 ACC/DEC 或放慢速度）。\n");
                }
            }
            if (per_tx > 8.0) {
                static int lat_hint_done = 0;
                if (!lat_hint_done) {
                    lat_hint_done = 1;
                    printf("    [提示] 单事务 %.1f ms 明显偏慢（115200 下纯线上时间约 1.8 ms）。\n"
                           "           FTDI / CP210x / PL2303 芯片：设备管理器 → 端口(COM 和 LPT) →\n"
                           "           你的串口 → 端口设置 → 高级 → “延迟计时器(毫秒)” 16 改 1 →\n"
                           "           确定后重开本程序，通常可提速数倍。\n"
                           "           【CH340（VID_1A86&PID_7523）不适用】：其驱动没有这一项，改不了。\n"
                           "           只能换 FTDI/CP210x 转换器；提波特率收益有限——实测 8.4 倍开销里\n"
                           "           线上只占 1.8 ms，波特率翻倍最多省一成，大头是等响应与系统调度。\n",
                           per_tx);
                }
            }
        }
    } else {
        dev_report("step ");
    }
}

static double flange_tilt_deg(const double pose[4][4])
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double c = -pose[2][2];
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return acos(c) * RAD2DEG;
}

void cmd_getpos(Robot *robot)
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double q[6];
    int ok[6];
    double pose[4][4];
    double xyz[3], rpy[3];
    int all_ok = 1;

    for (int i = 0; i < 6; i++) {
        q[i] = robot_read_position_deg(robot, i + 1, &ok[i]);
        if (!ok[i]) all_ok = 0;
    }

    dh_forward(DH_TABLE, q, pose);
    dh_pose_to_xyz_rpy(pose, xyz, rpy);

    for (int i = 0; i < 6; i++) {
        printf(i ? ", J%d = %.2f°" : "J%d = %.2f°", i + 1, q[i]);
    }
    printf("\n");

    printf("X = %.2f , Y = %.2f , Z = %.2f , Rx = %.2f , Ry = %.2f , Rz = %.2f\n",
           xyz[0], xyz[1], xyz[2],
           rpy[0] * RAD2DEG, rpy[1] * RAD2DEG, rpy[2] * RAD2DEG);
    printf("法兰倾角 = %.2f°  %s   (q2+q3+q5 = %.2f)\n",
           flange_tilt_deg(pose),
           flange_tilt_deg(pose) < 0.05 ? "垂直于地面" : "歪了",
           q[1] + q[2] + q[4]);

    if (!all_ok) printf("[警告] 部分关节读取失败，坐标按读取值计算\n");
}

void cmd_fk(const ParsedCmd *cmd)
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    static const double DEG2RAD = 3.14159265358979323846 / 180.0;
    double q[6], pose[4][4], xyz[3], rpy[3], tilt;
    double acc[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double t[4][4];
    int i, j, k;

    for (i = 0; i < 6; i++) q[i] = cmd->angles[i];

    dh_forward(DH_TABLE, q, pose);
    dh_pose_to_xyz_rpy(pose, xyz, rpy);
    tilt = flange_tilt_deg(pose);

    printf("关节角 : J1..J6 = %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           q[0], q[1], q[2], q[3], q[4], q[5]);
    printf("末端   : X = %.2f , Y = %.2f , Z = %.2f , "
           "Rx = %.2f , Ry = %.2f , Rz = %.2f\n",
           xyz[0], xyz[1], xyz[2],
           rpy[0] * RAD2DEG, rpy[1] * RAD2DEG, rpy[2] * RAD2DEG);
    printf("法兰   : 法线 = (%.3f, %.3f, %.3f)  与竖直向下夹角 = %.2f°  %s\n",
           pose[0][2], pose[1][2], pose[2][2], tilt,
           (tilt < 0.05) ? "垂直于地面" : "歪了");
    printf("姿态和 : q2+q3+q5 = %.2f   （=180 时法兰/笔正好竖直朝下）\n",
           q[1] + q[2] + q[4]);

    printf("臂形   : 基座(0.0,0.0,0.0)");
    for (i = 0; i < 6; i++) {
        double nx[4][4];
        dh_transform(&DH_TABLE[i], q[i] * DEG2RAD + DH_TABLE[i].theta_offset, t);
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 4; k++) {
                nx[j][k] = 0.0;
                for (int m = 0; m < 4; m++) nx[j][k] += acc[j][m] * t[m][k];
            }
        }
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 4; k++) acc[j][k] = nx[j][k];
        }
        printf(" → J%d(%.1f,%.1f,%.1f)", i + 1, acc[0][3], acc[1][3], acc[2][3]);
    }
    printf("\n");
}

void cmd_diag(Robot *robot, const ParsedCmd *cmd)
{
    (void)cmd;
    const int N = 30;
    uint32_t n = 0, n_nr = 0;
    double fl = 0.0, wr = 0.0, rd = 0.0, tot = 0.0, nr = 0.0;
    int i, ok = 0;

    if (robot == NULL) return;

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);
    bus_drain();

    modbus_stats_reset();
    for (i = 0; i < N; i++) {
        int32_t p = motor_read_position(robot, 1, &ok);
        (void)p;
    }
    modbus_stats_get(&n, &fl, &wr, &rd, &tot, &n_nr, &nr);

    const double tot_rd = tot;

    printf("总线体检：%u 次完整事务（读 J1 位置，只读不动臂）\n", n);
    printf("    flush (PurgeComm)  %7.2f ms\n", fl);
    printf("    write  (下发请求)  %7.2f ms\n", wr);
    printf("    read   (等响应)    %7.2f ms   ← 通常就是大头\n", rd);
    printf("    ── 单事务合计      %7.2f ms\n", tot);
    printf("    一轮 6 轴 %7.1f ms  ⇒  理论刷新率 %6.1f Hz\n",
           tot * 6.0, 1000.0 / (tot * 6.0));

    {
        const int R = 10;
        LARGE_INTEGER freq, a, b;
        double sum = 0.0;
        QueryPerformanceFrequency(&freq);
        for (i = 0; i < R; i++) {
            int j;
            QueryPerformanceCounter(&a);
            for (j = 1; j <= 6; j++) {
                int32_t p = motor_read_position(robot, j, &ok);
                (void)p;
            }
            QueryPerformanceCounter(&b);
            sum += (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
        }
        printf("实测一轮：%d 次\"连读 J1..J6\"，平均 %7.2f ms  ⇒  刷新率 %6.1f Hz\n",
               R, sum / R, 1000.0 / (sum / R));
        printf("          对比推算值 %.2f ms（单事务×6）——两者差得越多，"
               "说明批处理/调度的影响越大\n", tot_rd * 6.0);
    }
    {
        int32_t pos0 = motor_read_position(robot, 1, &ok);
        if (!ok) {
            printf("[警告] 读 J1 位置失败，跳过只写不读对照\n");
            goto done;
        }
        modbus_stats_reset();
        for (i = 0; i < N; i++)
            motor_move_abs_noread(robot, 1, pos0);
        modbus_stats_get(&n, &fl, &wr, &rd, &tot, &n_nr, &nr);
        printf("对照：%u 次只写不读（写 J1 当前位置，原地不动）\n", n_nr);
        printf("    ── 单事务合计      %7.2f ms\n", nr);
        printf("    一轮 6 轴 %7.1f ms  ⇒  理论刷新率 %6.1f Hz\n",
               nr * 6.0, 1000.0 / (nr * 6.0));
        printf("    （上界：省掉等响应最多 %.1f 倍；共享总线上还要再留出从站响应时间）\n",
               (nr > 0.0 && tot_rd > 0.0) ? (tot_rd / nr) : 0.0);
    }
    {
        uint32_t baud_now = serial_get_baud();
        double wire;
        if (baud_now == 0) baud_now = MODBUS_BAUDRATE;
        wire = 21.0 * 10.0 / (double)baud_now * 1000.0;
        printf("参考：%u bps 下单事务纯线上时间 %.2f ms（21 字节 × 10 bit）\n",
               (unsigned)baud_now, wire);
        printf("      实测/线上 = %.1f 倍 ⇒ 其余全是等待与系统开销\n",
               (wire > 0.0) ? (tot_rd / wire) : 0.0);
        printf("      转换器吞吐上界约 %.0f 事务/秒，当前实跑约 %.0f 事务/秒\n",
               1000.0 / wire, 1000.0 / ((tot_rd > 0.0) ? tot_rd : 1.0));
        printf("      单总线 %u bps 下【六轴刷新率天花板】约 %.0f Hz（读写事务，等响应）\n",
               (unsigned)baud_now, 1000.0 / ((wire + 0.30) * 6.0));
        printf("      改\"只写不等响应\"的上界约 %.0f Hz（未计从站响应占线时间）\n",
               1000.0 / (nr * 6.0));
    }
    printf("\n");

done:
    Sleep(20);
    bus_drain();
    monitor_pause_active(0);
}

#define TAB_REG_SIZE    0x00AA
#define TAB_REG_PTR     0x00AB
#define TAB_REG_BASE    0x00AC
#define TAB_REG_EXEC    0x00DD
#define TAB_DATA_ADDR   500
#define TAB_N_POINT     3

static void cmd_tabtest(Robot *robot, const ParsedCmd *cmd)
{
    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    int j = cmd->joint;
    int32_t seg = DEG2STEPS(cmd->angle_deg, red[j - 1]);
    double rpm = cmd->speed_rpm;
    int i, n_div = 0;
    uint32_t t0, t_start;
    double est_s;
    int32_t v_prev = 0, v_min_run = 0;
    int seen_run = 0;
    int32_t pos0 = 0;
    int ok = 0;

    if (seg == 0) {
        printf("[错误] 每段 %.3f° 换算后是 0 步，调大每段角度\n", cmd->angle_deg);
        return;
    }
    est_s = (double)seg / (rpm * (double)ENCODER_STEPS_PER_REV / 60.0);
    if (est_s < 0.05) est_s = 0.05;

    printf("tabtest 关节%d：%d 段相对位移表，每段 %+.2f° = %+d 步，%.0f rpm（每段约 %.2f s）\n",
           j, TAB_N_POINT, cmd->angle_deg, seg, rpm, est_s);
    printf("  表数据实际地址 %d，表开始地址寄存器 = %d - 300 = %d\n",
           TAB_DATA_ADDR, TAB_DATA_ADDR, TAB_DATA_ADDR - 300);
    printf("  ⚠️ 轴会停在 +%.2f°（3×%.2f）处，确认行程内无障碍\n",
           3.0 * cmd->angle_deg, cmd->angle_deg);

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    pos0 = motor_read_position(robot, j, &ok);

    if (motor_set_speed(robot, j, rpm) != ERR_NONE)
        printf("[警告] 关节%d 速度设置失败\n", j);

    for (i = 0; i < TAB_N_POINT; i++) {
        if (motor_write_i32(robot, j, (uint16_t)(TAB_DATA_ADDR + 2 * i), seg) != ERR_NONE) {
            printf("[错误] 写表数据[%d] 失败（地址 %d），实验中止\n", i, TAB_DATA_ADDR + 2 * i);
            monitor_pause_active(0);
            return;
        }
    }
    motor_write_u16(robot, j, TAB_REG_SIZE, TAB_N_POINT);
    motor_write_u16(robot, j, TAB_REG_PTR, 0);
    motor_write_u16(robot, j, TAB_REG_BASE, (uint16_t)(TAB_DATA_ADDR - 300));

    printf("  下发执行 0x00DD = 0x%04X（相对位置 / 指针 +1）\n", 0x8001u);
    t0 = GetTickCount();
    if (motor_write_u16(robot, j, TAB_REG_EXEC, 0x8001u) != ERR_NONE) {
        printf("[错误] 执行表格失败（0x00DD 写入失败）\n");
        monitor_pause_active(0);
        return;
    }

    printf("\n%8s %12s %10s\n", "t(ms)", "速度(0.01rpm)", "速度(rpm)");
    t_start = GetTickCount();
    while ((int32_t)(GetTickCount() - t_start) < (int32_t)((est_s * TAB_N_POINT + 2.0) * 1000.0)) {
        int32_t v = motor_read_speed_raw(robot, j);
        uint32_t el = GetTickCount() - t_start;
        if (v >= 0) {
            printf("%8lu %12ld %10.2f\n", (unsigned long)el, (long)v, (double)v / 100.0);
            if (v > 50) seen_run = 1;
            if (seen_run && v_prev > 50 && v <= 50) {
                n_div++;
                printf("        ^^^ 速度回落（段间？）\n");
            }
            if (seen_run && v > 50 && (v_min_run == 0 || v < v_min_run)) v_min_run = v;
            v_prev = v;
        }
        if (n_div >= TAB_N_POINT) break;
    }
    printf("\n执行耗时统计：下发 %lu ms；段间速度回落 %d 次；运行中最低速度 %.2f rpm\n",
           (unsigned long)(GetTickCount() - t0), n_div, (double)v_min_run / 100.0);

    {
        int32_t pos1 = motor_read_position(robot, j, &ok);
        if (ok && pos0 != 0) {
            double moved_deg = (double)(pos1 - pos0) / (double)red[j - 1] * 360.0 / 10000.0;
            double seg_deg = cmd->angle_deg;
            printf("位置：%ld → %ld 步 = %+.2f°，共 %d 段 ⇒ 一次 0x00DD 走了 %.1f 段\n",
                   (long)pos0, (long)pos1, moved_deg, TAB_N_POINT, moved_deg / seg_deg);
            printf("判读：%.1f 段 ⇒ %s\n", moved_deg / seg_deg,
                   (moved_deg / seg_deg > 2.5)
                       ? "【一次走完整个表】驱动器自己连续执行 ⇒ 表格模式成立，值得全量开发"
                       : "【一次只走一个点】表格省不掉总线往返 ⇒ 这条路不成立，"
                         "要\"驱动器自己连续走\"只能靠编程区命令(0x00DB)，但手册未给指令格式");
        } else {
            printf("位置读取失败，无法判断走了几段\n");
        }
    }
    printf("速度曲线判读：回落 %d 次 ⇒ %s\n", n_div,
           (n_div == 0) ? "段间不归零" : "每段末都归零（与 ACC+DEC 对应，见 accel 命令）");
    monitor_pause_active(0);
}

static void cmd_busrate(Robot *robot, const ParsedCmd *cmd)
{
    const int N = 30;
    int j = (cmd != NULL && cmd->joint >= 1 && cmd->joint <= ROBOT_JOINT_COUNT)
                ? cmd->joint : 1;
    int32_t v[ROBOT_JOINT_COUNT + 1] = {0};
    int vok[ROBOT_JOINT_COUNT + 1] = {0};
    int i, k, ok = 0, succ, n_online = 0;
    LARGE_INTEGER freq, a, b;
    double ms, t_read = 0.0, t_noread = 0.0, t_round_nr = 0.0, t_round_rd = 0.0;

    if (robot == NULL) return;

    for (k = 1; k <= ROBOT_JOINT_COUNT; k++) {
        if (motor_read_i32(robot, k, LEESN_REG_VEL_RUN, &v[k]) == ERR_NONE) {
            vok[k] = 1;
            n_online++;
        }
    }
    if (!vok[j]) {
        printf("[错误] 读关节%d 运行速度(0x00D8) 失败 ⇒ 该轴不在线，busrate 中止\n", j);
        return;
    }
    if (n_online == 0) {
        printf("[错误] 六轴全部无响应 ⇒ 总线不通，busrate 中止\n");
        return;
    }

    printf("busrate：RS485 总线极限速率实测（探针关节%d，每档 %d 次，高精度计时）\n", j, N);
    printf("  探针 = 把 0x00D8(运行速度) 写回原值 ⇒ 电机不动、速度不变。在线 %d/6 轴\n",
           n_online);

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);
    bus_drain();
    QueryPerformanceFrequency(&freq);

    succ = 0;
    QueryPerformanceCounter(&a);
    for (i = 0; i < N; i++) { motor_read_position(robot, j, &ok); if (ok) succ++; }
    QueryPerformanceCounter(&b);
    ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
    t_read = ms / N;
    printf("[1] 单轴 读位置            %7.2f ms/次  %7.1f Hz   (成功 %d/%d)\n",
           t_read, 1000.0 / t_read, succ, N);

    succ = 0;
    QueryPerformanceCounter(&a);
    for (i = 0; i < N; i++)
        if (motor_write_i32(robot, j, LEESN_REG_VEL_RUN, v[j]) == ERR_NONE) succ++;
    QueryPerformanceCounter(&b);
    ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
    printf("[2] 单轴 写 + 等响应       %7.2f ms/次  %7.1f Hz   (成功 %d/%d)\n",
           ms / N, N * 1000.0 / ms, succ, N);

    succ = 0;
    QueryPerformanceCounter(&a);
    for (i = 0; i < N; i++)
        if (motor_write_i32_noread(robot, j, LEESN_REG_VEL_RUN, v[j]) == ERR_NONE) succ++;
    QueryPerformanceCounter(&b);
    ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
    t_noread = ms / N;
    printf("[3] 单轴 只写不等响应      %7.2f ms/次  %7.1f Hz   (成功 %d/%d) ← 推帧时间\n",
           t_noread, 1000.0 / t_noread, succ, N);

    QueryPerformanceCounter(&a);
    for (i = 0; i < N; i++)
        for (k = 1; k <= ROBOT_JOINT_COUNT; k++)
            if (vok[k]) motor_write_i32_noread(robot, k, LEESN_REG_VEL_RUN, v[k]);
    QueryPerformanceCounter(&b);
    ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
    t_round_nr = ms / N;
    printf("[4] 六轴一轮 只写不等响应  %7.2f ms/轮  %7.1f Hz   ← 并线顺序发 6 帧（不含回帧占线）\n",
           t_round_nr, 1000.0 / t_round_nr);

    Sleep(20);
    bus_drain();

    QueryPerformanceCounter(&a);
    for (i = 0; i < N; i++)
        for (k = 1; k <= ROBOT_JOINT_COUNT; k++) motor_read_position(robot, k, &ok);
    QueryPerformanceCounter(&b);
    ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
    t_round_rd = ms / N;
    printf("[5] 六轴一轮 读位置        %7.2f ms/轮  %7.1f Hz   ← 当前整机更新率\n",
           t_round_rd, 1000.0 / t_round_rd);

    printf("\n—— 判读 ——\n");
    printf("  [1]≈[2] ⇒ 读与写成本相同，瓶颈都在\"等响应\"：占单事务 %.0f%%\n",
           t_read > 1e-6 ? (t_read - t_noread) / t_read * 100.0 : 0.0);
    {
        uint32_t baud_now = serial_get_baud();
        double wire13;
        if (baud_now == 0) baud_now = MODBUS_BAUDRATE;
        wire13 = 13.0 * 10.0 / (double)baud_now * 1000.0;
        printf("  参考：%u bps 下 13 字节写帧纯线上 %.2f ms（实测 [3] = %.2f ms）\n",
               (unsigned)baud_now, wire13, t_noread);
        if (t_noread <= wire13 * 1.3) {
            printf("        [3] 已贴近线上时间 ⇒ WriteFile 是\"等帧发完\"才返回，没法再快\n");
        } else {
            printf("        [3] 比线上时间多 %.2f ms ⇒ 差额是 USB 发送路径开销\n",
                   t_noread - wire13);
        }
        printf("        ⚠️ 提波特率【只影响 [3]/[4] 这一项】；[1]/[2]/[5] 的\"等响应\"不受它影响\n");
    }
    printf("  现在：六轴一轮 %.1f ms ⇒ %.1f Hz（[5]，整机真实更新率）\n",
           t_round_rd, 1000.0 / t_round_rd);
    printf("  并线上界：六轴一轮 %.1f ms ⇒ %.1f Hz（[4]，6 帧顺序发，仍要 6 倍时间）\n",
           t_round_nr, 1000.0 / t_round_nr);
    printf("  ★ 拆成 6 根独立总线（每轴一个 USB-RS485）并行后，周期 = 单轴一次：\n");
    printf("      物理下界 %.2f ms（[3]，不含从站回帧占线）\n", t_noread);
    printf("      保守上界 %.2f ms（[2]，连 PC 侧接收延迟都算进去了）\n", t_read);
    printf("      ⚠️ 真实周期 = [3] + 从站回帧占线（半双工，8 字节 ≈ 0.7 ms）+ 驱动器处理时间\n");
    printf("         ⇒ 预计 2~5 ms，即 200~500 Hz，是现在的 %.0f~%.0f 倍\n",
           t_round_rd / 5.0, t_round_rd / 2.0);
    printf("      想直接吃到 [3] 的 %.0f Hz：必须让从站【不回帧】= 用广播（见 bcast 命令）\n",
           1000.0 / t_noread);
    printf("  ⚠️ 所有这一切的前提：驱动器在运动中收到新目标时【从当前速度续规划】\n"
           "     而不是强制归零。否则更新率再高，每帧仍是一次起步+刹车 ⇒ 依旧一卡一卡。\n"
           "     这一点用 tabtest 或单轴高频改目标实验回答。\n");

    Sleep(20);
    bus_drain();
    monitor_pause_active(0);
}

#define LOOP_FRAME_LEN 13

#define LOOP_REF_WITH_MOTOR_MS 1.70
#define LOOP_GAP_MS            2.00
#define LOOP_T_DRV_MIN_MS      1.39

static const int         kRtsModes[3] = {2, 1, 0};
static const char *const kRtsNames[3] = {
    "TOGGLE(发送时高,默认)", "ENABLE(常高)", "DISABLE(常低)"
};

static const char *rts_name(int mode)
{
    int k;
    for (k = 0; k < 3; k++) {
        if (kRtsModes[k] == mode) return kRtsNames[k];
    }
    return "?";
}


static SerialPort *g_loop_aux = NULL;

static int loop_aux_write(const uint8_t *buf, int len)
{
    return (g_loop_aux != NULL) ? serial_write(g_loop_aux, buf, (size_t)len) : -1;
}
static int loop_aux_read(uint8_t *buf, int cap, int timeout_ms)
{
    return (g_loop_aux != NULL)
               ? serial_read(g_loop_aux, buf, (size_t)cap, (uint32_t)timeout_ms) : -1;
}
static void loop_aux_flush(void)
{
    if (g_loop_aux != NULL) {
        serial_flush(g_loop_aux);
    }
}

typedef struct {
    int    ok_n;
    int    mismatch;
    int    partial;
    int    timeout;
    double sum;
    double tmin, tmax;
} LoopStat;

static void loop_measure(int N, int (*wr)(const uint8_t *, int),
                         int (*rd)(uint8_t *, int, int), void (*fl)(void),
                         const uint8_t *tx, uint8_t *rx, LoopStat *st)
{
    LARGE_INTEGER freq, a, b;
    int i;

    memset(st, 0, sizeof(*st));
    st->tmin = 1e9;
    QueryPerformanceFrequency(&freq);

    for (i = 0; i < N; i++) {
        int need = LOOP_FRAME_LEN, off = 0, r;
        double ms;

        fl();
        QueryPerformanceCounter(&a);
        if (wr(tx, LOOP_FRAME_LEN) <= 0) {
            QueryPerformanceCounter(&b);
            continue;
        }
        while (need > 0) {
            r = rd(rx + off, need, 50);
            if (r <= 0) break;
            off += r;
            need -= r;
        }
        QueryPerformanceCounter(&b);
        ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;

        if (off == 0) { st->timeout++; continue; }
        if (off < LOOP_FRAME_LEN) st->partial++;
        st->sum += ms;
        if (ms < st->tmin) st->tmin = ms;
        if (ms > st->tmax) st->tmax = ms;
        if (off == LOOP_FRAME_LEN) {
            if (memcmp(rx, tx, LOOP_FRAME_LEN) == 0) st->ok_n++;
            else st->mismatch++;
        }
    }
}

static double loop_report(const char *label, const LoopStat *st, int N)
{
    int cnt = N - st->timeout;
    double rtt = (cnt > 0) ? st->sum / (double)cnt : 0.0;

    printf("%s  平均 %6.2f ms（最小 %6.2f / 最大 %6.2f）⇒ %6.1f Hz   一致 %d/%d",
           label, rtt, (st->tmin < 1e9 ? st->tmin : 0.0), st->tmax,
           (rtt > 1e-6) ? 1000.0 / rtt : 0.0, st->ok_n, N);
    if (st->mismatch || st->partial || st->timeout) {
        printf("（不一致 %d 半帧 %d 超时 %d）", st->mismatch, st->partial, st->timeout);
    }
    printf("\n");
    return rtt;
}

static void cmd_looptest(Robot *robot, const ParsedCmd *cmd)
{
    const CommOps *ops = modbus_comm_get();
    uint8_t tx[LOOP_FRAME_LEN];
    uint8_t rx[LOOP_FRAME_LEN];
    int N = (cmd != NULL && cmd->joint >= 1) ? cmd->joint : 30;
    int baud_req = (cmd != NULL) ? (int)cmd->param : 0;
    int aux_mode = (cmd != NULL && cmd->rel == 1 && cmd->raw[0] != '\0');
    uint32_t baud_orig, baud_now;
    int i, rts_orig = 2, loop_ok = 0;
    double wire, t_burst = 0.0, rtt_fwd = 0.0, rtt_rev = 0.0;
    LoopStat st_fwd, st_rev;
    LARGE_INTEGER freq, a, b;

    (void)robot;

    if (ops == NULL || ops->write_frame == NULL || ops->read_frame == NULL) {
        printf("[错误] 串口未初始化，looptest 中止\n");
        return;
    }

    for (i = 0; i < LOOP_FRAME_LEN; i++) {
        tx[i] = (uint8_t)(0x11 + i * 7);
    }

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    baud_orig = serial_get_baud();
    if (baud_orig == 0) {
        printf("[警告] 读不到当前波特率（串口未打开？），仍按当前速率测\n");
    }

    printf("looptest：RS485 转换器纯工具测试（脱离电机），每档 %d 次\n", N);
    printf("         接线：A、B【悬空】—— 千万别把 A 和 B 接在一起（差分被短路）\n");
    if (baud_req > 0 && (uint32_t)baud_req != baud_orig) {
        if (serial_set_baud((uint32_t)baud_req) != 0) {
            printf("[错误] 切到 %d bps 失败 ⇒ 该转换器/驱动不支持这个速率\n", baud_req);
            printf("       仍按 %u bps 继续测\n", baud_orig);
        } else {
            printf("波特率：%u → %d bps（测完自动改回 %u）\n", baud_orig, baud_req, baud_orig);
        }
    } else {
        printf("波特率：%u bps（未改；想扫别的档位就写 looptest:%d:921600）\n", baud_orig, N);
    }
    baud_now = serial_get_baud();
    wire = (double)LOOP_FRAME_LEN * 10.0 /
           (double)(baud_now ? baud_now : 115200) * 1000.0;
    printf("参考：%u bps 下 %d 字节纯线上时间 %.3f ms（单向）\n", baud_now, LOOP_FRAME_LEN, wire);

    if (aux_mode) {
        g_loop_aux = serial_open(cmd->raw, baud_now);
        if (g_loop_aux == NULL) {
            printf("[错误] 打开辅口 %s 失败 ⇒ 退回单模块回环模式\n", cmd->raw);
            aux_mode = 0;
        } else {
            serial_set_timeout(g_loop_aux, 200, 200);
            printf("辅口：%s —— 两个转换器 A-A / B-B 对接，主口发、辅口收\n", cmd->raw);
        }
    }
    printf("\n");

    if (aux_mode) {
        loop_measure(N, ops->write_frame, loop_aux_read, loop_aux_flush, tx, rx, &st_fwd);
        rtt_fwd = loop_report("[1]  主口发 → 辅口收", &st_fwd, N);
        loop_measure(N, loop_aux_write, ops->read_frame, ops->flush, tx, rx, &st_rev);
        rtt_rev = loop_report("[1b] 辅口发 → 主口收", &st_rev, N);
        loop_ok = (st_fwd.ok_n > 0 || st_rev.ok_n > 0);
    } else {
        rts_orig = serial_get_rts();
        if (rts_orig < 0) rts_orig = 2;
        {
            int k, best = -1, best_got = 0;
            for (k = 0; k < 3; k++) {
                int got_total = 0, t;
                if (serial_set_rts(kRtsModes[k]) != 0) {
                    printf("[0] RTS %-22s 设置失败，跳过\n", kRtsNames[k]);
                    continue;
                }
                for (t = 0; t < 3; t++) {
                    int need = LOOP_FRAME_LEN, off = 0, r;
                    ops->flush();
                    if (ops->write_frame(tx, LOOP_FRAME_LEN) <= 0) break;
                    while (need > 0) {
                        r = ops->read_frame(rx + off, need, 50);
                        if (r <= 0) break;
                        off += r;
                        need -= r;
                    }
                    got_total += off;
                }
                printf("[0] RTS %-22s 3 次共收到 %2d 字节%s\n",
                       kRtsNames[k], got_total, got_total > 0 ? "  ← 能回环" : "");
                if (got_total > best_got) { best_got = got_total; best = k; }
            }
            if (best < 0) best = 0;
            loop_ok = (best_got > 0);
            serial_set_rts(kRtsModes[best]);
            if (loop_ok) {
                printf("    ⇒ 用 RTS %s 做正式测量（测完恢复原设置）\n", kRtsNames[best]);
            }
            printf("\n");
        }

        if (loop_ok) {
            loop_measure(N, ops->write_frame, ops->read_frame, ops->flush, tx, rx, &st_fwd);
            rtt_fwd = loop_report("[1]  单模块 A/B 回环", &st_fwd, N);
        } else {
            printf("[1]  单模块 A/B 回环  —— 跳过（三种 RTS 电平都没收到回包）\n");
        }
    }

    {
        double ms;
        int m = 0;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&a);
        for (i = 0; i < N; i++) {
            if (ops->write_frame(tx, LOOP_FRAME_LEN) > 0) m++;
        }
        QueryPerformanceCounter(&b);
        ms = (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
        t_burst = ms / N;
        ops->flush();
        printf("[2]  连发推帧（只写不等回环）  %7.2f ms/帧  %7.1f Hz   (成功 %d/%d)\n",
               t_burst, (t_burst > 1e-6) ? 1000.0 / t_burst : 0.0, m, N);
    }

    printf("\n—— 判读 ——\n");
    if (!loop_ok) {
        printf("  ★ 收不到回包 ⇒ 这个模块【不回显自己的发送】。\n");
        printf("    先排除接线：A/B 必须【悬空】，短接 ⇒ 差分恒 0 ⇒ 必然收不到。\n");
        printf("    接线没问题还收不到，就是模块的方向电路在发送期间关掉了接收。\n");
        printf("    ⇒ 单模块测不出往返；要直接测往返只能上两个转换器：\n");
        printf("      A-A / B-B 对接，跑 looptest:%d:0:COM5（COM5 换成辅口实际名字）\n", N);
        printf("    但【不用回环也能推算】—— 用另两条实测值夹出来：\n");
        {
            uint32_t baud = serial_get_baud();
            double allw, rxw, t_drv_max, t_stack_max;
            if (baud == 0) baud = MODBUS_BAUDRATE;
            allw = 21.0 * 10.0 / (double)baud * 1000.0;
            rxw  =  7.0 * 10.0 / (double)baud * 1000.0;
            t_drv_max   = LOOP_GAP_MS - rxw;
            t_stack_max = LOOP_REF_WITH_MOTOR_MS - allw - LOOP_T_DRV_MIN_MS;
            printf("      完整读事务 %.2f ms = 线上 %.2f + 驱动器应答 + USB栈\n",
                   LOOP_REF_WITH_MOTOR_MS, allw);
            printf("      nrtest 安全帧间隔 %.2f ms ⇒ 驱动器应答 ≤ %.2f ms（%.2f − 回帧 %.2f）\n",
                   LOOP_GAP_MS, t_drv_max, LOOP_GAP_MS, rxw);
            printf("      115200 下 2ms 档撞车 ⇒ 驱动器应答 > %.2f ms\n", LOOP_T_DRV_MIN_MS);
            printf("      ⇒ 驱动器应答 ≈ %.2f ~ %.2f ms，【USB 栈最多只剩 %.2f ms】\n",
                   LOOP_T_DRV_MIN_MS, t_drv_max, t_stack_max);
            printf("      ⇒ 瓶颈在【驱动器自己的处理时间】，不在工具\n");
            printf("        ⇒ 换转换器、再提波特率，剩下的空间都只有一成量级；\n");
            printf("          想再快只能【减少事务数】或【拆成多路独立总线并行】。\n");
            printf("      ⚠️ 2026-09-23 之前这里印的是「工具接收路径 ≥10.35ms、占 67%%」——\n");
            printf("         那是把 ReadFile 的驱动定时器等待误当成 USB 延迟算出来的，已作废。\n");
        }
        printf("  ⇒ 目前实测到的工具能力：单向推帧 %.2f ms/帧 = %.0f Hz（不含往返）\n",
               t_burst, (t_burst > 1e-6) ? 1000.0 / t_burst : 0.0);
    } else {
        double rtt = (rtt_fwd > 1e-6) ? rtt_fwd : rtt_rev;
        double drv = LOOP_REF_WITH_MOTOR_MS - rtt;
        if (aux_mode && rtt_fwd > 1e-6 && rtt_rev > 1e-6) {
            printf("  两个方向 %.2f / %.2f ms，接近 ⇒ 两个模块延迟相当\n", rtt_fwd, rtt_rev);
        }
        printf("  ★ 这个工具的最大往返频率 = %.1f Hz（一次完整往返 %.2f ms）\n",
               (rtt > 1e-6) ? 1000.0 / rtt : 0.0, rtt);
        printf("  ⇒ 接电机单事务 %.1f ms − 回环 %.2f ms = 驱动器侧 %.2f ms（占 %.0f%%）\n",
               LOOP_REF_WITH_MOTOR_MS, rtt, drv,
               (LOOP_REF_WITH_MOTOR_MS > 1e-6) ? drv / LOOP_REF_WITH_MOTOR_MS * 100.0 : 0.0);
        printf("     判据：\n");
        printf("       回环 RTT ≪ 驱动器侧 ⇒ 瓶颈在驱动器，换转换器/提波特率收益有限\n");
        printf("       回环 RTT ≈ 单事务   ⇒ 瓶颈真在工具，换转换器（判据=diag 的 read 段）才有用\n");
        printf("     线上时间只占 %.3f ms（单向），其余是 USB 轮询粒度 + 转换器/驱动响应\n", wire);
        printf("     想再看「提波特率有没有用」：跑 looptest:%d:921600%s\n", N,
               aux_mode ? ":COM5（换成辅口名）" : "");
    }
    printf("  注：回环 RTT 的粒度受 USB 轮询（约 1 ms）与 ReadIntervalTimeout(2 ms) 影响，\n");
    printf("      不会低于这个量级；[2] 是「不等回环」的推帧周期，比 RTT 小是正常的。\n");

    if (g_loop_aux != NULL) {
        serial_close(g_loop_aux);
        g_loop_aux = NULL;
        printf("辅口 %s 已关闭\n", cmd->raw);
    }
    if (!aux_mode && serial_get_rts() != rts_orig) {
        if (serial_set_rts(rts_orig) == 0) {
            printf("RTS 已恢复为 %s\n", rts_name(rts_orig));
        } else {
            printf("[警告] RTS 恢复失败 ⇒ 请重启程序\n");
        }
    }
    if (baud_orig != 0 && serial_get_baud() != baud_orig) {
        if (serial_set_baud(baud_orig) == 0) {
            printf("波特率已改回 %u bps\n", baud_orig);
        } else {
            printf("[警告] 波特率改回 %u 失败 ⇒ 请重启程序\n", baud_orig);
        }
    }

    monitor_pause_active(0);
}

#define LEESN_REG_VEL_START 0x0096
#define LEESN_REG_VEL_STOP  0x0097
#define LEESN_REG_ACC_MS    0x0098
#define LEESN_REG_DEC_MS    0x0099


static const struct { int code; uint32_t baud; } kDrvBaudTab[] = {
    {1, 300},    {2, 600},     {3, 1200},   {4, 2400},   {5, 4800},
    {6, 9600},   {7, 14400},   {8, 19200},  {9, 38400},  {10, 56000},
    {11, 57600}, {12, 115200}, {13, 230400}, {14, 460800}, {15, 921600}
};

static uint32_t drvbaud_lookup(int code)
{
    size_t i;
    for (i = 0; i < sizeof(kDrvBaudTab) / sizeof(kDrvBaudTab[0]); i++) {
        if (kDrvBaudTab[i].code == code) {
            return kDrvBaudTab[i].baud;
        }
    }
    return 0;
}

static void drvbaud_decode(uint16_t v, char *out, size_t sz)
{
    int code = v & 0xFF;
    int parity = (v >> 8) & 0x3;
    int stop = (v >> 10) & 0x3;
    uint32_t baud = drvbaud_lookup(code);
    const char *par_s = (parity == 0) ? "无校验"
                      : (parity == 1) ? "偶校验"
                      : (parity == 3) ? "奇校验" : "校验?";
    const char *stp_s = (stop == 0) ? "1" : (stop == 1) ? "0.5"
                      : (stop == 2) ? "2" : "1.5";
    char baud_s[32];

    if (baud != 0) {
        snprintf(baud_s, sizeof(baud_s), "%u bps", (unsigned)baud);
    } else {
        snprintf(baud_s, sizeof(baud_s), "表外码(%d)", code);
    }
    snprintf(out, sz, "码 %2d  %-14s %s  停止位 %s", code, baud_s, par_s, stp_s);
}

static void cmd_drvbaud(Robot *robot, const ParsedCmd *cmd)
{
    int code = (cmd != NULL) ? cmd->joint : 0;
    uint32_t pc_baud = (cmd != NULL && cmd->param > 0.0)
                           ? (uint32_t)(cmd->param + 0.5) : 0;
    int do_save = (cmd != NULL && strcmp(cmd->raw, "save") == 0);
    int j;

    if (robot == NULL) return;

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    if (code == 0 && !do_save) {
        printf("驱动器波特率寄存器 0x0009（手册 §6：低8位档位码 / bit9~8 校验 / bit11~10 停止位）\n");
        printf("%5s %9s %9s   %s\n", "关节", "原始值", "固件版本", "解码");
        for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
            uint16_t v = 0, fw_lo = 0, fw_hi = 0;
            char dec[96], fw[8];
            fw[0] = '\0';
            if (motor_read_u16(robot, j, LEESN_REG_BAUD_CODE, &v) != ERR_NONE) {
                printf("%5d %9s %9s   %s\n", j, "-", "-", "读取失败（该轴离线？）");
                continue;
            }
            drvbaud_decode(v, dec, sizeof(dec));
            if (motor_read_u16(robot, j, 0x0002, &fw_lo) == ERR_NONE &&
                motor_read_u16(robot, j, 0x0003, &fw_hi) == ERR_NONE) {
                char b[4];
                int k;
                b[0] = (char)(fw_lo >> 8); b[1] = (char)(fw_lo & 0xFF);
                b[2] = (char)(fw_hi >> 8); b[3] = (char)(fw_hi & 0xFF);
                for (k = 0; k < 4; k++) {
                    fw[k] = (b[k] >= 32 && b[k] < 127) ? b[k] : '.';
                }
                fw[4] = '\0';
            }
            printf("%5d %9u %9s   %s\n", j, (unsigned)v, fw[0] ? fw : "-", dec);
        }
        printf("\n只读模式。改法：\n");
        printf("  drvbaud:15:921600   广播写档位15 → PC侧切921600 → 回读验证\n");
        printf("  drvbaud:16:256000   试探手册档位表里没有的码（不 save 则断电可恢复）\n");
        printf("  drvbaud:save        广播固化 0x00DC=1（断电不丢）\n");
        printf("  手册 §6 档位：12=115200  13=230400  14=460800  15=921600\n");
        return;
    }

    if (do_save) {
        printf("广播写 0x00DC = 1（断电保存：把所有带记忆的 RAM 值写进 Flash）\n");
        printf("  ⚠️ 这一步固化的是【全部】记忆寄存器 —— 包括 ACC/DEC。\n");
        if (motor_write_u16_broadcast(robot, LEESN_REG_SAVE_CMD, 0x0001) != ERR_NONE) {
            printf("  [错误] 广播固化帧下发失败（总线错误）\n");
            return;
        }
        Sleep(500);
        printf("  已下发。断电重启驱动器后再读一次 0x0009 验证是否真的存住了。\n");
        return;
    }

    {
        uint32_t tgt = drvbaud_lookup(code);
        uint32_t cur = serial_get_baud();

        printf("广播写 0x0009 = %d", code);
        if (tgt != 0) {
            printf("（手册档位 = %u bps）\n", (unsigned)tgt);
        } else {
            printf("（⚠️ 手册档位表里没有这个码，属于试探）\n");
        }

        if (motor_write_u16_broadcast(robot, LEESN_REG_BAUD_CODE, (uint16_t)code) != ERR_NONE) {
            printf("  [错误] 广播帧下发失败（总线错误）\n");
            return;
        }
        printf("  已下发。驱动器此刻已经换速率 —— PC 侧没跟着改的话，总线就是断的。\n");
        Sleep(300);

        if (pc_baud == 0) {
            printf("\n  没给 PC 侧波特率 ⇒ 到此为止。下一步：\n");
            printf("    改 src/config/robot_config.ini 的 baudrate，重启程序\n");
            printf("    ⚠️ 想撤销：给驱动器断电重启（未 save 则回 115200）\n");
            return;
        }

        printf("\n[切 PC 侧] %u → %u bps\n", (unsigned)cur, (unsigned)pc_baud);
        if (serial_set_baud(pc_baud) != 0) {
            printf("  [错误] PC 侧切到 %u 失败（转换器/驱动不支持该速率）\n", (unsigned)pc_baud);
            printf("  ⇒ PC 侧改回 %u，并给驱动器断电重启\n", (unsigned)cur);
            serial_set_baud(cur);
            return;
        }
        Sleep(300);

        printf("\n[回读验证] 用 %u bps 读六轴 0x0009：\n", (unsigned)pc_baud);
        {
            int ok = 0;
            for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
                uint16_t v = 0;
                char dec[96];
                if (motor_read_u16(robot, j, LEESN_REG_BAUD_CODE, &v) == ERR_NONE) {
                    drvbaud_decode(v, dec, sizeof(dec));
                    printf("  J%d  读到 0x%04X  %s\n", j, (unsigned)v, dec);
                    ok++;
                } else {
                    printf("  J%d  无响应\n", j);
                }
            }
            printf("\n");
            if (ok == ROBOT_JOINT_COUNT) {
                printf("★ 六轴全部读到 ⇒ %u bps 可用，驱动器确实支持这个速率。\n",
                       (unsigned)pc_baud);
                printf("  要断电不丢：drvbaud:save\n");
                printf("  然后改 ini 的 baudrate = %u，重启程序。\n", (unsigned)pc_baud);
            } else if (ok > 0) {
                printf("★ 只有 %d/6 轴响应 ⇒ 广播丢帧，或该速率下部分轴没切过来。\n", ok);
                printf("  ⇒ 给驱动器断电重启（未 save 则全部回 115200），再重试。\n");
            } else {
                printf("✗ 一轴都没响应 ⇒ %u bps 通不了。可能原因：\n", (unsigned)pc_baud);
                printf("   ① 驱动器固件不接受这个档位码（就是手册那 1~15）\n");
                printf("   ② 转换器/线材在该速率下不可靠\n");
                printf("  ⇒ 给驱动器断电重启即回 115200（未 save，波特率码没落 Flash）。\n");
                printf("  ⇒ 现在把 PC 侧改回 %u\n", (unsigned)cur);
                serial_set_baud(cur);
            }
        }
    }
}

static void cmd_accel(Robot *robot, const ParsedCmd *cmd)
{
    int set_acc = (cmd != NULL && cmd->angle_deg >= 0.0);
    int set_dec = (cmd != NULL && cmd->speed_rpm >= 0.0);
    int acc = set_acc ? (int)(cmd->angle_deg + 0.5) : 0;
    int dec = set_dec ? (int)(cmd->speed_rpm + 0.5) : 0;
    int j, ok_cnt;

    if (robot == NULL) return;

    printf("accel：驱动器加减速参数（0x0096 启动速度 / 0x0097 停止速度 / "
           "0x0098 加速 / 0x0099 减速）\n");
    printf("%5s %9s %9s %8s %8s\n", "关节", "启动rpm", "停止rpm", "加速ms", "减速ms");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        uint16_t vs = 0, vp = 0, ra = 0, rd = 0;
        if (motor_read_u16(robot, j, LEESN_REG_VEL_START, &vs) != ERR_NONE ||
            motor_read_u16(robot, j, LEESN_REG_VEL_STOP, &vp) != ERR_NONE ||
            motor_read_u16(robot, j, LEESN_REG_ACC_MS, &ra) != ERR_NONE ||
            motor_read_u16(robot, j, LEESN_REG_DEC_MS, &rd) != ERR_NONE) {
            printf("%5d %9s %9s %8s %8s\n", j, "-", "-", "-", "-");
            continue;
        }
        printf("%5d %9u %9u %8u %8u\n", j,
               (unsigned)vs, (unsigned)vp, (unsigned)ra, (unsigned)rd);
    }

    if (!set_acc && !set_dec) {
        printf("\n只读模式。改法：accel:40  或  accel:40,40\n"
               "  出厂 120/120；只写 RAM，驱动器断电即恢复（本命令不发 0x00DC）。\n"
               "  ⚠️ 调太小会丢步/过流，从 40 起试。\n");
        return;
    }

    printf("\n[写入] ");
    if (set_acc) printf("加速 0x0098 = %d ms  ", acc);
    if (set_dec) printf("减速 0x0099 = %d ms", dec);
    printf("\n");

    ok_cnt = 0;
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        uint16_t rb = 0;
        int good = 1;
        if (set_acc) {
            if (motor_write_u16(robot, j, LEESN_REG_ACC_MS, (uint16_t)acc) != ERR_NONE)
                good = 0;
            else if (motor_read_u16(robot, j, LEESN_REG_ACC_MS, &rb) != ERR_NONE ||
                     rb != (uint16_t)acc)
                good = 0;
        }
        if (set_dec) {
            if (motor_write_u16(robot, j, LEESN_REG_DEC_MS, (uint16_t)dec) != ERR_NONE)
                good = 0;
            else if (motor_read_u16(robot, j, LEESN_REG_DEC_MS, &rb) != ERR_NONE ||
                     rb != (uint16_t)dec)
                good = 0;
        }
        printf("  J%d %s\n", j, good ? "写后读回一致 ✓" : "写后读回【不一致】✗");
        if (good) ok_cnt++;
    }
    printf("六轴 %d/6 写入并读回确认。\n", ok_cnt);
    printf("下一步：用同一条线跑一次 movel（过流关：stall:N:0）对比实际耗时。\n"
           "  若调小后仍有顿 ⇒ 顿的主因不在驱动器归零，而在 PC 侧总线开销。\n");
}

static void cmd_alarm(Robot *robot, const ParsedCmd *cmd)
{
    static const char *const names[11] = {
        "正常", "电机相位过流", "供电电压过高", "供电电压过低",
        "电机A相开路", "电机B相开路", "其他报警或位置超差", "内部24V电压偏移",
        "AI电压错误", "BI电压错误", "编码器错误"
    };
    int do_clear = (cmd != NULL && cmd->rel == 1);
    int j, n_alarm = 0;

    if (robot == NULL) return;

    printf("驱动器报警（0x00A3：低4位=当前，高12位=最近三次历史）\n");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        uint16_t v = 0;
        int cur, h[3], k;
        if (motor_read_u16(robot, j, 0x00A3, &v) != ERR_NONE) {
            printf("  J%d  读取失败\n", j);
            continue;
        }
        cur = v & 0xF;
        h[0] = (v >> 4) & 0xF;
        h[1] = (v >> 8) & 0xF;
        h[2] = (v >> 12) & 0xF;
        if (cur != 0) n_alarm++;
        printf("  J%d  当前: %s", j, names[cur]);
        if (h[0] || h[1] || h[2]) {
            printf("   历史: ");
            for (k = 0; k < 3; k++) {
                if (h[k]) printf("%s%s", names[h[k]], (k < 2) ? " / " : "");
            }
        }
        printf("\n");
    }
    printf("当前有报警的轴：%d/6\n", n_alarm);

    if (!do_clear) {
        printf("清除：alarm:clear（⚠️ 先确认报警原因已排除，否则清完立刻复现）\n");
        return;
    }
    {
        int okc = 0;
        for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
            if (motor_clear_alarm(robot, j) == ERR_NONE) okc++;
        }
        printf("已向六轴下发清除报警（0x00A4=0）：%d/6 成功\n", okc);
        printf("⚠️ 清完请再跑一次 alarm 确认已归零；若立刻复现 ⇒ 原因未排除。\n");
    }
}

void cmd_bcast(Robot *robot)
{
    int32_t b32[7] = {0}, a32[7] = {0};
    uint16_t b16[7] = {0}, a16[7] = {0};
    const int32_t probe32 = 10000;
    const uint16_t probe16 = 100;
    int j, hit10 = 0, hit06 = 0, amb = 0;

    if (robot == NULL) return;

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    printf("广播帧验证：向地址 0 写速度寄存器，再逐轴读回看是否被执行\n");
    printf("  （广播会让总线上所有从站执行同一条指令，故选不动臂的速度源，测完恢复）\n\n");

    printf("[1] 功能码 10H：广播写 0x00D8(运行速度) = 100.00 rpm\n");
    for (j = 1; j <= 6; j++) {
        int32_t v = -1;
        if (motor_read_i32(robot, j, LEESN_REG_VEL_RUN, &v) != ERR_NONE) v = -1;
        b32[j] = v;
        if (v == probe32) amb++;
    }
    if (motor_write_i32_broadcast(robot, LEESN_REG_VEL_RUN, probe32) != ERR_NONE)
        printf("  [错误] 广播帧下发失败（总线错误）\n");
    Sleep(200);

    printf("  轴    写前(rpm)    写后(rpm)   执行\n");
    for (j = 1; j <= 6; j++) {
        int32_t v = -1;
        int h;
        if (motor_read_i32(robot, j, LEESN_REG_VEL_RUN, &v) != ERR_NONE) v = -1;
        a32[j] = v;
        h = (v == probe32) ? 1 : 0;
        if (h) hit10++;
        printf("  J%d   %10.2f   %10.2f    %s\n",
               j, (double)b32[j] / 100.0, (double)a32[j] / 100.0, h ? "是" : "否");
    }

    if (hit10 == 0) {
        printf("\n  10H 广播无响应，改测 06H（部分驱动器只实现单寄存器广播）\n");
        printf("[2] 功能码 06H：广播写 0x009A(连续运行速度源) = 100 rpm\n");
        for (j = 1; j <= 6; j++) {
            uint16_t v = 0;
            if (motor_read_u16(robot, j, LEESN_REG_RUN_SPEED16, &v) != ERR_NONE) v = 0;
            b16[j] = v;
        }
        if (motor_write_u16_broadcast(robot, LEESN_REG_RUN_SPEED16, probe16) != ERR_NONE)
            printf("  [错误] 广播帧下发失败（总线错误）\n");
        Sleep(200);

        printf("  轴    写前(rpm)    写后(rpm)   执行\n");
        for (j = 1; j <= 6; j++) {
            uint16_t v = 0;
            int h;
            if (motor_read_u16(robot, j, LEESN_REG_RUN_SPEED16, &v) != ERR_NONE) v = 0;
            a16[j] = v;
            h = (v == probe16) ? 1 : 0;
            if (h) hit06++;
            printf("  J%d   %10u   %10u    %s\n", j,
                   (unsigned)b16[j], (unsigned)a16[j], h ? "是" : "否");
        }
    }

    printf("\n判定：");
    if (hit10 == 6 || hit06 == 6) {
        printf("六轴全部执行广播（%s）⇒ 方案成立。\n", hit10 == 6 ? "10H" : "06H");
        printf("  每条总线只挂一个从站时，可用广播下发位置：从站不回包，\n");
        printf("  既省掉每笔约 15ms 的等响应，也不会像 noread 那样撞车。\n");
        printf("  ⇒ 6 路独立 USB-RS485 值得做，一轮六轴可从 92ms 降到约 6ms。\n");
    } else if (hit10 == 0 && hit06 == 0) {
        printf("两种功能码都无响应 ⇒ 该型号不执行广播帧。\n");
        printf("  退回 noread 方案（只写不等响应），需先测出从站响应占线时间；\n");
        printf("  或者保持现状——用 smooth 的单段下发换流畅，接受小弧。\n");
    } else {
        printf("只有 %d 轴响应 ⇒ 广播支持不稳定，不要用于下发位置。\n",
               hit10 ? hit10 : hit06);
    }
    if (amb > 0)
        printf("  注意：有 %d 轴写前值恰好已是 100rpm，这些轴无法区分，"
               "改探针值重测才准。\n", amb);

    printf("\n恢复原速度值…");
    for (j = 1; j <= 6; j++) {
        if (a32[j] != b32[j]) motor_set_speed(robot, j, (double)b32[j] / 100.0);
        if (a16[j] != b16[j]) motor_set_speed16(robot, j, (int)b16[j]);
    }
    printf("已恢复。可再跑一次 bcast 确认（写后值应等于写前值）。\n\n");

    monitor_pause_active(0);
}

void cmd_nrtest(Robot *robot)
{
    static const int delays[] = {0, 2, 3, 4, 5, 6};
    const int rounds = 20;
    const int verify = 3;
    const int reps = 3;
    int32_t base[7];
    int j, d, r, rep;

    if (robot == NULL) return;

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);
    bus_drain();

    printf("noread 连发帧完整性测试：反复下发【当前位置】⇒ 臂原地不动\n");
    printf("（下发目标 = 当前位置，所以不会动；测的是连发会不会撞车）\n");
    printf("每档：%d 轮 × 6 轴，重复 %d 遍取最坏值；连发后每轴连读 %d 遍\n\n",
           rounds, reps, verify);

    for (j = 1; j <= 6; j++) {
        int ok = 0;
        base[j] = motor_read_position(robot, j, &ok);
        if (!ok) {
            printf("[错误] 读 J%d 初始位置失败，测试中止\n", j);
            monitor_pause_active(0);
            return;
        }
    }

    printf("  帧间延迟   一轮六轴耗时    读失败    位置漂移    判定\n");
    for (d = 0; d < (int)(sizeof(delays) / sizeof(delays[0])); d++) {
        int worst_fail = 0, worst_drift = 0;
        double sum_round = 0.0;

        for (rep = 0; rep < reps; rep++) {
            int fail = 0, drift = 0;
            uint32_t t0 = GetTickCount();

            for (r = 0; r < rounds; r++) {
                for (j = 1; j <= 6; j++) {
                    motor_move_abs_noread(robot, j, base[j]);
                    if (delays[d] > 0) Sleep((DWORD)delays[d]);
                }
            }
            sum_round += (double)(GetTickCount() - t0) / (double)rounds;

            for (j = 1; j <= 6; j++) {
                int v;
                for (v = 0; v < verify; v++) {
                    int ok = 0;
                    int32_t p = motor_read_position(robot, j, &ok);
                    if (!ok) {
                        fail++;
                        break;
                    }
                    if (p < base[j] - MOVEJ_INPOS_TOL ||
                        p > base[j] + MOVEJ_INPOS_TOL) {
                        drift++;
                    }
                }
            }
            if (fail > worst_fail) worst_fail = fail;
            if (drift > worst_drift) worst_drift = drift;
        }

        printf("  %6d ms   %10.2f ms   %6d/6   %6d/6    %s\n",
               delays[d], sum_round / (double)reps, worst_fail, worst_drift,
               (worst_fail == 0 && worst_drift == 0) ? "干净" : "撞车/异常");
    }

    {
        uint32_t baud_now = serial_get_baud();
        double frame_ms = 13.0 * 10.0 /
                          (double)(baud_now ? baud_now : MODBUS_BAUDRATE) * 1000.0;
        printf("\n读法：找到【判定=干净】的【最小】延迟，那一档就是安全帧间隔。\n");
        printf("      一轮六轴耗时 ≈ 6 ×(推帧 %.2fms + 该延迟) + 从站回包占线。\n", frame_ms);
        printf("      对比基线：等响应读位置一轮【10~12ms 量级】（约 84~96Hz，见 busrate[5]）。\n");
        printf("      注意：noread 的最小干净延迟是 2ms ⇒ 一轮 6×(0.15+2)≈13ms，\n");
        printf("            已经【不比等响应快】。noread 现在只在\"必须不等到位\"时才值得用。\n");
        printf("      若连 6ms 都不干净 ⇒ 单总线 noread 走不通，只能换 6 路总线用广播。\n\n");
    }

    monitor_pause_active(0);
}
