/*
 * commands.c —— CLI 命令实现与分发
 * ------------------------------------------------------------
 * 所属模块：应用命令层（cli）
 * 对外接口：cmd_dispatch
 * 支持命令：home、MoveJ、MoveL、disable、enable、motor、getpos、zero、
 *   zero_save、help、exit
 * main.c 仅负责初始化与命令循环。
 */

#include "cli/commands.h"
#include "control/robot.h"
#include "control/home.h"
#include "control/monitor.h"
#include "api/motor_reg.h"
#include "utils/err.h"
#include "utils/ini_rw.h"
#include "kinematics/joint_zero.h"
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"
#include "config/robot_config.h"

#include <stdio.h>
#include <math.h>

#include <windows.h>

/* ====================== 内部辅助 ====================== */

#define MOVEJ_POLL_MS      50
#define MOVEJ_INPOS_TOL   100
#define MOVEJ_TIMEOUT_MS  60000

/* 前向声明 */
static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                          const double angles[6], double speed,
                          int accel_ms, int decel_ms);
static void movej_multi(Robot *robot, const ParsedCmd *cmd);

/* ====================== 电机实时监控线程 ====================== */

typedef struct {
    Robot *robot;
    volatile LONG running;
    HANDLE thread;
} MotorMonitor;

static MotorMonitor *g_motor_mon = NULL;

/* motor_monitor_thread：后台线程主循环，每1秒打印一次全部电机信息 */
static DWORD WINAPI motor_monitor_thread(LPVOID arg)
{
    MotorMonitor *mm = (MotorMonitor *)arg;

    /* 使用 InterlockedCompareExchange 原子读 running，与写入侧
     * InterlockedExchange 配对，避免非 x86 平台上的撕裂读。 */
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

        /* 等待1秒，期间每500ms检查一次 running 是否被置0 */
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
    InterlockedExchange(&mm->running, 1);   /* 与线程内原子读配对 */
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

/* cmd_motor_running：查询电机监控线程是否在运行 */
int cmd_motor_running(void)
{
    return motor_monitor_is_running(g_motor_mon);
}

/* ====================== 命令分发 ====================== */

/* cmd_dispatch：命令分发总入口
 * 返回 1 表示用户请求退出（exit/quit），否则 0。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd)
{
    int j;
    switch (cmd->type) {
    case CMD_HOME: {
        motor_monitor_stop(g_motor_mon);
        monitor_stop(mon);
        ErrCode rc = (cmd->joint >= 1) ? robot_home_single(robot, cmd->joint)
                                       : robot_home(robot);
        if (rc != ERR_NONE) printf("[错误] 回零失败：%s\n", err_str(rc));
        if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
            printf("[警告] 回零后监控线程重启失败\n");
        break;
    }
    case CMD_MOVEJ: {
        if (cmd->num_joints > 1) {
            movej_multi(robot, cmd);
        } else {
            double target = cmd->angle_deg;
            if (cmd->rel) {
                int ok = 0;
                double cur = robot_read_position_deg(robot, cmd->joint, &ok);
                if (!ok) {
                    printf("[错误] 关节%d 读取当前位置失败，相对运动无法执行\n", cmd->joint);
                    break;
                }
                target = cur + cmd->angle_deg;   /* 相对当前实际位置 */
            }
            ErrCode rc = robot_movej(robot, cmd->joint, target, cmd->speed_rpm);
            if (rc != ERR_NONE) printf("[错误] 运动指令失败：%s\n", err_str(rc));
        }
        break;
    }
    case CMD_MOVEL: {
        cmd_movl(robot, cmd);
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
        /* 只有目标轴全部成功才提示成功，避免"最后一轴成功"掩盖前面轴的失败 */
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

/* cmd_zero：显示零点标定数据和当前机械角 */
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

/* cmd_zero_save：保存指定的零点标定值，内存 + ini 持久化 */
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

/* movej_joints：多关节同时运动核心逻辑 — 按距离比例分配速度，确保同步到位 */
static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                         const double angles[6], double speed,
                         int accel_ms, int decel_ms)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    uint8_t pend[7] = {0};
    int32_t tgt[7] = {0};
    int remain = 0;
    int j;
    uint32_t timeout_ms = MOVEJ_TIMEOUT_MS;

    const double *zero = joint_zero_get();
    double max_dist = 0;
    int32_t pos_buf[7] = {0};
    int32_t tgt_buf[7] = {0};

    /* 速度必须为正：speed<=0 时按距离比例分配会得到非法转速，直接拒绝执行
     * （movel 与多关节 movej 均经本函数下发，统一在此兜底校验） */
    if (speed <= 0.0) {
        printf("[警告] 多关节运动速度须大于 0 rpm，已拒绝执行\n");
        return;
    }

    for (int i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        double motor_deg = angles[i] + zero[j - 1];
        tgt_buf[j] = DEG2STEPS(motor_deg, reductions[j - 1]);
        int pos_ok = 0;
        pos_buf[j] = motor_read_position(robot, j, &pos_ok);
        int32_t dist = pos_ok ? abs(tgt_buf[j] - pos_buf[j]) : 0;
        if (dist > max_dist) max_dist = dist;
    }

    for (int i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        double s;
        if (max_dist > 0) {
            int32_t dist = abs(tgt_buf[j] - pos_buf[j]);
            s = (double)dist / max_dist * speed;
            if (s < 5) s = 5;
        } else {
            s = speed;
        }
        motor_set_profile(robot, j, accel_ms, decel_ms);
        if (motor_set_speed(robot, j, s) != ERR_NONE ||
            motor_move_abs(robot, j, tgt_buf[j]) != ERR_NONE) {
            printf("[警告] 关节%d 多关节运动发指令失败\n", j);
            continue;
        }
        tgt[j] = tgt_buf[j];
        pend[j] = 1;
        remain++;
    }
    if (remain == 0) return;

    uint32_t start_ms = GetTickCount();
    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= timeout_ms) {
            for (j = 1; j <= 6; j++) {
                if (!pend[j]) continue;
                /* 超时仍未到位的轴可能还在朝目标运动，必须显式急停后再退出 */
                ErrCode rc = motor_estop(robot, j);
                printf("[警告] 关节%d 多关节运动超时，已急停%s\n", j,
                       (rc == ERR_NONE) ? "" : "（急停指令下发失败）");
                pend[j] = 0; remain--;
            }
            break;
        }
        for (j = 1; j <= 6; j++) {
            if (!pend[j]) continue;
            int32_t pos;
            int pos_ok;
            pos = motor_read_position(robot, j, &pos_ok);
            if (pos_ok && pos >= tgt[j] - MOVEJ_INPOS_TOL &&
                pos <= tgt[j] + MOVEJ_INPOS_TOL) {
                pend[j] = 0; remain--;
            }
        }
        if (remain > 0) Sleep(MOVEJ_POLL_MS);
    }
}

/* movej_multi：多关节同时运动（从 ParsedCmd 调用） */
static void movej_multi(Robot *robot, const ParsedCmd *cmd)
{
    int joints[6], idx = 0;
    double angles[6];
    for (int i = 0; i < cmd->num_joints; i++) {
        joints[idx] = cmd->joints[i];
        angles[idx] = cmd->angles[i];
        idx++;
    }
    movej_joints(robot, cmd->num_joints, joints, angles,
                 cmd->speeds[0], cmd->accel_ms[0], cmd->decel_ms[0]);
}

/* ====================== moveL：笛卡尔直线运动 ====================== */

#define MOVL_STEP_MM        2.0     /* 直线插补步长（mm）— 小步长保证IK分支连续 */
#define MOVL_INPOS_TOL      10      /* 到位判定容差（脉冲），步长越小容差越严 */
#define MOVL_INPOS_TIMEOUT_MS 10000 /* 单段/整段到位等待上限（ms） */

/* moveL 工作缓冲：257×6 双精度约 12KB，放静态区，避免占用线程栈 */
static LinePath g_movl_path;
static double   g_movl_q[LINE_MAX_POINTS][6];

/* movl_seg_speed：由相邻两点的关节位移与段时长换算各关节电机转速（rpm）
 * 轴上角速度(deg/s) = 电机转速(rpm) * 6 / 减速比 */
static void movl_seg_speed(const double q0[6], const double q1[6], double seg_dt,
                           double v_rpm[6])
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    int j;

    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        double v_deg_s = (seg_dt > 1e-9) ? fabs(q1[j] - q0[j]) / seg_dt : 0.0;
        v_rpm[j] = v_deg_s * (double)reductions[j] / 6.0;
    }
}

/* movl_send_point：把单个插补点的各关节目标角按各自转速下发（不等待到位）。
 * set_profile=1 时同时下发加减速时间。返回成功下发的轴数。 */
static int movl_send_point(Robot *robot, const double q[6], const double v_rpm[6],
                           int accel_ms, int decel_ms, int set_profile)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    int sent = 0;
    int j;

    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        double motor_deg, rpm;

        if (robot_is_masked(robot, j)) continue;

        motor_deg = q[j - 1] + zero[j - 1];
        rpm = v_rpm ? v_rpm[j - 1] : 0.0;
        if (rpm < 1.0) rpm = 1.0;   /* 驱动器速度须为正，微位移段按最低转速下发 */

        if (set_profile) motor_set_profile(robot, j, accel_ms, decel_ms);
        if (motor_set_speed(robot, j, rpm) != ERR_NONE) continue;
        if (motor_move_abs(robot, j, DEG2STEPS(motor_deg, reductions[j - 1])) != ERR_NONE) continue;
        sent++;
    }
    return sent;
}

/* movl_wait_point：轮询等待各关节到达目标角（度）；超时对未到位轴急停。
 * 直线运动是"边动边刷新目标"，必须用位置比对判到位，不能只看到位标志位。 */
static int movl_wait_point(Robot *robot, const double q[6], uint32_t timeout_ms)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    int32_t tgt[ROBOT_JOINT_COUNT + 1] = {0};
    uint8_t pend[ROBOT_JOINT_COUNT + 1] = {0};
    uint32_t start_ms = GetTickCount();
    int remain = 0;
    int j;

    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        if (robot_is_masked(robot, j)) continue;
        tgt[j] = DEG2STEPS(q[j - 1] + zero[j - 1], reductions[j - 1]);
        pend[j] = 1;
        remain++;
    }
    if (remain == 0) return 0;

    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= timeout_ms) {
            for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
                if (!pend[j]) continue;
                ErrCode rc = motor_estop(robot, j);
                printf("[警告] 关节%d 直线运动到位等待超时，已急停%s\n", j,
                       (rc == ERR_NONE) ? "" : "（急停指令下发失败）");
                pend[j] = 0;
                remain--;
            }
            return -1;
        }
        for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
            int pos_ok = 0;
            int32_t pos;

            if (!pend[j]) continue;
            pos = motor_read_position(robot, j, &pos_ok);
            if (pos_ok && pos >= tgt[j] - MOVL_INPOS_TOL && pos <= tgt[j] + MOVL_INPOS_TOL) {
                pend[j] = 0;
                remain--;
            }
        }
        if (remain > 0) Sleep(MOVEJ_POLL_MS);
    }
    return 0;
}

/* cmd_movl：笛卡尔直线运动（X,Y,Z in mm; Rx,Ry,Rz in deg）
 * 流程：读当前位姿 -> 按步长离散直线段 -> 逐点逆解（分支连续选解）
 *       -> 按关节限速生成分段时间表 -> 下发（逐段到位 / 周期刷新）。
 * 逆解失败时在动作之前报错返回，不会出现"走一半卡住"。 */
void cmd_movl(Robot *robot, const ParsedCmd *cmd)
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    const double limit_min[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double limit_max[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
    JointLimit limits[ROBOT_JOINT_COUNT];
    double q_start[6];
    double start_pose[6], end_pose[6];
    double seg_dt[LINE_MAX_SEGS];
    double vmax[6], v_rpm[6];
    double dt_total = 0.0, dist = 0.0, cart_speed = 0.0;
    double pose[4][4], xyz[3], rpy[3];
    int ok, count, fail_idx = -1, stream, sent;
    int i, j;

    /* 1) 读起点关节角，经正运动学换算起点位姿（直线插补必须以真实当前位姿为起点） */
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        q_start[i] = robot_read_position_deg(robot, i + 1, &ok);
        if (!ok) {
            printf("[错误] 关节%d 当前位置读取失败，MoveL 中止\n", i + 1);
            return;
        }
    }
    dh_forward(DH_TABLE, q_start, pose);
    dh_pose_to_xyz_rpy(pose, xyz, rpy);
    for (i = 0; i < 3; i++) {
        start_pose[i] = xyz[i];
        start_pose[i + 3] = rpy[i] * RAD2DEG;
    }
    for (i = 0; i < 6; i++) end_pose[i] = cmd->cartesian[i];

    for (i = 0; i < 3; i++) {
        double d = end_pose[i] - start_pose[i];
        dist += d * d;
    }
    dist = sqrt(dist);
    if (dist < 0.01) {
        printf("[警告] 起终点重合（直线距离 %.3f mm），未下发运动\n", dist);
        return;
    }

    /* 2) 直线离散：位置线性 + 姿态线性过渡 */
    count = line_count_for_distance(dist, MOVL_STEP_MM);
    if (line_plan(start_pose, end_pose, count, &g_movl_path) != 0) {
        printf("[错误] 直线插补失败（起点/终点位姿非法）\n");
        return;
    }

    /* 3) 逐点逆解 + 分支连续选解：全部算完再下发 */
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        limits[i].min_deg = limit_min[i];
        limits[i].max_deg = limit_max[i];
    }
    if (line_solve(&g_movl_path, DH_TABLE, limits, q_start, g_movl_q, &fail_idx) != 0) {
        printf("[错误] MoveL 第 %d/%d 个插补点逆解失败（无解或候选解全部超出关节软限位），未下发任何运动\n",
               fail_idx + 1, g_movl_path.count);
        return;
    }

    /* 4) 按关节限速生成分段时间表：SPD(rpm) -> 各关节角速度上限(deg/s) */
    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        vmax[j] = cmd->speeds[0] * 6.0 / (double)reductions[j];
    }
    if (line_time_table(g_movl_q, g_movl_path.count, vmax, seg_dt, &dt_total) != 0) {
        printf("[错误] 分段时间表生成失败（速度须大于 0）\n");
        return;
    }
    cart_speed = (dt_total > 1e-9) ? dist / dt_total : 0.0;

    stream = cmd->stream ? 1 : 0;
    printf("MoveL %s：%d 个插补点，路径长 %.2f mm，步长 %.2f mm，预计 %.2f s（末端 %.2f mm/s）\n",
       stream ? "周期刷新" : "逐段到位", g_movl_path.count, dist,
       dist / (double)(g_movl_path.count - 1), dt_total, cart_speed);

    /* 5) 下发 */
    {
        uint32_t t_start = GetTickCount();

        if (!stream) {
            /* 逐段到位：每段按分段时间表转速下发，等待该点到位后再走下一段 */
            for (i = 1; i < g_movl_path.count; i++) {
                movl_seg_speed(g_movl_q[i - 1], g_movl_q[i], seg_dt[i - 1], v_rpm);
                sent = movl_send_point(robot, g_movl_q[i], v_rpm,
                                       cmd->accel_ms[0], cmd->decel_ms[0], 1);
                if (sent == 0) {
                    printf("[错误] 第 %d 段下发失败，movl 中止\n", i);
                    return;
                }
                if (movl_wait_point(robot, g_movl_q[i], MOVL_INPOS_TIMEOUT_MS) != 0) {
                    printf("[错误] 第 %d/%d 段未在 %d ms 内到位，MoveL 中止\n",
                           i, g_movl_path.count - 1, MOVL_INPOS_TIMEOUT_MS);
                    return;
                }
            }
        } else {
            /* 周期刷新：按段时长节拍连续刷新目标点（不逐点等待），最后统一判到位 */
            for (i = 1; i < g_movl_path.count; i++) {
                movl_seg_speed(g_movl_q[i - 1], g_movl_q[i], seg_dt[i - 1], v_rpm);
                sent = movl_send_point(robot, g_movl_q[i], v_rpm,
                                       cmd->accel_ms[0], cmd->decel_ms[0], (i == 1));
                if (sent == 0) {
                    printf("[错误] 第 %d 段下发失败，movl 中止\n", i);
                    return;
                }
                Sleep((DWORD)(seg_dt[i - 1] * 1000.0 + 0.5));
            }
            if (movl_wait_point(robot, g_movl_q[g_movl_path.count - 1],
                                MOVL_INPOS_TIMEOUT_MS) != 0) {
                printf("[错误] movl 终点未到位，已在超时后急停\n");
                return;
            }
        }

        printf("MoveL 完成：用时 %.2f s，末端目标 X=%.2f Y=%.2f Z=%.2f\n",
               (double)(GetTickCount() - t_start) / 1000.0,
               cmd->cartesian[0], cmd->cartesian[1], cmd->cartesian[2]);
    }

    printf("终点关节角：");
    for (i = 0; i < 6; i++) {
        printf(i ? ", J%d=%.2f°" : "J%d=%.2f°", i + 1, g_movl_q[g_movl_path.count - 1][i]);
    }
    printf("\n");
}

/* cmd_getpos：读取当前关节机械角，经正运动学求末端笛卡尔坐标与姿态 */
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

    if (!all_ok) printf("[警告] 部分关节读取失败，坐标按读取值计算\n");
}
