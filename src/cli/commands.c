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
#include <string.h>

#include <windows.h>

/* ====================== 内部辅助 ====================== */

#define MOVEJ_POLL_MS      10
#define MOVEJ_INPOS_TOL   100
#define MOVEJ_TIMEOUT_MS  60000
/* 单轴最低转速(rpm)：movej_issue 靠"转速按行程比例分配"让六轴同起同停，
 * 行程小的轴分到的转速是个位甚至小数，这里是防止它小到驱动器执行不了。
 * 注意两点：
 *   1) 绝不能抬高——一旦某轴被下限截断，它就提前到位，最后一段只剩其余几轴在动，
 *      笛卡尔末端就是一段弧（"逼近目标点画弧"的根因）。比例分配保证下限只在
 *      "该轴行程 < 最大轴的 最低转速/指令转速"（10rpm 时即 10%）时才生效，
 *      此时提前到位的偏差以该轴自身那点行程为界，可忽略。
 *   2) 依赖 motor_set_speed 传小数：寄存器单位 0.01rpm，若形参是 int 会把
 *      0.28rpm 截成 0 ⇒ 该轴完全不动。曾因此把 0.05 写成 0 导致 J2 前两拍不动、
 *      J3 最终超时急停。 */
#define MOVEJ_MIN_RPM       1.0
#define MOVL_ACC_FLOOR_MS   60      /* 加减速安全下限（ms） */
#define MOVL_STEP_MM        1.0     /* 直线插补弦步长(mm)：越小越直、点越密（封顶 LINE_MAX_POINTS） */
#define MOVL_STREAM_MIN_MS  20      /* stream 单节拍下限(ms)：防 seg_dt 过小打爆总线；实际节拍由总线耗时与 seg_dt 取大者 */
/* stream 目标节拍(s)：一个节拍要写 6 轴速度+绝对位置(12 事务)，115200 下实测约 8ms/事务
 * ⇒ 单节拍约 100ms。这是 PC 端周期下发的物理下限，弦步长必须按它放大，
 * 否则段行程 1.4ms 就走完、剩余 95ms 干等，退化成比 step 更差的"走一步停一下"。
 * 换更快的总线或更少的轴时按实测定值调小本常量。 */
#define MOVL_STREAM_BEAT_S  0.10    /* stream 目标节拍(s)：段行程应≈一个节拍走完，见上方说明 */
#define MOVL_BOW_SAMPLES    24      /* 估算弓高的关节空间采样数（每段） */
#define MOVL_BOW_WARN_MM    1.0     /* 弓高超过此值就显式警告：说明段数被节拍压得太少 */

/* 前向声明：line_a/line_b 非空时，等待循环中顺便打印末端相对理想直线的实时偏差 */
static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                          const double angles[6], double speed,
                          int accel_ms, int decel_ms,
                          const double *line_a, const double *line_b);
static void movej_multi(Robot *robot, const ParsedCmd *cmd);

/* movl_point_dev_mm：点 x 到直线段 AB 的距离(mm)，用于实测轨迹偏差读数 */
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
        cmd_movel(robot, cmd);
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

/* movej_issue：按关节行程比例分配速度并下发绝对位置，【不等待到位】。
 * ref_angles 为行程参考点：
 *   NULL    -> 读各轴当前实际位置算行程（逐段到位模式，多 6 次总线读，最准）；
 *   非 NULL -> 用上一插补点角度算行程（stream 模式，省掉读位置，压单节拍事务数）。
 * set_profile=0 时跳过加减速写入（stream 只在首段写一次）。
 * 输出 tgt/pend（各轴目标脉冲与待到位标记），返回待到位轴数。 */
static int movej_issue(Robot *robot, int num_joints, const int joints[6],
                       const double angles[6], const double *ref_angles,
                       double speed, int accel_ms, int decel_ms, int set_profile,
                       int32_t tgt[7], uint8_t pend[7])
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    double dist[7] = {0};
    double max_dist = 0;
    int i, j, remain = 0;

    /* 速度必须为正：speed<=0 时按距离比例分配会得到非法转速，直接拒绝执行
     * （movel 与多关节 movej 均经本函数下发，统一在此兜底校验） */
    if (speed <= 0.0) {
        printf("[警告] 多关节运动速度须大于 0 rpm，已拒绝执行\n");
        return 0;
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
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        double s;
        if (max_dist > 0) {
            s = dist[j] / max_dist * speed;
            if (s < MOVEJ_MIN_RPM) s = MOVEJ_MIN_RPM;   /* 见 MOVEJ_MIN_RPM 说明，勿抬高 */
        } else {
            s = speed;
        }
        if (set_profile && motor_set_profile(robot, j, accel_ms, decel_ms) != ERR_NONE) {
            printf("[警告] 关节%d 加减速设置失败\n", j);
        }
        if (motor_set_speed(robot, j, s) != ERR_NONE ||
            motor_move_abs(robot, j, tgt[j]) != ERR_NONE) {
            printf("[警告] 关节%d 多关节运动发指令失败\n", j);
            continue;
        }
        pend[j] = 1;
        remain++;
    }
    return remain;
}

/* movej_wait：轮询等待 pend 中轴到位（位置进容差即算到位），超时急停。
 * line_a/line_b 非空时额外打印末端相对理想直线的实时偏差(mm)——轮询本来就要读
 * 六轴角度，故该读数不增加任何总线开销。 */
static void movej_wait(Robot *robot, const int joints[6], const int32_t tgt[7],
                       uint8_t pend[7], int remain,
                       const double *line_a, const double *line_b)
{
    uint32_t start_ms = GetTickCount();
    double worst_dev = 0.0;
    int j;

    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= MOVEJ_TIMEOUT_MS) {
            const uint16_t red_to[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
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
        double qnow[6];
        for (j = 0; j < 6; j++) qnow[j] = robot_read_position_deg(robot, j + 1, NULL);
        printf("\rJ1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f spd=%d",
               qnow[0], qnow[1], qnow[2], qnow[3], qnow[4], qnow[5],
               motor_read_speed(robot, joints[0]));
        if (line_a != NULL && line_b != NULL) {
            double m[4][4], xyz[3], dev;
            dh_forward(DH_TABLE, qnow, m);
            for (j = 0; j < 3; j++) xyz[j] = m[j][3];
            dev = movl_point_dev_mm(xyz, line_a, line_b);
            if (dev > worst_dev) worst_dev = dev;   /* 实时值被 \r 覆盖，峰值要单独存 */
            printf("  偏差%6.2fmm", dev);
        }
        fflush(stdout);
        for (j = 1; j <= 6; j++) {
            if (!pend[j]) continue;
            int pos_ok = 0;
            int32_t pos = motor_read_position(robot, j, &pos_ok);
            if (pos_ok && pos >= tgt[j] - MOVEJ_INPOS_TOL &&
                pos <= tgt[j] + MOVEJ_INPOS_TOL) {
                pend[j] = 0; remain--;
            }
        }
        if (remain > 0) Sleep(20);
    }
    printf("\n");
    if (line_a != NULL && line_b != NULL)
        printf("本段实测最大偏差 %.2f mm（实时值比到位判定早约一个轮询周期，末屏不是终点残差）\n",
               worst_dev);
}

/* movej_joints：多关节同时运动完整流程 = 下发(读实际位置) + 等待同步到位 */
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
                 cmd->speeds[0], cmd->accel_ms[0], cmd->decel_ms[0], NULL, NULL);
}

/* movl_plan：按给定弦步长做直线离散 + 逐点 IK + 分段时间表（可被 stream 重规划复用） */
static int movl_plan(const double start_pose[6], const double end_pose[6],
                     const double q_start[6], const JointLimit *limits,
                     double dist_mm, double step_mm, const double vmax[6],
                     double q_seq[LINE_MAX_POINTS][6], double seg_dt[LINE_MAX_SEGS],
                     int *out_count, double *out_total_dt, int *out_fail_idx)
{
    LinePath path;
    int count = line_count_for_distance(dist_mm, step_mm);

    if (line_plan(start_pose, end_pose, count, &path) != 0) return -1;
    if (line_solve(&path, DH_TABLE, limits, q_start, q_seq, out_fail_idx) != 0) return -2;
    if (line_time_table(q_seq, count, vmax, seg_dt, out_total_dt) != 0) return -3;
    *out_count = count;
    return 0;
}

/* movl_bow_mm：单发同步模式下，关节空间线性插补相对笛卡尔直线 AB 的最大偏差(弓高, mm)。
 * 驱动器只对终点做插补，中间走的是关节空间直线，故偏离规划直线，偏离量随行程平方增长。 */
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

/* ====================== moveL：笛卡尔真直线插补 ======================
 * 实现：起点FK → line_plan(位置线性+姿态SLERP) → line_solve(逐点IK+分支连续)
 *       → line_time_table(按关节限速生成等间隔分段时间表) → 逐段 movej_joints 下发。
 * 末端走空间直线（密集弦逼近），而非单终点 IK 的弧线。
 * P2 电流自适应限速：实时电流逼近 ROBOT_STALL_CURRENT_MA 阈值则按比例降速，
 *       超阈值则急停全部关节（过流=降速/限流，非加力）；阈值=0 时关闭（默认）。
 * step 模式：逐段下发并等待六轴到位后再发下一段，段间有停顿但绝不过冲（真直线）。
 * stream 模式：按分段时间表【周期刷新】——段间不等待到位，加减速只在首段写一次，
 *       每个节拍只写速度+绝对位置（6轴×2事务，省去读位置与重复写 profile），
 *       末端走完最后一段后才统一等待真正到位。轨迹连续无段间停顿，
 *       代价是轨迹整体滞后于规划时刻（轴未到位即被下一目标覆盖），
 *       故 stream 依赖节拍 ≥ 总线耗时，开启 P2 电流轮询会拉长每节拍、需放宽节拍。
 * sync 模式（默认）：仍用密集弦做整条路径的可达性/限位校验，但【只下发终点】一次，
 *       六轴按行程比例分配转速同起同停，全程一次加减速，无段间停顿。
 *       代价是段内走关节空间直线 ⇒ 偏离笛卡尔直线（弓高），已按行程估算并打印。
 *       P2 电流自适应限速依赖分段，sync 下不生效。 */

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
    double total_dt = 0.0;
    double base_rpm = (cmd->speeds[0] > 0) ? cmd->speeds[0] : 60.0;
    double speed_rpm = base_rpm;
    int acc = (cmd->accel_ms[0] > 0) ? cmd->accel_ms[0] : MOVL_ACC_FLOOR_MS;
    int dec = (cmd->decel_ms[0] > 0) ? cmd->decel_ms[0] : MOVL_ACC_FLOOR_MS;
    double stall = ROBOT_STALL_CURRENT_MA;   /* 0 = 关闭 */

    /* 1) 起点关节角 + FK 求起点笛卡尔位姿 */
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

    /* 2) 按位移定插补点数 → 直线离散 + 逐点 IK + 分段时间表 */
    double dx = end_pose[0] - start_pose[0];
    double dy = end_pose[1] - start_pose[1];
    double dz = end_pose[2] - start_pose[2];
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    {
        const double lmin[6] = ROBOT_JOINT_LIMIT_MIN_DEG;
        const double lmax[6] = ROBOT_JOINT_LIMIT_MAX_DEG;
        for (j = 0; j < 6; j++) {
            limits[j].min_deg = lmin[j];
            limits[j].max_deg = lmax[j];
        }
    }
    /* 速度口径：指令 rpm 与 MoveJ 一致，是【电机轴】rpm（0x00D8 / 0x009A 均为电机轴）。
     * 而 line_time_table 的 vmax 是【机械角】deg/s，故必须除以减速比。
     * 曾漏除 ⇒ 时间表比真实快 50 倍 ⇒ stream 段数被算成 1 ⇒ 目标在几毫秒内全砸下去、
     * 臂其实要走 5 秒 ⇒ 退化成单关节空间直插 ⇒ 末端大弧（84mm 实测 7.9mm）。 */
    {
        const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
        for (j = 0; j < 6; j++) vmax[j] = speed_rpm * 6.0 / (double)red[j];
    }

    double step_mm = MOVL_STEP_MM;
    if (movl_plan(start_pose, end_pose, q_start, limits, dist, step_mm, vmax,
                  q_seq, seg_dt, &count, &total_dt, &fail_idx) != 0) {
        printf("[错误] MoveL 第 %d 个插补点逆解失败/越软限位，未下发\n", fail_idx);
        return;
    }

    /* stream：把弦步长放大到"一段≈一个节拍"。
     * 固定 1mm 在 60rpm 下每段仅约 1.4ms，而一个下发节拍要 ~100ms，
     * 轴会瞬间走完再干等，反而比 step 更抖。段数 = 总时长 / 目标节拍。 */
    if (cmd->movl_mode == MOVL_MODE_STREAM && count > 2) {
        int n_seg = (int)ceil(total_dt / MOVL_STREAM_BEAT_S);
        if (n_seg < 1) n_seg = 1;
        if (n_seg < count - 1) {
            step_mm = dist / (double)n_seg;
            if (movl_plan(start_pose, end_pose, q_start, limits, dist, step_mm, vmax,
                          q_seq, seg_dt, &count, &total_dt, &fail_idx) != 0) {
                printf("[错误] MoveL 第 %d 个插补点逆解失败/越软限位，未下发\n", fail_idx);
                return;
            }
        }
    }

    /* sync（默认）：只下发终点，六轴按行程分配转速同起同停，全程一次加减速 */
    if (cmd->movl_mode == MOVL_MODE_SYNC) {
        const double *q_end = q_seq[count - 1];
        double dmax = 0.0, sync_rpm;
        {
            const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
            for (j = 0; j < 6; j++) {
                double d = fabs(q_end[j] - q_start[j]) * (double)red[j];   /* 电机角度 */
                if (d > dmax) dmax = d;
            }
        }
        /* 按笛卡尔规划时长反推转速：保证实际速度不快于指令速度 */
        sync_rpm = (total_dt > 1e-6) ? (dmax / total_dt / 6.0) : base_rpm;
        if (sync_rpm > base_rpm) sync_rpm = base_rpm;
        if (sync_rpm < 1.0) sync_rpm = 1.0;
        printf("MoveL: 单发同步, 位移 %.1f mm, 速度 %.1f rpm, 预计 %.2f s, "
               "直线偏差 ≤ %.2f mm（P2 阈值 %d mA %s）\n",
               dist, sync_rpm, total_dt,
               movl_bow_mm(q_start, q_end, start_pose, end_pose),
               (int)stall, (stall > 0.0) ? "开启" : "关闭");
        movej_joints(robot, 6, joints, q_end, sync_rpm, acc, dec, start_pose, end_pose);
        return;
    }

    /* 段内走关节空间插补 ⇒ 偏离笛卡尔直线，把弓高上界一并打出来。
     * 段数被总线节拍压到 1 时（速度过快、总时长 < 一个节拍）弓高会很大，
     * 必须让用户看见，而不是跑完才发现画弧。 */
    {
        double bow = 0.0;
        for (i = 1; i < count; i++) {
            double d = movl_bow_mm(q_seq[i - 1], q_seq[i], start_pose, end_pose);
            if (d > bow) bow = d;
        }
        printf("MoveL: %d 段, 步长 %.1f mm, 位移 %.1f mm, 节拍 %.3f s, 预计 %.2f s, "
               "模式 %s, 直线偏差 ≤ %.2f mm（P2 阈值 %d mA %s）",
               count - 1, step_mm, dist, (count > 1) ? seg_dt[0] : 0.0, total_dt,
               (cmd->movl_mode == MOVL_MODE_STREAM) ? "stream" : "step", bow,
               (int)stall, (stall > 0.0) ? "开启" : "关闭");
        if (bow > MOVL_BOW_WARN_MM)
            printf("\n[警告] 弓高 %.1f mm 偏大：段数被总线节拍压到 %d 段，建议降速"
                   "（或改用 sync 并接受弧线）", bow, count - 1);
        /* 段时长若短于加减速时间之和，驱动器每段都在重新爬坡、达不到指令转速，
         * 实际耗时会显著长于"预计"（预计按匀速算）。实测：段 99ms / 加减速 230ms → 慢 2.9 倍 */
        if (count > 1 && seg_dt[0] * 1000.0 < (double)(acc + dec))
            printf("\n[提示] 段时长 %.0f ms 短于加减速时间之和 %d ms，驱动器每段都在重新爬坡，"
                   "实际耗时会明显长于预计；想提速请调小 ACC/DEC 或放慢速度",
                   seg_dt[0] * 1000.0, acc + dec);
        printf("\n");
    }

    /* 3) 逐段下发（密集弦逼近直线），段间做 P2 电流自适应限速 */
    int32_t s_tgt[7] = {0};
    uint8_t s_pend[7] = {0};
    int s_remain = 0;
    uint32_t mv_t0 = GetTickCount();
    for (i = 1; i < count; i++) {
        if (stall > 0.0) {
            int max_cur = 0;
            for (j = 0; j < 6; j++) {
                int cur = robot_read_current_ma(robot, j + 1);
                if (cur > 0 && cur > max_cur) max_cur = cur;
            }
            if (max_cur >= (int)stall) {
                printf("[过流保护] 实时电流 %d mA 超阈值 %d mA，急停全部关节\n",
                       max_cur, (int)stall);
                for (j = 0; j < 6; j++)
                    if (!robot_is_masked(robot, j + 1)) motor_estop(robot, j + 1);
                return;
            } else if (max_cur > (int)(stall * 0.6)) {
                double factor = (double)stall / max_cur * 0.8;   /* 留 20% 余量 */
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

        /* 本段速度：取对时间最紧的关节所需【电机轴】rpm
         * （movej_issue 按电机轴行程比例分配，整段同时间到达） */
        double seg_speed = 0.0;
        {
            const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
            for (j = 0; j < 6; j++) {
                double d = fabs(q_seq[i][j] - q_seq[i - 1][j]) * (double)red[j];
                double rpm = d / seg_dt[i - 1] / 6.0;
                if (rpm > seg_speed) seg_speed = rpm;
            }
        }
        if (seg_speed < 1.0) seg_speed = 1.0;

        if (cmd->movl_mode == MOVL_MODE_STREAM) {
            /* 周期刷新：只下发不等待；加减速仅首段写一次，行程参考取上一插补点（省读位置） */
            uint32_t t0 = GetTickCount();
            int r = movej_issue(robot, 6, joints, q_seq[i], q_seq[i - 1],
                                seg_speed, acc, dec, (i == 1) ? 1 : 0, s_tgt, s_pend);
            if (r > 0) s_remain = r;
            uint32_t period_ms = (uint32_t)(seg_dt[i - 1] * 1000.0);
            if (period_ms < MOVL_STREAM_MIN_MS) period_ms = MOVL_STREAM_MIN_MS;
            uint32_t used = GetTickCount() - t0;
            if (used < period_ms) Sleep(period_ms - used);
            if (i % 20 == 0) {
                printf("\rMoveL stream %d/%d 段", i, count - 1);
                fflush(stdout);
            }
        } else {
            movej_joints(robot, 6, joints, q_seq[i], seg_speed, acc, dec,
                         start_pose, end_pose);
        }
    }

    /* stream：全部段下发完后才统一等待真正到位，避免命令返回时臂仍在运动 */
    if (cmd->movl_mode == MOVL_MODE_STREAM) {
        if (s_remain > 0) movej_wait(robot, joints, s_tgt, s_pend, s_remain,
                                     start_pose, end_pose);
        printf("MoveL stream %d/%d 段完成，实际耗时 %.2f s（预计 %.2f s）\n",
               count - 1, count - 1, (GetTickCount() - mv_t0) / 1000.0, total_dt);
    }
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
