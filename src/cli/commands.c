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
#define MOVL_ACC_FLOOR_MS   60      /* 加减速安全下限（ms） */

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
        if (motor_set_profile(robot, j, accel_ms, decel_ms) != ERR_NONE) {
            printf("[警告] 关节%d 加减速设置失败\n", j);
        }
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
                ErrCode rc = motor_estop(robot, j);
                printf("[警告] 关节%d 多关节运动超时，已急停%s\n", j,
                       (rc == ERR_NONE) ? "" : "（急停指令下发失败）");
                pend[j] = 0; remain--;
            }
            break;
        }
        printf("\rJ1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f spd=%d",
               robot_read_position_deg(robot, 1, NULL),
               robot_read_position_deg(robot, 2, NULL),
               robot_read_position_deg(robot, 3, NULL),
               robot_read_position_deg(robot, 4, NULL),
               robot_read_position_deg(robot, 5, NULL),
               robot_read_position_deg(robot, 6, NULL),
               motor_read_speed(robot, joints[0]));
        fflush(stdout);
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
        if (remain > 0) Sleep(20);
    }
    printf("\n");
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
/* 注意：LEESN驱动器仅支持关节空间绝对位置运动（motor_move_abs），
 * 驱动器内部做关节梯形profile，TCP轨迹必然为弧线。
 * MoveL 实现为：终点逆解 + movej_joints 关节空间平滑运动。 */

void cmd_movel(Robot *robot, const ParsedCmd *cmd)
{
    double end_m[4][4];
    double sols[IK_MAX_SOLUTIONS][6];
    double filtered[IK_MAX_SOLUTIONS][6];
    double q_start[6], q_end[6];
    JointLimit limits[ROBOT_JOINT_COUNT];
    int cnt, n, j;

    /* 1) 读起点关节角 */
    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        q_start[j] = robot_read_position_deg(robot, j + 1, NULL);
    }

    /* 2) 终点逆解 + 分支连续选解 */
    line_pose_to_matrix(cmd->cartesian, end_m);
    cnt = ik_solve(DH_TABLE, end_m, sols);
    if (cnt <= 0) {
        printf("[错误] MoveL 终点逆解失败（无解或超出工作空间），未下发运动\n");
        return;
    }
    const double limit_min[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double limit_max[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        limits[j].min_deg = limit_min[j];
        limits[j].max_deg = limit_max[j];
    }
    n = ik_unwrap_solutions(sols, cnt, q_start, filtered);
    n = ik_filter_by_limits(filtered, n, limits, filtered);
    if (n <= 0) {
        printf("[错误] MoveL 终点所有候选解超出关节软限位，未下发运动\n");
        return;
    }
    if (ik_select_best_continuous(filtered, n, q_start, NULL, q_end) != 0) {
        printf("[错误] MoveL 终点选解失败，未下发运动\n");
        return;
    }

    /* 3) movej_joints：关节空间平滑运动（驱动器内部梯形profile） */
    int acc = (cmd->accel_ms[0] > 0) ? cmd->accel_ms[0] : MOVL_ACC_FLOOR_MS;
    int dec = (cmd->decel_ms[0] > 0) ? cmd->decel_ms[0] : MOVL_ACC_FLOOR_MS;
    int joints[6] = {1, 2, 3, 4, 5, 6};
    movej_joints(robot, 6, joints, q_end,
                 cmd->speeds[0] > 0 ? cmd->speeds[0] : 1.0, acc, dec);
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
