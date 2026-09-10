/*
 * commands.c —— CLI 命令实现与分发（原 main.c 中的命令逻辑整体迁入）
 * ------------------------------------------------------------
 * 所属模块：应用命令层（cli）
 * 对外接口：cmd_dispatch
 * 依赖模块：control/robot、control/home、control/monitor、api/motor_reg、utils
 *
 * 说明：main.c 只保留"初始化 + 命令循环"，所有具体命令（home / movej /
 * enable / disable / status / scan / diag / torque / mask / unmask / calib /
 * help）的实现都在本文件，便于单独维护，不再堆在 main.c。
 */

#include "cli/commands.h"
#include "control/robot.h"
#include "control/robot_internal.h"
#include "control/home.h"
#include "control/monitor.h"
#include "api/motor_reg.h"
#include "config/robot_config.h"
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "kinematics/mat3.h"
#include "utils/err.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* cmd_status：持续刷新全部关节状态，按任意键退出；每行前缀实时 YYYY-MM-DD HH:MM:SS */
static int cmd_status(Robot *robot)
{
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    DWORD old_mode = 0;

    /* 设为非阻塞输入模式 */
    GetConsoleMode(hStdin, &old_mode);
    SetConsoleMode(hStdin, old_mode & ~(ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT));
    /* 丢弃键入 "status" 命令时遗留的回车等输入事件：
     * 否则 WaitForSingleObject 会立即返回，循环只跑一帧就当成"按任意键退出"。 */
    FlushConsoleInputBuffer(hStdin);

    while (1) {
        int j;
        static const int reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;

        /* 每行前缀实时时钟（用户明确"时间"为当前实时时间，非字面值）。
         * 每轮取一次，同一轮 6 轴共用同一时间戳；含年月日。 */
        char ts[32];
        {
            SYSTEMTIME lt;
            GetLocalTime(&lt);
            snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
                     lt.wYear, lt.wMonth, lt.wDay,
                     lt.wHour, lt.wMinute, lt.wSecond);
        }

        for (j = 1; j <= 6; j++) {
            uint32_t st32 = 0;
            int ok = 0;
            int32_t pos;
            int cur, spd, alm;
            ErrCode rc;

            if (robot_is_masked(robot, j)) {
                printf("%s 关节:%d,状态:已屏蔽\n", ts, j);
                continue;
            }
            rc = robot_read_status32(robot, j, &st32);
            if (rc != ERR_NONE) {
                printf("%s 关节:%d,状态:离线\n", ts, j);
                continue;
            }
            pos = robot_read_position_steps(robot, j, &ok);
            cur = robot_read_current_ma(robot, j);
            spd = robot_read_speed_rpm(robot, j);
            alm = robot_read_alarm(robot, j);

            {
                char flags[64] = "";
                int in0 = (st32 & LEESN_STAT_INPUT(0)) ? 1 : 0;
                int in1 = (st32 & LEESN_STAT_INPUT(1)) ? 1 : 0;
                double angle = ok ? STEPS2DEG(pos, reductions[j - 1]) : 0.0;

                if (st32 & LEESN_STAT_INPOS)      strcat(flags, "到位 ");
                if (st32 & LEESN_STAT_SOFT_NEG)   strcat(flags, "负限位 ");
                if (st32 & LEESN_STAT_SOFT_POS)   strcat(flags, "正限位 ");
                if (st32 & LEESN_STAT_HOMED)      strcat(flags, "原点 ");
                if (st32 & LEESN_STAT_ENABLE_LVL) strcat(flags, "使能 ");
                if (st32 & LEESN_STAT_ALARM)      strcat(flags, "报警!");

                printf("%s 关节:%d,状态:在线,字:0x%06X,步长:%d,角度:%.2f,电流:%d,速度:%d,报警:%s,IN0:%d,IN1:%d,标志:%s\n",
                       ts, j, (unsigned)(st32 & 0xFFFFFFu),
                       ok ? (int)pos : 0,
                       angle,
                       cur >= 0 ? cur : 0,
                       spd >= 0 ? spd : 0,
                       alm >= 0 ? leesn_alarm_text(alm) : "?",
                       in0, in1,
                       flags[0] ? flags : "—");
            }
        }

        fflush(stdout);

        /* 检查是否有按键 */
        if (WaitForSingleObject(hStdin, 300) == WAIT_OBJECT_0) {
            INPUT_RECORD ir;
            DWORD read;
            PeekConsoleInputA(hStdin, &ir, 1, &read);
            if (read > 0) {
                ReadConsoleInputA(hStdin, &ir, 1, &read);
                if (ir.EventType == KEY_EVENT && ir.Event.KeyEvent.bKeyDown) {
                    break;
                }
            }
        }
    }

    /* 恢复控制台模式 */
    SetConsoleMode(hStdin, old_mode);
    /* 丢弃用于退出的那次按键，避免它泄漏到主循环的 fgets 里被当成下一条命令 */
    FlushConsoleInputBuffer(hStdin);
    return 0;
}

/* cmd_scan：扫描总线电机在线状态
 * 对关节 1..6 逐个发状态查询（读 0x0006），收到响应即在线。
 * 拨码型驱动器总线地址由拨码决定，扫描只判断“有无响应”，不读 0x0066。 */
static int cmd_scan(Robot *robot)
{
    int j;
    int online_list[6];
    int online_count = 0;
    int masked_count = 0;

    printf("总线电机扫描（关节 1..6）...\n\n");

    for (j = 1; j <= 6; j++) {
        if (robot_is_masked(robot, j)) {
            printf("  关节 %d: 已屏蔽（跳过）\n", j);
            masked_count++;
            continue;
        }
        if (robot_is_online(robot, j)) {
            printf("  关节 %d: 在线\n", j);
            online_list[online_count++] = j;
        } else {
            printf("  关节 %d: 离线（无响应）\n", j);
        }
        Sleep(30); /* 给总线留方向切换余量 */
    }

    printf("\n扫描结果: %d/6 在线", online_count);
    if (online_count > 0) {
        printf("，在线关节 ");
        for (int k = 0; k < online_count; k++) {
            printf("%s%d", k > 0 ? "," : "", online_list[k]);
        }
    }
    if (masked_count > 0) {
        printf("，%d 个关节被屏蔽", masked_count);
    }
    printf("\n");
    return 0;
}

/* cmd_diag：回零诊断读数 —— 辨识"真堵转"与"传动打滑/跳齿"
 * 背景：闭环电机的 0x0004 是编码器位置。顶死后若它仍按满速累加，说明电机轴真的
 * 还在转，那不是堵转而是同步带跳齿/传动打滑（伴随皮带响声）。此时无论怎么调电流
 * 阈值都判不到，且继续顶会磨坏皮带 —— 必须先解决机械侧。
 * 判定要点：
 *   实际速度(0x00D6) ≈ 设定速度 且 电流高            → 电机轴在转 = 打滑/跳齿
 *   实际速度 ≈ 0 且 位置偏差(0x0011) 持续累积        → 命令走、电机没转 = 真堵转
 * 顺带输出细分(0x0024) 与配置 ENCODER_STEPS_PER_REV 的对比，确认角度换算口径。
 * joint_only：1..6 只查该轴并连续采样 3 次（看增量）；0 = 全轴各采样 1 次。 */
static int cmd_diag(Robot *robot, int joint_only)
{
    int j, lo, hi, samples, s;

    if (joint_only >= 1 && joint_only <= 6) {
        lo = hi = joint_only;
        samples = 3;
    } else {
        lo = 1; hi = 6; samples = 1;
    }

    printf("\n回零诊断读数（闭环判据）\n");
    printf("  配置细分 ENCODER_STEPS_PER_REV = %d\n\n", (int)ENCODER_STEPS_PER_REV);
    printf("  %-6s %-15s %-9s %-11s %-10s %-11s %-8s %s\n",
           "关节", "细分(实际/配置)", "编码器线数", "实际速度rpm", "位置偏差",
           "位置(步)", "电流mA", "状态字");
    printf("  ------ --------------- --------- ----------- ---------- ----------- -------- --------\n");

    for (j = lo; j <= hi; j++) {
        int32_t prev_pos = 0;
        int     prev_err = 0;
        int     have_prev = 0;

        if (robot_is_masked(robot, j)) {
            printf("  %-6d 已屏蔽\n", j);
            continue;
        }
        for (s = 0; s < samples; s++) {
            uint32_t st32 = 0;
            int32_t subdiv, pos, spd_raw;
            int     enc_lines, pos_err, cur, pos_ok = 0;
            char    tag[16], subdiv_txt[32], enc_txt[16], spd_txt[16];
            char    err_txt[16], pos_txt[16], cur_txt[16];

            if (motor_read_status(robot, j, &st32) != ERR_NONE) {
                printf("  %-6d 离线\n", j);
                break;
            }
            subdiv    = motor_read_subdivision(robot, j);
            enc_lines = motor_read_enc_lines(robot, j);
            spd_raw   = motor_read_speed_raw(robot, j);
            pos_err   = motor_read_pos_err(robot, j);
            pos       = motor_read_position(robot, j, &pos_ok);
            cur       = motor_read_current(robot, j);

            if (samples > 1) snprintf(tag, sizeof(tag), "%d#%d", j, s + 1);
            else             snprintf(tag, sizeof(tag), "%d", j);

            if (subdiv < 0) {
                snprintf(subdiv_txt, sizeof(subdiv_txt), "读失败");
            } else if (subdiv == (int32_t)ENCODER_STEPS_PER_REV) {
                snprintf(subdiv_txt, sizeof(subdiv_txt), "%d 一致", (int)subdiv);
            } else {
                snprintf(subdiv_txt, sizeof(subdiv_txt), "%d/%d 不符",
                         (int)subdiv, (int)ENCODER_STEPS_PER_REV);
            }
            if (enc_lines >= 0) snprintf(enc_txt, sizeof(enc_txt), "%d", enc_lines);
            else                snprintf(enc_txt, sizeof(enc_txt), "—");
            if (spd_raw >= 0)   snprintf(spd_txt, sizeof(spd_txt), "%.2f", (double)spd_raw / 100.0);
            else                snprintf(spd_txt, sizeof(spd_txt), "—");
            if (pos_err >= 0)   snprintf(err_txt, sizeof(err_txt), "%d", pos_err);
            else                snprintf(err_txt, sizeof(err_txt), "—");
            if (pos_ok)         snprintf(pos_txt, sizeof(pos_txt), "%d", (int)pos);
            else                snprintf(pos_txt, sizeof(pos_txt), "—");
            if (cur >= 0)       snprintf(cur_txt, sizeof(cur_txt), "%d", cur);
            else                snprintf(cur_txt, sizeof(cur_txt), "—");

            printf("  %-6s %-15s %-9s %-11s %-10s %-11s %-8s 0x%06X\n",
                   tag, subdiv_txt, enc_txt, spd_txt, err_txt, pos_txt, cur_txt,
                   (unsigned)(st32 & 0xFFFFFFu));

            if (have_prev && pos_ok && pos_err >= 0 && prev_err >= 0) {
                printf("        └ Δ位置=%+d 步, Δ偏差=%+d 步（%dms 内）\n",
                       (int)(pos - prev_pos), pos_err - prev_err,
                       (samples > 1) ? 400 : 0);
            }
            if (pos_ok)   prev_pos = pos;
            if (pos_err >= 0) prev_err = pos_err;
            have_prev = 1;

            if (samples > 1 && s < samples - 1) Sleep(400);
        }
    }
    if (samples > 1) {
        int k;
        int32_t tp;
        uint32_t ts, t0, dt_pos = 0, dt_cur = 0;

        /* 总线测速：堵转轮询周期 ≈ 每轮事务数 × 单事务耗时。
         * 事务数是可控项（位置+状态合并后已由 3→2），这里量化单事务耗时下限。 */
        t0 = GetTickCount();
        for (k = 0; k < 20; k++) (void)motor_read_pos_status(robot, lo, &tp, &ts);
        dt_pos = GetTickCount() - t0;
        t0 = GetTickCount();
        for (k = 0; k < 20; k++) (void)motor_read_current(robot, lo);
        dt_cur = GetTickCount() - t0;

        printf("\n  总线测速（关节%d，各 20 次）：\n", lo);
        printf("    位置+状态合并读  %u ms / 20 次 = %.1f ms/次\n",
               (unsigned)dt_pos, (double)dt_pos / 20.0);
        printf("    电流读            %u ms / 20 次 = %.1f ms/次\n",
               (unsigned)dt_cur, (double)dt_cur / 20.0);
        printf("    堵转轮询周期 ≈ %.1f ms（2 事务/轮）\n",
               (double)dt_pos / 20.0 + (double)dt_cur / 20.0);
    }
    if (samples > 1) {
        printf("\n  判定提示：\n");
        printf("    实际速度≈设定速度 且 电流高、Δ位置≈额定步数 → 电机轴仍在转 = 打滑/跳齿（机械问题）\n");
        printf("    实际速度≈0、Δ位置≈0 且 Δ偏差持续累积        → 命令走电机不转 = 真堵转（判据侧可解）\n");
    }
    printf("\n");
    return 0;
}

/* ============================================================
 * 运动学命令（fk / ik）
 * ------------------------------------------------------------
 * 复用 kinematics 模块：dh_forward（正解）、ik_solve + ik_filter_by_limits
 * + ik_select_best（球腕解耦解析逆解）。纯离线计算，不占用 RS485 总线。
 * ============================================================ */

#define KIN_D2R (3.14159265358979323846 / 180.0)
#define KIN_R2D (180.0 / 3.14159265358979323846)

/* rpy_deg_to_pose：XYZ(mm) + ZYX 欧拉角(度) → 4x4 齐次矩阵
 * （dh_pose_to_xyz_rpy 的逆：R = Rz(rz)·Ry(ry)·Rx(rx)） */
static void rpy_deg_to_pose(double x, double y, double z,
                            double rx_deg, double ry_deg, double rz_deg,
                            double pose[4][4])
{
    Mat3 rz, ry, rx, rzy, r;
    int i, j;

    mat3_rotz(rz_deg * KIN_D2R, rz);
    mat3_roty(ry_deg * KIN_D2R, ry);
    mat3_rotx(rx_deg * KIN_D2R, rx);
    mat3_mul(rz, ry, rzy);   /* Rz * Ry */
    mat3_mul(rzy, rx, r);    /* (Rz*Ry) * Rx */

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            pose[i][j] = r[i][j];
        }
    }
    pose[0][3] = x;
    pose[1][3] = y;
    pose[2][3] = z;
    pose[3][0] = 0.0;
    pose[3][1] = 0.0;
    pose[3][2] = 0.0;
    pose[3][3] = 1.0;
}

/* print_fk_line：打印一组关节角（度）的 FK 结果（末端 XYZ mm + RPY 度） */
static void print_fk_line(const char *tag, const double *joints_deg)
{
    double pose[4][4];
    double xyz[3], rpy[3];

    dh_forward(DH_TABLE, joints_deg, pose);
    dh_pose_to_xyz_rpy(pose, xyz, rpy);

    printf("  %-3s J: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  ->  "
           "X=%8.3f Y=%8.3f Z=%8.3f mm  RPY=%7.2f %7.2f %7.2f deg\n",
           tag,
           joints_deg[0], joints_deg[1], joints_deg[2],
           joints_deg[3], joints_deg[4], joints_deg[5],
           xyz[0], xyz[1], xyz[2],
           rpy[0] * KIN_R2D, rpy[1] * KIN_R2D, rpy[2] * KIN_R2D);
}

/* cmd_fk：正运动学。
 *   无参数      —— 打印预设验证姿态表 A~F（供实机 movej 到位后量测对照）；
 *   fk:J1:...:J6 —— 给定关节角（度）算末端位姿。 */
static int cmd_fk(const ParsedCmd *cmd)
{
    static const double presets[6][6] = {
        {  0.0,   0.0,   0.0, 0.0, 0.0, 0.0 },  /* A：全轴零位（home） */
        { 90.0,   0.0,   0.0, 0.0, 0.0, 0.0 },  /* B：仅 J1 转 +90° */
        { 45.0,   0.0,   0.0, 0.0, 0.0, 0.0 },  /* C：仅 J1 转 +45° */
        {  0.0,  45.0,   0.0, 0.0, 0.0, 0.0 },  /* D：仅 J2 抬 +45° */
        {  0.0, -45.0,   0.0, 0.0, 0.0, 0.0 },  /* E：仅 J2 压 -45° */
        {  0.0,   0.0, -90.0, 0.0, 0.0, 0.0 },  /* F：仅 J3 转 -90° */
    };
    static const char *tags[6] = { "A", "B", "C", "D", "E", "F" };
    int i;

    printf("\n正运动学（DH 表 kinematics/dh_params.h；位置 mm，角度 度）\n");

    if (cmd->val_count == 6) {
        print_fk_line("in", cmd->vals);
    } else {
        printf("  预设验证姿态（实机 movej 到位后量测末端 XYZ 对照，±3mm 内判可信）：\n");
        for (i = 0; i < 6; i++) {
            print_fk_line(tags[i], presets[i]);
        }
    }
    printf("\n");
    return 0;
}

/* cmd_ik：逆运动学。
 * 给定目标位姿（X:Y:Z:RX:RY:RZ，位置 mm、姿态 ZYX 欧拉角 度），
 * 输出全部候选解，并按"相对零位关节角变化最小"推荐一组，附 FK 回代误差自检。 */
static int cmd_ik(const ParsedCmd *cmd)
{
    double pose[4][4];
    double sols[IK_MAX_SOLUTIONS][6];
    double kept[IK_MAX_SOLUTIONS][6];
    double zero[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double best[6];
    double back[4][4], xyz[3], rpy[3];
    double dx, dy, dz;
    int cnt, kept_cnt, i;

    rpy_deg_to_pose(cmd->vals[0], cmd->vals[1], cmd->vals[2],
                    cmd->vals[3], cmd->vals[4], cmd->vals[5], pose);

    printf("\n逆运动学（目标位姿 X=%.3f Y=%.3f Z=%.3f mm，RPY=%.2f %.2f %.2f deg）\n",
           cmd->vals[0], cmd->vals[1], cmd->vals[2],
           cmd->vals[3], cmd->vals[4], cmd->vals[5]);

    cnt = ik_solve(DH_TABLE, pose, sols);
    if (cnt <= 0) {
        printf("  无解：目标位姿不可达，或落在解析 IK 的退化（奇异）分支\n\n");
        return 0;
    }
    printf("  解析解组数：%d\n", cnt);

    /* 关节软限位尚未标定，暂不限位（limits=NULL 表示全部保留） */
    kept_cnt = ik_filter_by_limits(sols, cnt, NULL, kept);
    for (i = 0; i < kept_cnt; i++) {
        printf("    解%d: J1=%8.2f J2=%8.2f J3=%8.2f J4=%8.2f J5=%8.2f J6=%8.2f\n",
               i + 1, kept[i][0], kept[i][1], kept[i][2],
               kept[i][3], kept[i][4], kept[i][5]);
    }

    if (ik_select_best(kept, kept_cnt, zero, NULL, best) != 0) {
        printf("  推荐解：无\n\n");
        return 0;
    }
    printf("  推荐解（相对零位变化最小）：J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f\n",
           best[0], best[1], best[2], best[3], best[4], best[5]);

    /* FK 回代自检：推荐解应能复现目标位姿 */
    dh_forward(DH_TABLE, best, back);
    dh_pose_to_xyz_rpy(back, xyz, rpy);
    dx = xyz[0] - cmd->vals[0];
    dy = xyz[1] - cmd->vals[1];
    dz = xyz[2] - cmd->vals[2];
    printf("  FK 回代：X=%.3f Y=%.3f Z=%.3f mm，RPY=%.2f %.2f %.2f deg（位置误差 %.3f mm）\n",
           xyz[0], xyz[1], xyz[2],
           rpy[0] * KIN_R2D, rpy[1] * KIN_R2D, rpy[2] * KIN_R2D,
           sqrt(dx * dx + dy * dy + dz * dz));
    printf("\n");
    return 0;
}

/* cmd_dispatch：命令分发总入口（原 main.c 交互循环 switch 整体迁入）
 * 返回 1 表示用户请求退出（exit/quit），否则 0。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd)
{
    switch (cmd->type) {
    case CMD_HOME: {
        /* 回零期间暂停后台监控，避免其全轴扫描与堵转采样抢占 RS485 总线 */
        monitor_stop(mon);
        ErrCode rc = (cmd->joint >= 1) ? robot_home_single(robot, cmd->joint)
                                       : robot_home(robot);
        if (rc != ERR_NONE) printf("[错误] 回零失败：%s\n", err_str(rc));
        if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
            printf("[警告] 回零后监控线程重启失败\n");
        break;
    }
    case CMD_HOMEJ: {
        monitor_stop(mon);
        ErrCode rc = robot_home_joint(robot, cmd->joint, cmd->angle_deg, cmd->speed_rpm);
        if (rc != ERR_NONE) printf("[错误] 单轴回零失败：%s\n", err_str(rc));
        if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
            printf("[警告] 回零后监控线程重启失败\n");
        break;
    }
    case CMD_MOVEJ: {
        ErrCode rc = robot_movej(robot, cmd->joint, cmd->angle_deg, cmd->speed_rpm);
        if (rc != ERR_NONE) printf("[错误] 运动指令失败：%s\n", err_str(rc));
        break;
    }
    case CMD_ENABLE: {
        ErrCode rc = robot_enable(robot, cmd->joint);
        if (rc != ERR_NONE) printf("[错误] 使能失败：%s\n", err_str(rc));
        break;
    }
    case CMD_DISABLE: {
        ErrCode rc = robot_disable(robot, cmd->joint);
        if (rc != ERR_NONE) printf("[错误] 失能失败：%s\n", err_str(rc));
        break;
    }
    case CMD_STATUS:
        cmd_status(robot);
        break;
    case CMD_MASK:
    case CMD_UNMASK: {
        int is_mask = (cmd->type == CMD_MASK);
        if (cmd->joint_count > 0) {
            for (int k = 0; k < cmd->joint_count; k++) {
                ErrCode rc = is_mask ? robot_mask(robot, cmd->joints[k])
                                     : robot_unmask(robot, cmd->joints[k]);
                if (rc != ERR_NONE) {
                    printf("[错误] %s关节%d失败：%s\n", is_mask ? "屏蔽" : "恢复",
                           cmd->joints[k], err_str(rc));
                }
            }
        } else {
            ErrCode rc = is_mask ? robot_mask(robot, cmd->joint)
                                 : robot_unmask(robot, cmd->joint);
            if (rc != ERR_NONE) printf("[错误] %s失败：%s\n", is_mask ? "屏蔽" : "恢复", err_str(rc));
        }
        break;
    }
    case CMD_SCAN:
        cmd_scan(robot);
        break;
    case CMD_DIAG:
        cmd_diag(robot, cmd->joint);
        break;
    case CMD_TORQUE: {
        monitor_stop(mon);
        ErrCode rc = robot_torque_probe(robot, cmd->joint, cmd->torque_level);
        if (rc != ERR_NONE) printf("[错误] 力矩碰撞诊断失败：%s\n", err_str(rc));
        if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
            printf("[警告] 诊断后监控线程重启失败\n");
        break;
    }
    case CMD_FK:
        cmd_fk(cmd);
        break;
    case CMD_IK:
        cmd_ik(cmd);
        break;
    case CMD_CALIB:
        printf("单关节调试功能未启用\n");
        break;
    case CMD_HELP:
        cmd_print_help();
        break;
    case CMD_EXIT:
        return 1;
    case CMD_EMPTY:
        break;
    default:
        printf("[警告] 未知命令，输入 help 查看帮助\n");
        break;
    }
    return 0;
}
