/*
 * movl_tip_trace.c —— MoveL 笔尖轨迹追踪（纯离线，不接硬件、不动臂）
 * ------------------------------------------------------------
 * 为什么需要它：2026-09-19 用户报"画的线是斜的、落点不对"，但离线复算
 * **法兰**轨迹是精确的直线（X/Z 零漂移 <0.05mm）。法兰直 ≠ 笔尖直 ——
 * 笔尖在法兰下方 tool_length 处，姿态只要沿路径变一点，笔尖就绕法兰摆。
 *
 * 本工具用【程序规划的同一组关节角】，分别算法兰轨迹和笔尖轨迹，
 * 直接回答："斜线是姿态造成的，还是 tool_length 造成的，还是都不是"。
 *
 * 用法：movl_tip_trace.exe  J1,J2,J3,J4,J5,J6  X,Y,Z,Rx,Ry,Rz  [tool_length]
 *   参数1 = 起点关节角（度），抄 `getpos` 那一行
 *   参数2 = 你要下发的 movel 前 6 个数（姿态那三个必须原样抄！）
 *   参数3 = 法兰面到笔尖的长度(mm)，省略则 41.17
 *
 * 例：  movl_tip_trace.exe 0,0,110,0,70,0  143,132,155.16,-180,0,-180  41.17
 *
 * 编译：C:/Qt/Tools/mingw1310_64/bin/gcc.exe tests/movl_tip_trace.c \
 *           src/kinematics/dh.c src/kinematics/ik.c src/trajectory/line.c \
 *           -Isrc -lm -o build/bin/movl_tip_trace.exe
 *
 * 判定：看最后三行
 *   ① 法兰直线度 < 0.05mm 而 笔尖直线度 > 0.5mm  ⇒ 姿态在变，笔尖绕法兰甩（画斜线）
 *   ② 两者都直、但笔尖位移 ≠ 法兰位移           ⇒ 纯 tool_length 平移（整体偏，不斜）
 *   ③ 两者都斜                                   ⇒ 规划层问题，不是工具问题
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"
#include "config/robot_config.h"

#define RJ 6
#define STEP_MM 1.0          /* 与 commands.c MOVL_STEP_MM 一致 */

static const double LIM_MIN[RJ] = ROBOT_JOINT_LIMIT_MIN_DEG;
static const double LIM_MAX[RJ] = ROBOT_JOINT_LIMIT_MAX_DEG;
static const double RAD2DEG = 180.0 / 3.14159265358979323846;

static int parse6(const char *s, double v[6])
{
    char buf[256];
    char *tok;
    int n = 0;
    if (strlen(s) >= sizeof(buf)) return -1;
    strcpy(buf, s);
    tok = strtok(buf, ",");
    while (tok != NULL && n < 6) {
        char *end = NULL;
        v[n] = strtod(tok, &end);
        if (end == tok) return -1;
        n++;
        tok = strtok(NULL, ",");
    }
    return (n == 6) ? 0 : -1;
}

/* 点 p 到线段 ab 的距离（3D / XY 平面两用，z 置 0 即投影） */
static double dev_from_seg(const double p[3], const double a[3], const double b[3], int xy_only)
{
    double d[3] = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
    double w[3] = { p[0]-a[0], p[1]-a[1], p[2]-a[2] };
    double dd, t, q[3];
    if (xy_only) { d[2] = 0.0; w[2] = 0.0; }
    dd = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
    if (dd < 1e-12) return sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    t = (w[0]*d[0] + w[1]*d[1] + w[2]*d[2]) / dd;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    q[0] = w[0] - t*d[0];
    q[1] = w[1] - t*d[1];
    q[2] = w[2] - t*d[2];
    return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2]);
}

/* 角度差，做 360° 环绕（180 与 -180 是同一个角，不能简单相减） */
static double ang_delta(double a, double b)
{
    double d = fabs(a - b);
    while (d > 180.0) d = 360.0 - d;
    return d;
}

/* 用当前 tool_length 设置算 FK，返回 xyz 与 rpy(deg) */
static void fk_of(const double q[6], double xyz[3], double rpy[3])
{
    double m[4][4], r[3];
    dh_forward(DH_TABLE, q, m);
    dh_pose_to_xyz_rpy(m, xyz, r);
    rpy[0] = r[0] * RAD2DEG;
    rpy[1] = r[1] * RAD2DEG;
    rpy[2] = r[2] * RAD2DEG;
}

int main(int argc, char **argv)
{
    double q0[RJ], target[6];
    double start_pose[6], start_xyz[3], start_rpy[3];
    double q_seq[LINE_MAX_POINTS][6];
    LinePath path;
    JointLimit limits[RJ];
    double m[4][4], r[3];
    double L = 41.17;
    int mode_joint = 0;      /* 1 = smooth（关节空间单段插补），0 = 笛卡尔直线插补 */
    int count, i, j, rc;
    char reason[64] = {0};
    int fail_idx = -1;

    /* ---- 统计累加器 ---- */
    double fla_first[3], fla_last[3], tip_first[3], tip_last[3];
    double fla_dev_max = 0.0, tip_dev_max = 0.0, tip_dev_xy_max = 0.0;
    double fla_len = 0.0, tip_len = 0.0;

    if (argc < 3) {
        printf("用法: %s  J1..J6  X,Y,Z,Rx,Ry,Rz  [tool_length]  [line|joint]\n", argv[0]);
        printf("例  : %s  0,0,110,0,70,0  143,132,155.16,-180,0,-180  41.17\n", argv[0]);
        printf("      %s  0,0,110,0,70,0  143,152,155.16,-180,0,-180  41.17  joint\n", argv[0]);
        printf("  line(默认) = 笛卡尔直线插补；joint = smooth 单段（关节空间直线）\n");
        return 2;
    }
    if (parse6(argv[1], q0) != 0) { printf("[错误] 参数1 须为 6 个关节角，逗号分隔\n"); return 2; }
    if (parse6(argv[2], target) != 0) { printf("[错误] 参数2 须为 6 个数 X,Y,Z,Rx,Ry,Rz\n"); return 2; }
    if (argc >= 4) {
        char *end = NULL;
        L = strtod(argv[3], &end);
        if (end == argv[3]) { printf("[错误] 参数3 tool_length 不是数字\n"); return 2; }
    }
    if (argc >= 5) {
        if (strcmp(argv[4], "joint") == 0 || strcmp(argv[4], "smooth") == 0) {
            mode_joint = 1;
        } else if (strcmp(argv[4], "line") == 0) {
            mode_joint = 0;
        } else {
            printf("[错误] 参数4 须为 line(默认) 或 joint/smooth\n");
            return 2;
        }
    }
    for (j = 0; j < RJ; j++) {
        limits[j].min_deg = LIM_MIN[j];
        limits[j].max_deg = LIM_MAX[j];
    }

    /* ---------- 1) 起点位姿（tool_length=0，与程序 ini 现状一致） ---------- */
    dh_set_tool_length(0.0);
    fk_of(q0, start_xyz, start_rpy);
    for (j = 0; j < 3; j++) start_pose[j] = start_xyz[j];
    for (j = 0; j < 3; j++) start_pose[j + 3] = start_rpy[j];

    printf("================ MoveL 笔尖轨迹追踪（离线，不动臂）================\n");
    printf("起点关节角  J = %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           q0[0], q0[1], q0[2], q0[3], q0[4], q0[5]);
    printf("起点位姿(法兰) X=%.3f Y=%.3f Z=%.3f   Rx=%.3f Ry=%.3f Rz=%.3f\n",
           start_pose[0], start_pose[1], start_pose[2],
           start_pose[3], start_pose[4], start_pose[5]);
    printf("目标位姿(输入) X=%.3f Y=%.3f Z=%.3f   Rx=%.3f Ry=%.3f Rz=%.3f\n",
           target[0], target[1], target[2], target[3], target[4], target[5]);
    printf("工具长度 tool_length = %.2f mm（法兰面 → 笔尖）\n", L);
    printf("插补模式             = %s\n", mode_joint
           ? "joint（smooth 单段：六轴按行程比例配速同时到达，一次启动、不卡，但末端有弓高）"
           : "line（笛卡尔直线插补：末端走直线，段间由驱动器各自启停）");
    printf("------------------------------------------------------------------\n");

    /* 姿态差预警：这是"画斜线"最常见的成因 */
    {
        double drx = ang_delta(start_pose[3], target[3]);
        double dry = ang_delta(start_pose[4], target[4]);
        double drz = ang_delta(start_pose[5], target[5]);
        if (drx > 0.05 || dry > 0.05 || drz > 0.05) {
            printf("[警告] 目标姿态与起点姿态【不一致】 ΔRx=%.2f° ΔRy=%.2f° ΔRz=%.2f°\n",
                   drx, dry, drz);
            printf("       moveL 会用 SLERP 把姿态从起点【一路拧到】目标 ⇒ 中途姿态一直在变。\n");
            printf("       笔尖在法兰下方 %.2fmm，姿态转 θ 会让笔尖横移 %.2f×sin(θ)。\n",
                   L, L);
            printf("       ⇒ 法兰走直线，笔尖画【弧/斜线】，落点也会偏。\n");
            printf("       想要笔尖走直线：把 Rx,Ry,Rz 抄成 getpos 打印的那些数（含负号）。\n");
        } else {
            printf("[OK] 目标姿态与起点姿态一致（差 <0.05°）⇒ 姿态全程恒定。\n");
        }
    }
    printf("------------------------------------------------------------------\n");

    /* ---------- 2) 规划（tool_length=0，与程序一致） ---------- */
    {
        double dx = target[0] - start_pose[0];
        double dy = target[1] - start_pose[1];
        double dz = target[2] - start_pose[2];
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        count = line_count_for_distance(dist, STEP_MM);
    }
    if (mode_joint) {
        /* smooth（单段）：只给终点，六轴按行程比例配速、同时到达 ⇒ 关节空间
         * 直线插补。驱动器各自做梯形规划，全程只有【一次】启动，所以不卡，
         * 但末端走的是关节空间直线，不是笛卡尔直线 ⇒ 有弓高。
         * 这里用 FK 逐点反算真实轨迹，不碰任何"预测值"—— 本项目已在弓高
         * 预测上栽过三次（低估 11.6 倍、高估到 9.55mm 而实测很直）。 */
        double mt[4][4];
        double sols[IK_MAX_SOLUTIONS][6];
        double unwrapped[IK_MAX_SOLUTIONS][6];
        double filtered[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        double q_end[6];
        int cnt, n;

        dh_set_tool_length(0.0);
        line_pose_to_matrix(target, mt);
        cnt = ik_solve_ref(DH_TABLE, mt, q0, sols, info);
        if (cnt <= 0) { printf("[错误] 目标位姿 IK 无解\n"); return 1; }
        n = ik_unwrap_solutions(sols, cnt, q0, unwrapped);
        n = ik_filter_by_limits(unwrapped, n, limits, filtered);
        if (n <= 0) { printf("[错误] 目标位姿 IK 候选解全部越软限位\n"); return 1; }
        if (ik_select_best_continuous(filtered, n, q0, NULL, q_end) != 0) {
            printf("[错误] 目标位姿分支连续选解失败\n"); return 1;
        }
        for (i = 0; i < count; i++) {
            double t = (double)i / (double)(count - 1);
            for (j = 0; j < 6; j++) q_seq[i][j] = q0[j] + (q_end[j] - q0[j]) * t;
        }
    } else {
        if (line_plan(start_pose, target, count, &path) != 0) {
            printf("[错误] 位姿插值失败（NaN？）\n");
            return 1;
        }
        rc = line_solve(&path, DH_TABLE, limits, q0, q_seq, &fail_idx, reason);
        if (rc != 0) {
            printf("[错误] IK 失败：第 %d 点，%s\n", fail_idx, reason);
            return 1;
        }
    }

    /* ---------- 3) 逐点：法兰 vs 笔尖 ----------
     * 两趟：第一趟把所有点算完并取首尾，第二趟才拿首尾连线做偏离基准。
     * （一趟写法会在 i<count-1 时把还没赋值的末点当参考，直线度直接飞掉。） */
    static double fla[LINE_MAX_POINTS][3];
    static double tip[LINE_MAX_POINTS][3];
    static double nrm[LINE_MAX_POINTS][3];   /* 法兰 +Z 轴（法线）在世界系的方向 */
    double tilt_max = 0.0;

    printf(" 点   法兰X    法兰Y    法兰Z  |  笔尖X    笔尖Y    笔尖Z  |   Rx      Ry      Rz   倾角\n");
    for (i = 0; i < count; i++) {
        double mm[4][4], xyz[3], rpy[3], frpy[3];

        dh_set_tool_length(0.0);
        dh_forward(DH_TABLE, q_seq[i], mm);
        dh_pose_to_xyz_rpy(mm, xyz, rpy);
        fla[i][0] = xyz[0]; fla[i][1] = xyz[1]; fla[i][2] = xyz[2];
        nrm[i][0] = mm[0][2]; nrm[i][1] = mm[1][2]; nrm[i][2] = mm[2][2];
        frpy[0] = rpy[0] * RAD2DEG; frpy[1] = rpy[1] * RAD2DEG; frpy[2] = rpy[2] * RAD2DEG;

        dh_set_tool_length(L);
        dh_forward(DH_TABLE, q_seq[i], mm);
        dh_pose_to_xyz_rpy(mm, xyz, rpy);
        tip[i][0] = xyz[0]; tip[i][1] = xyz[1]; tip[i][2] = xyz[2];

        /* 每 8 点打印一次，或首尾点 */
        if (i == 0 || i == count - 1 || (i % 8) == 0) {
            double cs = nrm[i][0]*nrm[0][0] + nrm[i][1]*nrm[0][1] + nrm[i][2]*nrm[0][2];
            if (cs > 1.0) cs = 1.0;
            if (cs < -1.0) cs = -1.0;
            printf("%3d  %8.3f %8.3f %8.3f  | %8.3f %8.3f %8.3f  | %7.2f %7.2f %7.2f  %5.2f\n",
                   i, fla[i][0], fla[i][1], fla[i][2], tip[i][0], tip[i][1], tip[i][2],
                   frpy[0], frpy[1], frpy[2], acos(cs) * RAD2DEG);
        }
    }

    for (j = 0; j < 3; j++) {
        fla_first[j] = fla[0][j];   fla_last[j] = fla[count-1][j];
        tip_first[j] = tip[0][j];   tip_last[j] = tip[count-1][j];
    }

    /* 第二趟：偏离 + 倾角。倾角用【法线夹角】衡量，不受欧拉角表示
     * （180 ↔ -180 跳变、万向锁附近耦合）影响，这才是笔尖摆动的真量度。 */
    for (i = 1; i < count - 1; i++) {
        double d1 = dev_from_seg(fla[i], fla_first, fla_last, 0);
        double d2 = dev_from_seg(tip[i], tip_first, tip_last, 0);
        double d3 = dev_from_seg(tip[i], tip_first, tip_last, 1);
        double cs = nrm[i][0]*nrm[0][0] + nrm[i][1]*nrm[0][1] + nrm[i][2]*nrm[0][2];
        double th;
        if (cs > 1.0) cs = 1.0;
        if (cs < -1.0) cs = -1.0;
        th = acos(cs) * RAD2DEG;
        if (d1 > fla_dev_max) fla_dev_max = d1;
        if (d2 > tip_dev_max) tip_dev_max = d2;
        if (d3 > tip_dev_xy_max) tip_dev_xy_max = d3;
        if (th > tilt_max) tilt_max = th;
    }

    fla_len = sqrt(pow(fla_last[0]-fla_first[0],2) + pow(fla_last[1]-fla_first[1],2) + pow(fla_last[2]-fla_first[2],2));
    tip_len = sqrt(pow(tip_last[0]-tip_first[0],2) + pow(tip_last[1]-tip_first[1],2) + pow(tip_last[2]-tip_first[2],2));

    printf("------------------------------------------------------------------\n");
    printf("法兰法线倾角变化（相对起点，最大） = %.3f°   ⇒ 笔尖理论横移 %.2f×sin(%.3f°) = %.3f mm\n",
           tilt_max, L, tilt_max, L * sin(tilt_max / RAD2DEG));
    printf("法兰位移 %.3f mm ｜ 笔尖位移 %.3f mm ｜ 差 %.3f mm\n",
           fla_len, tip_len, tip_len - fla_len);
    printf("------------------------------------------------------------------\n");
    printf("★ 法兰直线度（3D 最大偏离）      = %.4f mm\n", fla_dev_max);
    printf("★ 笔尖直线度（3D 最大偏离）      = %.4f mm\n", tip_dev_max);
    printf("★ 笔尖直线度（投影到水平纸面 XY）= %.4f mm   ← 这才是画在纸上的线\n", tip_dev_xy_max);
    printf("------------------------------------------------------------------\n");

    if (fla_dev_max < 0.05 && tip_dev_xy_max > 0.5) {
        printf("⇒ 判定：法兰走得【很直】，笔尖却【明显弯/斜】。\n");
        printf("  根因是【姿态沿路径在变】，笔尖绕法兰摆 %.2fmm 半径。\n", L);
        printf("  修法：把 movel 的 Rx,Ry,Rz 抄 getpos 打印的原值（含负号、含 -0.00）。\n");
    } else if (fla_dev_max < 0.05 && tip_dev_xy_max <= 0.5) {
        printf("⇒ 判定：法兰和笔尖都直。姿态没变，笔尖只是被 tool_length 整体平移。\n");
        printf("  这种情况【不会画斜线】；若落点不对，是 tool_length 没配（TCP 在法兰中心）。\n");
    } else {
        printf("⇒ 判定：法兰本身就不直（规划层/IK 问题），先解决这个再谈笔尖。\n");
    }

    (void)m; (void)r;
    return 0;
}
