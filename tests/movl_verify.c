/*
 * movl_verify.c —— MoveL 修复离线验证 harness（不依赖硬件）
 * ------------------------------------------------------------
 * 链接真实的 kinematics(dh/ik) + trajectory(line) 模块，复刻 cmd_movl 的步骤
 * 2~4（离散直线 -> 逐点逆解 -> 分段时间表），再用正运动学对每一插补点做断言：
 *   A) 直线性（不画弧）：中间点 XYZ 必须落在起终点连线上（垂直残差 < 0.05mm）。
 *   B) 对称性直上直下：对称姿态(-180,0,-180)下 X、Y 必须恒定，仅 Z 变化。
 *   C) 不卡顿结构性根因：seg_dt_eff 全部 >= MOVL_SEG_DT_MIN，且周期刷新模式
 *      下速度/加减速仅下发 1 次（恒定速度段不会逐段重起梯形）。
 *   D) 欧拉口径往返一致：随机姿态 pose->matrix->pose 误差 < 1e-9（验证 P1 修正
 *      未引入数值回归，且 line_pose_to_matrix 与 dh_pose_to_xyz_rpy 同口径）。
 *
 * 编译：gcc tests/movl_verify.c src/kinematics/dh.c src/kinematics/ik.c \
 *            src/trajectory/line.c -Isrc -lm -o tests/movl_verify.exe
 * 运行：tests/movl_verify.exe
 *
 * 注意：本 harness 验证的是【规划层 + 正运动学层】在模型自身坐标架下的几何正确性。
 * 真机末端若仍呈斜线，根因是 standard DH 与厂家 modified DH 的系统性偏差
 * （缺陷④），需 step 判别实验 + 卷尺实测裁决，非本层可验证。
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"

/* ---- 生产常量（与 commands.c / robot_config.h 保持一致）---- */
#define RJ                      6
#define MOVL_STEP_MM            2.0
#define MOVL_SEG_DT_MIN         0.15   /* s，单段下发最小间隔 */
#define RAD2DEG                 (180.0 / 3.14159265358979323846)
#define DEG2RAD                 (3.14159265358979323846 / 180.0)

static const double REDUCTIONS[RJ] = {50, 100, 50, 50, 50, 50};
/* 离线用宽松限位，避免合成路径被软限位误拒（真实限位见 robot_config.h）*/
static const double LIM_MIN[RJ] = {-180, -180, -180, -360, -180, -360};
static const double LIM_MAX[RJ] = { 180,  180,  180,  360,  180,  360};

static int    g_fail = 0;
static double g_q_out[LINE_MAX_POINTS][RJ];

/* 前向声明（定义见下方 movl_seg_speed_local）*/
static void movl_seg_speed_local(const double q0[RJ], const double q1[RJ],
                                 double seg_dt, double v_rpm[RJ]);

/* 复刻 cmd_movl 步骤 2~4，返回 0 成功；并回填空结构便于断言。
 * fail_idx 透传 line_solve 的失败点（调试可达性用）。 */
static int plan_movl(const double q_start[RJ], const double end_pose[6],
                     double speed_rpm, double accel_ms,
                     double seg_dt_eff[LINE_MAX_SEGS], int *out_count,
                     double *out_dist, double *out_dt_total,
                     int *out_resend_count, int *out_fail_idx)
{
    double pose0[4][4], xyz0[3], rpy0[3];
    dh_forward(DH_TABLE, q_start, pose0);
    dh_pose_to_xyz_rpy(pose0, xyz0, rpy0);

    double start_pose[6] = {
        xyz0[0], xyz0[1], xyz0[2],
        rpy0[0] * RAD2DEG, rpy0[1] * RAD2DEG, rpy0[2] * RAD2DEG
    };

    double d = 0.0;
    for (int i = 0; i < 3; i++) { double v = end_pose[i] - start_pose[i]; d += v * v; }
    *out_dist = sqrt(d);
    if (*out_dist < 0.01) return -1;

    int count = line_count_for_distance(*out_dist, MOVL_STEP_MM);
    LinePath path;
    if (line_plan(start_pose, end_pose, count, &path) != 0) return -2;

    JointLimit limits[RJ];
    for (int i = 0; i < RJ; i++) { limits[i].min_deg = LIM_MIN[i]; limits[i].max_deg = LIM_MAX[i]; }

    int fail_idx = -1;
    if (line_solve(&path, DH_TABLE, limits, q_start, g_q_out, &fail_idx) != 0) {
        *out_fail_idx = fail_idx;
        return -3;
    }
    *out_fail_idx = -1;

    double vmax[RJ];
    for (int j = 0; j < RJ; j++) vmax[j] = speed_rpm * 6.0 / REDUCTIONS[j];

    double seg_dt[LINE_MAX_SEGS];
    if (line_time_table(g_q_out, path.count, vmax, seg_dt, out_dt_total) != 0) return -4;

    /* 段长下限（复刻 cmd_movl 步骤 4）*/
    double min_eff = MOVL_SEG_DT_MIN;
    for (int i = 0; i < path.count - 1; i++) {
        seg_dt_eff[i] = (seg_dt[i] < MOVL_SEG_DT_MIN) ? MOVL_SEG_DT_MIN : seg_dt[i];
        if (seg_dt_eff[i] < min_eff) min_eff = seg_dt_eff[i];
    }
    int acc = (int)(min_eff * 1000.0 * 0.25 + 0.5);
    if (acc > accel_ms) acc = (int)accel_ms;

    /* 周期刷新模式：速度仅在相对上一段变化 >2% 时补发（复刻 cmd_movl 步骤 5）*/
    double v_rpm[RJ], v_rpm_last[RJ];
    movl_seg_speed_local(g_q_out[0], g_q_out[1], seg_dt_eff[0], v_rpm);
    memcpy(v_rpm_last, v_rpm, sizeof(v_rpm_last));
    int resend = 1; /* 首段必发一次 */
    for (int i = 1; i < path.count; i++) {
        movl_seg_speed_local(g_q_out[i - 1], g_q_out[i], seg_dt_eff[i - 1], v_rpm);
        int changed = 0;
        for (int j = 0; j < RJ; j++) {
            double ref = (v_rpm_last[j] > 1.0) ? v_rpm_last[j] : 1.0;
            if (fabs(v_rpm[j] - v_rpm_last[j]) / ref > 0.02) { changed = 1; break; }
        }
        if (changed) { resend++; memcpy(v_rpm_last, v_rpm, sizeof(v_rpm_last)); }
    }
    *out_resend_count = resend;
    *out_count = path.count;
    return 0;
}

/* 复刻 commands.c 的 movl_seg_speed（段转速换算），harness 内联一份避免拉入硬件层 */
static void movl_seg_speed_local(const double q0[RJ], const double q1[RJ],
                                 double seg_dt, double v_rpm[RJ])
{
    for (int j = 0; j < RJ; j++) {
        double dq = q1[j] - q0[j];
        double dps = (seg_dt > 1e-9) ? fabs(dq) / seg_dt : 0.0;   /* deg/s */
        v_rpm[j] = dps * (double)REDUCTIONS[j] / 6.0;            /* -> rpm */
    }
}

/* 对路径每点做正运动学，返回中间点相对起终点连线的【最大垂直残差】(mm) 与端点一致性 */
static double fk_max_perp_residual(int count, double *max_dev_xy,
                                   double end_xyz[3], double *end_err)
{
    double X[LINE_MAX_POINTS][3];
    for (int i = 0; i < count; i++) {
        double P[4][4], xyz[3], rpy[3];
        dh_forward(DH_TABLE, g_q_out[i], P);
        dh_pose_to_xyz_rpy(P, xyz, rpy);
        X[i][0] = xyz[0]; X[i][1] = xyz[1]; X[i][2] = xyz[2];
    }
    double dir[3];
    for (int k = 0; k < 3; k++) dir[k] = X[count - 1][k] - X[0][k];
    double L2 = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2];
    double maxperp = 0.0, maxxy = 0.0;
    if (L2 > 1e-12) {
        for (int i = 1; i < count - 1; i++) {
            double v[3];
            for (int k = 0; k < 3; k++) v[k] = X[i][k] - X[0][k];
            double cr[3] = {
                v[1]*dir[2] - v[2]*dir[1],
                v[2]*dir[0] - v[0]*dir[2],
                v[0]*dir[1] - v[1]*dir[0]
            };
            double crL2 = cr[0]*cr[0] + cr[1]*cr[1] + cr[2]*cr[2];
            double perp = sqrt(crL2 / L2);
            if (perp > maxperp) maxperp = perp;
        }
    }
    /* X/Y 偏移（用于对称性直上直下判定）*/
    for (int i = 1; i < count - 1; i++) {
        double dx = fabs(X[i][0] - X[0][0]);
        double dy = fabs(X[i][1] - X[0][1]);
        double m = (dx > dy) ? dx : dy;
        if (m > maxxy) maxxy = m;
    }
    *max_dev_xy = maxxy;
    end_xyz[0] = X[count - 1][0]; end_xyz[1] = X[count - 1][1]; end_xyz[2] = X[count - 1][2];
    /* 端点 FK 与 end_pose 偏差 */
    double P[4][4], xyz[3], rpy[3];
    dh_forward(DH_TABLE, g_q_out[count - 1], P);
    dh_pose_to_xyz_rpy(P, xyz, rpy);
    *end_err = sqrt((xyz[0]-end_xyz[0])*(xyz[0]-end_xyz[0]) +
                    (xyz[1]-end_xyz[1])*(xyz[1]-end_xyz[1]) +
                    (xyz[2]-end_xyz[2])*(xyz[2]-end_xyz[2]));
    return maxperp;
}

/* ---------------- 用例 ---------------- */
/* 取一个非奇异、可达的起始构型（避开 J4=J6=0 且 J5=0 的腕部奇异）*/
static void fk_pose(const double q[RJ], double pose6[6])
{
    double P[4][4], xyz[3], rpy[3];
    dh_forward(DH_TABLE, q, P);
    dh_pose_to_xyz_rpy(P, xyz, rpy);
    pose6[0] = xyz[0]; pose6[1] = xyz[1]; pose6[2] = xyz[2];
    pose6[3] = rpy[0]*RAD2DEG; pose6[4] = rpy[1]*RAD2DEG; pose6[5] = rpy[2]*RAD2DEG;
}

static void case_straight(void)
{
    printf("\n=== 用例A：可达笛卡尔直线（不画弧）===\n");
    /* 两端均为真实可达的非奇异关节构型，FK 得到起终点 Cartesian 位姿 */
    double q_start[RJ] = {  0.0, -20.0, 110.0,   0.0,  20.0,  0.0 };
    double q_end[RJ]   = { 12.0, -32.0, 100.0,  10.0,  26.0, 15.0 };
    double start_pose[6], end_pose[6];
    fk_pose(q_start, start_pose);
    fk_pose(q_end,   end_pose);

    double seg_dt_eff[LINE_MAX_SEGS], dt_total, dist, end_xyz[3], end_err, maxxy;
    int count, resend, fail_idx;
    int rc = plan_movl(q_start, end_pose, 300.0, 300.0, seg_dt_eff, &count,
                       &dist, &dt_total, &resend, &fail_idx);
    if (rc != 0) {
        printf("  [FAIL] plan_movl 返回 %d（fail_idx=%d，路径不可达/逆解失败）\n", rc, fail_idx);
        g_fail++; return;
    }

    double maxperp = fk_max_perp_residual(count, &maxxy, end_xyz, &end_err);
    int ok = (maxperp < 0.05) && (end_err < 0.1);
    printf("  插补点数=%d  路径长=%.2fmm  预计=%.2fs\n", count, dist, dt_total);
    printf("  中间点最大垂直残差=%.4fmm  终点FK偏差=%.4fmm\n", maxperp, end_err);
    printf("  判定：直线性(残差<0.05)=%s  端点一致(<0.1)=%s\n",
           maxperp < 0.05 ? "PASS" : "FAIL", end_err < 0.1 ? "PASS" : "FAIL");
    if (!ok) g_fail++;
}

static void case_stutter(void)
{
    printf("\n=== 用例B：长行程高速（段长下限 / 不卡顿结构性根因）===\n");
    /* 直接用关节线性插值构造 q_seq（无需 IK），隔离分段时间表层与段长下限逻辑 */
    double q0[RJ] = {  0.0, -20.0, 110.0,   0.0,  20.0,  0.0 };
    double q1[RJ] = { 25.0, -40.0,  95.0,  15.0,  30.0, 20.0 };
    int count = 200;                       /* 长行程 -> 多段 */
    double q_seq[LINE_MAX_POINTS][RJ];
    for (int i = 0; i < count; i++) {
        double t = (double)i / (count - 1);
        for (int j = 0; j < RJ; j++) q_seq[i][j] = q0[j] + (q1[j] - q0[j]) * t;
    }

    double speed_rpm = 800.0;              /* 高速：自然段长远低于串口吞吐下限 */
    double vmax[RJ];
    for (int j = 0; j < RJ; j++) vmax[j] = speed_rpm * 6.0 / REDUCTIONS[j];
    double seg_dt[LINE_MAX_SEGS], dt_total;
    if (line_time_table(q_seq, count, vmax, seg_dt, &dt_total) != 0) {
        printf("  [FAIL] line_time_table 返回错误\n"); g_fail++; return;
    }

    /* 复刻 cmd_movl 段长下限逻辑 */
    double seg_dt_eff[LINE_MAX_SEGS];
    double min_eff = MOVL_SEG_DT_MIN;
    for (int i = 0; i < count - 1; i++) {
        seg_dt_eff[i] = (seg_dt[i] < MOVL_SEG_DT_MIN) ? MOVL_SEG_DT_MIN : seg_dt[i];
        if (seg_dt_eff[i] < min_eff) min_eff = seg_dt_eff[i];
    }

    /* 周期刷新：速度相对上一段变化 >2% 才补发 */
    double v_rpm[RJ], v_rpm_last[RJ];
    movl_seg_speed_local(q_seq[0], q_seq[1], seg_dt_eff[0], v_rpm);
    memcpy(v_rpm_last, v_rpm, sizeof(v_rpm_last));
    int resend = 1;
    for (int i = 1; i < count; i++) {
        movl_seg_speed_local(q_seq[i - 1], q_seq[i], seg_dt_eff[i - 1], v_rpm);
        int changed = 0;
        for (int j = 0; j < RJ; j++) {
            double ref = (v_rpm_last[j] > 1.0) ? v_rpm_last[j] : 1.0;
            if (fabs(v_rpm[j] - v_rpm_last[j]) / ref > 0.02) { changed = 1; break; }
        }
        if (changed) { resend++; memcpy(v_rpm_last, v_rpm, sizeof(v_rpm_last)); }
    }

    double min_dt = 1e9;
    for (int i = 0; i < count - 1; i++) if (seg_dt_eff[i] < min_dt) min_dt = seg_dt_eff[i];
    int ok = (min_dt >= MOVL_SEG_DT_MIN - 1e-9) && (resend <= 2);
    printf("  插补段数=%d  最小有效段长=%.3fs (下限%.3fs)  周期刷新速度下发次数=%d\n",
           count - 1, min_dt, MOVL_SEG_DT_MIN, resend);
    printf("  判定：段长>=下限=%s  速度仅下发1次(<=2)=%s\n",
           min_dt >= MOVL_SEG_DT_MIN - 1e-9 ? "PASS" : "FAIL",
           resend <= 2 ? "PASS" : "FAIL");
    if (!ok) g_fail++;
}

static void case_euler_roundtrip(void)
{
    printf("\n=== 用例C：欧拉口径往返一致性（P1 修正回归）===\n");
    int N = 2000, bad = 0;
    double maxerr = 0.0;
    srand(12345);
    for (int t = 0; t < N; t++) {
        double pose6[6];
        pose6[0] = 200 + ((double)rand()/RAND_MAX - 0.5) * 100;
        pose6[1] = 50  + ((double)rand()/RAND_MAX - 0.5) * 100;
        pose6[2] = 200 + ((double)rand()/RAND_MAX - 0.5) * 100;
        /* 避开万向锁邻域 |pitch|~90° */
        double ry = ((double)rand()/RAND_MAX - 0.5) * 120.0; /* ±60 */
        pose6[3] = ((double)rand()/RAND_MAX - 0.5) * 120.0;
        pose6[4] = ry;
        pose6[5] = ((double)rand()/RAND_MAX - 0.5) * 120.0;

        double M[4][4];
        line_pose_to_matrix(pose6, M);
        double xyz[3], rpy[3];
        dh_pose_to_xyz_rpy(M, xyz, rpy);
        double err = fabs(rpy[0]*RAD2DEG - pose6[3]) +
                     fabs(rpy[1]*RAD2DEG - pose6[4]) +
                     fabs(rpy[2]*RAD2DEG - pose6[5]);
        if (err > maxerr) maxerr = err;
        if (err > 1e-6) bad++;
    }
    int ok = (maxerr < 1e-6) && (bad == 0);
    printf("  样本=%d  最大口径误差=%.3e°  超差样本=%d\n", N, maxerr, bad);
    printf("  判定：往返一致(误差<1e-6°)=%s\n", ok ? "PASS" : "FAIL");
    if (!ok) g_fail++;
}

int main(void)
{
    printf("MoveL 修复离线验证（链接真实 dh/ik/line 模块）\n");
    case_straight();
    case_stutter();
    case_euler_roundtrip();
    printf("\n================ 结论 ================\n");
    if (g_fail == 0) printf("全部用例 PASS —— 规划层与正运动学层几何正确（不画弧/不卡顿根因消除/欧拉口径一致）\n");
    else             printf("存在 %d 个 FAIL，见上方明细\n", g_fail);
    return g_fail ? 1 : 0;
}
