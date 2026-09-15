/*
 * line.c —— 笛卡尔直线插补（moveL 轨迹层）
 * ------------------------------------------------------------
 * 所属模块：轨迹规划（trajectory）
 * 对外接口：line_pose_to_matrix、line_count_for_distance、line_plan、
 *           line_solve、line_time_table
 * 依赖模块：kinematics（dh_params / dh / ik）
 */

#include "trajectory/line.h"
#include "kinematics/ik.h"

#include <math.h>

#define LINE_PI     3.14159265358979323846
#define LINE_DEG2RAD (LINE_PI / 180.0)
#define LINE_EPS    1e-12

/* line_pose_to_matrix：位姿(deg) -> 4x4 齐次矩阵（行主序）。
 * 姿态按 ZYX 欧拉角(roll-pitch-yaw)解释：pose6[3]=Rx(绕X), [4]=Ry(绕Y), [5]=Rz(绕Z)，
 * 矩阵 R = Rz(Rz)·Ry(Ry)·Rx(Rx)，与 dh_pose_to_xyz_rpy 输出顺序 [roll,pitch,yaw] 一致。 */
void line_pose_to_matrix(const double pose6[6], double m[4][4])
{
    double rx = pose6[3] * LINE_DEG2RAD;   /* roll  about X */
    double ry = pose6[4] * LINE_DEG2RAD;   /* pitch about Y */
    double rz = pose6[5] * LINE_DEG2RAD;   /* yaw   about Z */
    double crx = cos(rx), srx = sin(rx);
    double cry = cos(ry), sry = sin(ry);
    double crz = cos(rz), srz = sin(rz);

    m[0][0] = crz * cry;
    m[0][1] = crz * sry * srx - srz * crx;
    m[0][2] = crz * sry * crx + srz * srx;
    m[1][0] = srz * cry;
    m[1][1] = srz * sry * srx + crz * crx;
    m[1][2] = srz * sry * crx - crz * srx;
    m[2][0] = -sry;
    m[2][1] = cry * srx;
    m[2][2] = cry * crx;
    m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0;
    m[0][3] = pose6[0];
    m[1][3] = pose6[1];
    m[2][3] = pose6[2];
    m[3][3] = 1.0;
}

/* line_count_for_distance：按步长折算插补点数 */
int line_count_for_distance(double dist_mm, double step_mm)
{
    int count;

    if (step_mm <= LINE_EPS || dist_mm <= LINE_EPS) {
        return 2;
    }
    count = (int)(dist_mm / step_mm);
    if ((double)count * step_mm < dist_mm - 1e-9) {
        count++;                              /* 向上取整，保证覆盖整段 */
    }
    count += 1;                               /* 点数 = 段数 + 1 */
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;
    return count;
}

/* line_wrap_delta：角度增量归一化到 (-180, 180]，保证姿态走最短路径。
 * 作用：起终点 RPY 若为 ±180° 等价角（如 Rx/Rz = 180 与 -180 表示同一姿态），
 * 直接线性插值会得到 -360° 的"整圈翻滚"，使中间点腕部翻转、逆解越限位；
 * 归一化后该增量为 0，姿态保持不变。 */
static double line_wrap_delta(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

/* line_plan：位置线性插值 + 姿态按最短路径线性过渡的直线段离散 */
int line_plan(const double start_pose[6], const double end_pose[6],
              int count, LinePath *path)
{
    double delta[6];
    int i, j;

    if (start_pose == NULL || end_pose == NULL || path == NULL) {
        return -1;
    }
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;

    for (j = 0; j < 6; j++) {
        delta[j] = end_pose[j] - start_pose[j];
        if (j >= 3) {
            delta[j] = line_wrap_delta(delta[j]);   /* 姿态走最短路径 */
        }
    }

    path->count = count;
    for (i = 0; i < count; i++) {
        double t = (double)i / (double)(count - 1);
        for (j = 0; j < 6; j++) {
            path->pose[i][j] = start_pose[j] + delta[j] * t;
        }
    }
    return 0;
}

/* line_solve：逐点 IK + 分支连续选解 */
int line_solve(const LinePath *path, const DhParam *dh, const JointLimit *limits,
               const double *start_joints, double (*q_out)[6], int *fail_idx)
{
    double prev[6];
    double sols[IK_MAX_SOLUTIONS][6];
    double unwrapped[IK_MAX_SOLUTIONS][6];
    double filtered[IK_MAX_SOLUTIONS][6];
    int i, j, cnt, n;

    if (path == NULL || dh == NULL || q_out == NULL || path->count < 2) {
        return -1;
    }

    for (j = 0; j < 6; j++) {
        prev[j] = (start_joints != NULL) ? start_joints[j] : 0.0;
    }

    for (i = 0; i < path->count; i++) {
        double m[4][4];

        line_pose_to_matrix(path->pose[i], m);
        cnt = ik_solve(dh, m, sols);
        if (cnt <= 0) {                       /* 该点运动学无解（超出工作空间/奇异） */
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        /* 分支连续：候选解先按上一点去卷绕（±180° 等价角归一化），
         * 再按真实软限位筛选，最后取变化量最小者 */
        n = ik_unwrap_solutions(sols, cnt, prev, unwrapped);
        n = ik_filter_by_limits(unwrapped, n, limits, filtered);
        if (n <= 0) {                         /* 候选解全部越软限位 */
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        if (ik_select_best_continuous(filtered, n, prev, NULL, q_out[i]) != 0) {
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        for (j = 0; j < 6; j++) {
            prev[j] = q_out[i][j];
        }
    }
    return 0;
}

/* line_time_table：按关节限速生成等间隔分段时间表 */
int line_time_table(const double (*q_seq)[6], int count, const double vmax_joint[6],
                    double *seg_dt, double *dt_total)
{
    double dt = 0.0;
    int i, j;

    if (q_seq == NULL || seg_dt == NULL || count < 2) {
        return -1;
    }
    for (j = 0; j < 6; j++) {
        if (vmax_joint == NULL || vmax_joint[j] <= 0.0) {
            return -1;
        }
    }

    /* 各段所需最短时间取最大者作为统一段长：
     * 统一段长 => 末端沿直线匀速；取最大 => 任何关节角速度都不超限 */
    for (i = 0; i < count - 1; i++) {
        for (j = 0; j < 6; j++) {
            double need = fabs(q_seq[i + 1][j] - q_seq[i][j]) / vmax_joint[j];
            if (need > dt) dt = need;
        }
    }

    for (i = 0; i < count - 1; i++) {
        seg_dt[i] = dt;
    }
    if (dt_total != NULL) {
        *dt_total = dt * (double)(count - 1);
    }
    return 0;
}
