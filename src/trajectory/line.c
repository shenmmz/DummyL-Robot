/*
 * line.c —— 笛卡尔直线插补（moveL 轨迹层）
 * ------------------------------------------------------------
 * 所属模块：轨迹规划（trajectory）
 * 对外接口：line_pose_to_matrix、line_count_for_distance、line_plan、
 *           line_solve、line_time_table
 * 依赖模块：kinematics（dh_params / dh / ik）
 *
 * 姿态插值采用四元数 SLERP（球面线性插值），避免 RPY 线性插值在
 * ±180° 附近的跳变和万向节死锁问题，确保中间位姿姿态连续。
 */

#include "trajectory/line.h"
#include "kinematics/ik.h"

#include <math.h>

#define LINE_PI     3.14159265358979323846
#define LINE_DEG2RAD (LINE_PI / 180.0)
#define LINE_EPS    1e-12

/* ---------- 四元数工具 ---------- */

/* q = (w, x, y, z)，Hamilton 乘积 */
static void quat_mul(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* 欧拉角(ZYX: roll=X, pitch=Y, yaw=Z) 度 -> 四元数 */
static void euler_to_quat(double roll_deg, double pitch_deg, double yaw_deg, double q[4])
{
    double roll = roll_deg * LINE_DEG2RAD;
    double pitch = pitch_deg * LINE_DEG2RAD;
    double yaw = yaw_deg * LINE_DEG2RAD;
    double cr = cos(roll/2), sr = sin(roll/2);
    double cp = cos(pitch/2), sp = sin(pitch/2);
    double cy = cos(yaw/2), sy = sin(yaw/2);

    /* q = q_yaw * q_pitch * q_roll */
    double q_roll[4] = {cr, sr, 0, 0};
    double q_pitch[4] = {cp, 0, sp, 0};
    double q_yaw[4]  = {cy, 0, 0, sy};
    double tmp[4];
    quat_mul(q_pitch, q_roll, tmp);
    quat_mul(q_yaw, tmp, q);
}

/* 四元数 -> 欧拉角(ZYX) 度 */
static void quat_to_euler(const double q[4], double *roll_deg, double *pitch_deg, double *yaw_deg)
{
    double roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
    double pitch = asin(2*(q[0]*q[2] - q[1]*q[3]));
    double yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
    *roll_deg = roll / LINE_DEG2RAD;
    *pitch_deg = pitch / LINE_DEG2RAD;
    *yaw_deg = yaw / LINE_DEG2RAD;
}

/* SLERP：四元数球面线性插值，t∈[0,1] */
static void slerp(const double q1[4], const double q2[4], double t, double out[4])
{
    double qq2[4] = {q2[0], q2[1], q2[2], q2[3]};
    double cosom = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    if (cosom < 0) {
        cosom = -cosom;
        qq2[0] = -qq2[0]; qq2[1] = -qq2[1]; qq2[2] = -qq2[2]; qq2[3] = -qq2[3];
    }
    if (cosom > 0.9995) {
        out[0] = q1[0] + t*(qq2[0]-q1[0]);
        out[1] = q1[1] + t*(qq2[1]-q1[1]);
        out[2] = q1[2] + t*(qq2[2]-q1[2]);
        out[3] = q1[3] + t*(qq2[3]-q1[3]);
        double len = sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3]);
        out[0]/=len; out[1]/=len; out[2]/=len; out[3]/=len;
        return;
    }
    double omega = acos(cosom);
    double sinom = sin(omega);
    double s1 = sin((1-t)*omega) / sinom;
    double s2 = sin(t*omega) / sinom;
    out[0] = s1*q1[0] + s2*qq2[0];
    out[1] = s1*q1[1] + s2*qq2[1];
    out[2] = s1*q1[2] + s2*qq2[2];
    out[3] = s1*q1[3] + s2*qq2[3];
}

/* ---------- 直线插补 ---------- */

/* line_pose_to_matrix：位姿(deg) -> 4x4 齐次矩阵（行主序）。
 * 姿态按 ZYX 欧拉角(roll-pitch-yaw)解释：pose6[3]=Rx(绕X), [4]=Ry(绕Y), [5]=Rz(绕Z)，
 * 矩阵 R = Rz(Rz)·Ry(Ry)·Rx(Rx)，与 dh_pose_to_xyz_rpy 输出顺序 [roll,pitch,yaw] 一致。 */
void line_pose_to_matrix(const double pose6[6], double m[4][4])
{
    double rx = pose6[3] * LINE_DEG2RAD;
    double ry = pose6[4] * LINE_DEG2RAD;
    double rz = pose6[5] * LINE_DEG2RAD;
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
        count++;
    }
    count += 1;
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;
    return count;
}

/* line_plan：位置线性插值 + 姿态 SLERP 球面插值的直线段离散 */
int line_plan(const double start_pose[6], const double end_pose[6],
               int count, LinePath *path)
{
    double q_start[4], q_end[4], q_interp[4];
    double pose_interp[6];
    double t;
    int i, j;

    if (start_pose == NULL || end_pose == NULL || path == NULL) {
        return -1;
    }
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;

    euler_to_quat(start_pose[3], start_pose[4], start_pose[5], q_start);
    euler_to_quat(end_pose[3], end_pose[4], end_pose[5], q_end);

    path->count = count;
    for (i = 0; i < count; i++) {
        t = (double)i / (double)(count - 1);
        for (j = 0; j < 3; j++) {
            pose_interp[j] = start_pose[j] + (end_pose[j] - start_pose[j]) * t;
        }
        slerp(q_start, q_end, t, q_interp);
        quat_to_euler(q_interp, &pose_interp[3], &pose_interp[4], &pose_interp[5]);
        for (j = 0; j < 6; j++) {
            path->pose[i][j] = pose_interp[j];
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
        if (cnt <= 0) {
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        n = ik_unwrap_solutions(sols, cnt, prev, unwrapped);
        n = ik_filter_by_limits(unwrapped, n, limits, filtered);
        if (n <= 0) {
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
