/* ============================================================
 * 三点式空间圆弧插补（画弧线用）。
 *
 * 思路：起点 P0、途经点 Pv、终点 P2 三点定一个圆（圆心 C、半径 R、单位法向 n）。
 * 在圆所在平面建正交基 e1=(P0-C)/R、e2=n×e1，则圆上点 P(θ)=C+R(cosθ·e1+sinθ·e2)，
 * P0 对应 θ=0。算出 Pv、P2 的辐角 t1、t2，选出让 0→t1→t2 单调不回头的那条弧
 * （即"经过中间点"的那段），沿它等步长采样若干位姿点。姿态与 line.c 一致：
 * start→end 四元数 SLERP。下游 line_solve / line_time_table / 跳变闸原样复用。
 *
 * 近共线（|a×b| 相对 |a||b| 极小）时圆退化 → 转调 line_plan 走直线。
 * ============================================================ */

#include "trajectory/arc.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define ARC_PI        3.14159265358979323846
#define ARC_DEG2RAD   (ARC_PI / 180.0)
#define ARC_EPS       1e-12
/* 共线判据：|a×b| / (|a|·|b|) = sin(夹角)，小于它认为三点几乎共线 ⇒ 退化直线。 */
#define ARC_COLLINEAR_SIN 1e-4

/* ---- 四元数/欧拉角小工具：与 line.c 里的静态实现一致（同模块内不便共享 static，
 *      这里各留一份，逻辑极短且久经测试，改动时两处需同步）。 ---- */

static void quat_mul(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void euler_to_quat(double roll_deg, double pitch_deg, double yaw_deg, double q[4])
{
    double roll = roll_deg * ARC_DEG2RAD;
    double pitch = pitch_deg * ARC_DEG2RAD;
    double yaw = yaw_deg * ARC_DEG2RAD;
    double cr = cos(roll/2), sr = sin(roll/2);
    double cp = cos(pitch/2), sp = sin(pitch/2);
    double cy = cos(yaw/2), sy = sin(yaw/2);
    double q_roll[4] = {cr, sr, 0, 0};
    double q_pitch[4] = {cp, 0, sp, 0};
    double q_yaw[4]  = {cy, 0, 0, sy};
    double tmp[4];
    quat_mul(q_pitch, q_roll, tmp);
    quat_mul(q_yaw, tmp, q);
}

static double clamp_pm1(double x)
{
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

static void quat_to_euler(const double q[4], double *roll_deg, double *pitch_deg, double *yaw_deg)
{
    double roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
    double pitch = asin(clamp_pm1(2*(q[0]*q[2] - q[1]*q[3])));
    double yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
    *roll_deg = roll / ARC_DEG2RAD;
    *pitch_deg = pitch / ARC_DEG2RAD;
    *yaw_deg = yaw / ARC_DEG2RAD;
}

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

/* ---- 三维向量小工具 ---- */

static double v3_dot(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void v3_cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static double v3_norm(const double a[3])
{
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

static int pose_is_finite(const double p[6])
{
    int j;
    for (j = 0; j < 6; j++) if (!isfinite(p[j])) return 0;
    return 1;
}

int arc_plan_3pt(const double start_pose[6], const double via[3],
                 const double end_pose[6], double step_mm,
                 LinePath *path, double *out_arc_len_mm)
{
    double P0[3], Pv[3], P2[3];
    double a[3], b[3], cross[3];
    double an, bn, cn, aa, ab, bb, D;
    double u, v, C[3], R, n[3], e1[3], e2[3];
    double d1[3], d2[3], t1, t2, sense, u1, u2, sweep;
    double q_start[4], q_end[4], q_i[4];
    double arc_len, chord;
    int count, i, j;

    if (start_pose == NULL || via == NULL || end_pose == NULL || path == NULL) {
        return -1;
    }
    if (!pose_is_finite(start_pose) || !pose_is_finite(end_pose)) return -1;
    for (j = 0; j < 3; j++) if (!isfinite(via[j])) return -1;

    P0[0] = start_pose[0]; P0[1] = start_pose[1]; P0[2] = start_pose[2];
    Pv[0] = via[0];        Pv[1] = via[1];        Pv[2] = via[2];
    P2[0] = end_pose[0];   P2[1] = end_pose[1];   P2[2] = end_pose[2];

    for (j = 0; j < 3; j++) { a[j] = Pv[j] - P0[j]; b[j] = P2[j] - P0[j]; }
    an = v3_norm(a);
    bn = v3_norm(b);
    if (an < ARC_EPS || bn < ARC_EPS) {
        return -2;   /* 途经点或终点与起点重合，定不出圆 */
    }

    chord = bn;
    v3_cross(a, b, cross);
    cn = v3_norm(cross);

    /* 近共线 ⇒ 圆退化，走直线。 */
    if (cn / (an * bn) < ARC_COLLINEAR_SIN) {
        count = line_count_for_distance(chord, step_mm);
        if (line_plan(start_pose, end_pose, count, path) != 0) return -1;
        if (out_arc_len_mm != NULL) *out_arc_len_mm = chord;
        return 1;
    }

    /* 圆心：C = P0 + u·a + v·b，解 (a·a)u+(a·b)v=(a·a)/2, (a·b)u+(b·b)v=(b·b)/2。
     * D = |a×b|² = (a·a)(b·b)-(a·b)²，非零（已排除共线）。 */
    aa = v3_dot(a, a);
    ab = v3_dot(a, b);
    bb = v3_dot(b, b);
    D  = aa * bb - ab * ab;
    u  = ((aa * 0.5) * bb - (bb * 0.5) * ab) / D;
    v  = (aa * (bb * 0.5) - ab * (aa * 0.5)) / D;
    for (j = 0; j < 3; j++) C[j] = P0[j] + u * a[j] + v * b[j];

    {
        double r0[3];
        for (j = 0; j < 3; j++) r0[j] = P0[j] - C[j];
        R = v3_norm(r0);
        if (R < ARC_EPS) return -3;
        for (j = 0; j < 3; j++) e1[j] = r0[j] / R;
    }
    for (j = 0; j < 3; j++) n[j] = cross[j] / cn;   /* 单位法向（右手，a→b 正向） */
    v3_cross(n, e1, e2);                              /* 平面内与 e1 垂直的第二基 */

    /* 三点辐角（P0 恒为 0）。 */
    for (j = 0; j < 3; j++) { d1[j] = Pv[j] - C[j]; d2[j] = P2[j] - C[j]; }
    t1 = atan2(v3_dot(d1, e2), v3_dot(d1, e1));
    t2 = atan2(v3_dot(d2, e2), v3_dot(d2, e1));

    /* 选经过 Pv 的那段弧：朝 t1 的方向旋转，若 t2 在该方向上"排在 t1 之前"，
     * 就再绕一整圈把它收到 t1 之后 ⇒ 采样必然 0→t1→t2 单调不回头。 */
    sense = (t1 >= 0.0) ? 1.0 : -1.0;
    u1 = sense * t1;   /* ≥ 0 */
    u2 = sense * t2;
    if (u2 < u1) u2 += 2.0 * ARC_PI;
    sweep = sense * u2;

    arc_len = R * fabs(sweep);
    count = line_count_for_distance(arc_len, step_mm);
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;

    euler_to_quat(start_pose[3], start_pose[4], start_pose[5], q_start);
    euler_to_quat(end_pose[3],   end_pose[4],   end_pose[5],   q_end);

    path->count = count;
    for (i = 0; i < count; i++) {
        double f = (double)i / (double)(count - 1);
        double ang = sweep * f;
        double ca = cos(ang), sa = sin(ang);
        double pose[6];
        for (j = 0; j < 3; j++) pose[j] = C[j] + R * (ca * e1[j] + sa * e2[j]);
        slerp(q_start, q_end, f, q_i);
        quat_to_euler(q_i, &pose[3], &pose[4], &pose[5]);
        for (j = 0; j < 6; j++) {
            if (!isfinite(pose[j])) return -1;
            path->pose[i][j] = pose[j];
        }
    }

    if (out_arc_len_mm != NULL) *out_arc_len_mm = arc_len;
    return 0;
}
