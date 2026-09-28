
#include "kinematics/ik.h"
#include "kinematics/dh.h"

#include <math.h>
#include <string.h>

#define IK_PI      3.14159265358979323846
#define IK_EPS     1e-10
#define IK_WRIST_SINGULAR_SIN  1e-7
#define IK_DEG2RAD (IK_PI / 180.0)

/* 角度归一化到 (-pi, pi]（仅本文件用）。 */
static double norm_angle(double a)
{
    while (a > IK_PI)  a -= 2.0 * IK_PI;
    while (a <= -IK_PI) a += 2.0 * IK_PI;
    return a;
}

/* 绕 Z 轴的 3x3 旋转矩阵（仅本文件用）。 */
static void rz(double theta, double r[3][3])
{
    double c = cos(theta), s = sin(theta);
    r[0][0] = c;  r[0][1] = -s; r[0][2] = 0.0;
    r[1][0] = s;  r[1][1] = c;  r[1][2] = 0.0;
    r[2][0] = 0.0; r[2][1] = 0.0; r[2][2] = 1.0;
}

/* 3x3 矩阵相乘 out = a*b（仅本文件用）。 */
static void r_mul(const double a[3][3], const double b[3][3], double out[3][3])
{
    int i, j, k;
    double r[3][3];
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            r[i][j] = 0.0;
            for (k = 0; k < 3; k++) {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[i][j] = r[i][j];
        }
    }
}

/* 3x3 转置（仅本文件用）。 */
static void r_transpose(const double a[3][3], double out[3][3])
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[i][j] = a[j][i];
        }
    }
}

/* 累积求出第 i 关节到基座的旋转 R0i（仅本文件用）。 */
static void build_r0i(const DhParam *params, const double *theta_rad, int i, double out[3][3])
{
    double acc[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    int k;
    for (k = 0; k <= i; k++) {
        double r[3][3], tz[3][3], tx[3][3];
        double ca = cos(params[k].alpha), sa = sin(params[k].alpha);
        rz(theta_rad[k], r);
        tz[0][0] = 1.0; tz[0][1] = 0.0;  tz[0][2] = 0.0;
        tz[1][0] = 0.0; tz[1][1] = ca;   tz[1][2] = -sa;
        tz[2][0] = 0.0; tz[2][1] = sa;   tz[2][2] = ca;
        r_mul(r, tz, tx);
        r_mul(acc, tx, acc);
    }
    memcpy(out, acc, sizeof(acc));
}

/* 由末端位姿沿末端法线回退 d6，得到腕心位置（仅本文件用）。 */
static void target_wrist(const double pose[4][4], double d6, double w[3])
{
    w[0] = pose[0][3] - d6 * pose[0][2];
    w[1] = pose[1][3] - d6 * pose[1][2];
    w[2] = pose[2][3] - d6 * pose[2][2];
}

/* IK 求解状态码 → 中文说明。 */
const char *ik_sol_status_str(IkSolStatus s)
{
    switch (s) {
    case IK_SOL_VALID:        return "有效";
    case IK_SOL_OUT_OF_REACH: return "超可达";
    case IK_SOL_SINGULAR:     return "奇异";
    case IK_SOL_DEGENERATE:   return "退化";
    default:                  return "未知";
    }
}

/* 六轴 IK 主解算（解析法），最多 8 组解 = 肩 2 x 肘 2 x 腕 2。
 * ref_joints 只用于【腕奇异】分支定 theta4（其余分支不用）。
 * 返回解个数；info[] 逐位标注肩/肘/腕状态（VALID / SINGULAR / DEGENERATE / OUT_OF_REACH）。
 * ★ 52mm 肘部偏置写在 a3（沿 x3、与大臂共面），与 dh.c 的 DH_TABLE[2] 一致。
 *   肩：θ1 用 atan2(−d3·wx + U·wy, U·wx + d3·wy)，U = ±√(r²−d3²)。
 *       共面档 d3=0 ⇒ 自动退化成 atan2(wy,wx) 与 atan2(−wy,−wx) 两组。
 *   肘：平面投影里两条边是 a2=146 与 l_ew=√(a3²+d4²)=126.2101（52 折进长度与相位）。
 *   腕：θ4/θ5/θ6。 */
int ik_solve_ref(const DhParam *params, const double pose[4][4],
                 const double *ref_joints,
                 double solutions[IK_MAX_SOLUTIONS][6],
                 IkSolInfo info[IK_MAX_SOLUTIONS])
{
    double a1 = params[0].a;
    double d1 = params[0].d;
    double a2 = params[1].a;
    double d3 = params[2].d;
    double d4 = params[3].d;
    double d6 = params[5].d;
    /* 肘部偏置 a3（沿 x3）与 d4 合成后的【等效前臂长】与【相位】。
     * a3=0（52 写在 d3 的老档）时 l_ew=d4、beta=0 ⇒ 本函数逐字退化成老写法。 */
    double a3 = params[2].a;
    double l_ew = hypot(a3, d4);
    double beta = atan2(-a3, d4);
    double w[3];
    double r, d_horiz, Y;
    double theta1, theta3, theta2;
    int shoulder, elbow, wrist;
    int count = 0;
    int idx;
    double ref_theta4 = 0.0;
    if (ref_joints != NULL && isfinite(ref_joints[3])) {
        ref_theta4 = ref_joints[3] * IK_DEG2RAD;
    }

    if (info) {
        for (idx = 0; idx < IK_MAX_SOLUTIONS; idx++) {
            info[idx].shoulder = IK_SOL_OUT_OF_REACH;
            info[idx].elbow = IK_SOL_OUT_OF_REACH;
            info[idx].wrist = IK_SOL_OUT_OF_REACH;
            info[idx].valid = 0;
        }
    }

    if (a2 <= IK_EPS || l_ew <= IK_EPS) {
        if (info) {
            for (idx = 0; idx < IK_MAX_SOLUTIONS; idx++)
                info[idx].shoulder = IK_SOL_DEGENERATE;
        }
        return 0;
    }

    target_wrist(pose, d6, w);
    r = sqrt(w[0] * w[0] + w[1] * w[1]);
    Y = w[2] - d1;
    if (r <= fabs(d3) + IK_EPS) {
        if (info) {
            for (idx = 0; idx < IK_MAX_SOLUTIONS; idx++)
                info[idx].shoulder = IK_SOL_SINGULAR;
        }
        return 0;
    }

    d_horiz = sqrt(r * r - d3 * d3);

    for (shoulder = 0; shoulder < 2; shoulder++) {
        double U = (shoulder == 0) ? d_horiz : -d_horiz;
        double P = U - a1;
        double s3v, c3abs;
        int shoulder_reachable = 1;

        theta1 = atan2(-d3 * w[0] + U * w[1], U * w[0] + d3 * w[1]);

        s3v = (P * P + Y * Y - a2 * a2 - l_ew * l_ew) / (2.0 * a2 * l_ew);
        if (s3v < -1.0 - 1e-9 || s3v > 1.0 + 1e-9) {
            shoulder_reachable = 0;
            if (info) {
                for (elbow = 0; elbow < 2; elbow++) {
                    for (wrist = 0; wrist < 2; wrist++) {
                        idx = shoulder * 4 + elbow * 2 + wrist;
                        info[idx].shoulder = IK_SOL_OUT_OF_REACH;
                        info[idx].elbow = IK_SOL_OUT_OF_REACH;
                        info[idx].wrist = IK_SOL_OUT_OF_REACH;
                    }
                }
            }
        }
        if (!shoulder_reachable) continue;
        if (s3v < -1.0) s3v = -1.0;
        if (s3v > 1.0)  s3v = 1.0;
        c3abs = sqrt(1.0 - s3v * s3v);

        for (elbow = 0; elbow < 2; elbow++) {
            double A, B, D0, c2, s2;
            double c3 = (elbow == 0) ? c3abs : -c3abs;
            int elbow_ok = 1;

            theta3 = atan2(s3v, c3) + beta;

            A = a2 + l_ew * s3v;
            B = l_ew * c3;
            D0 = A * A + B * B;
            if (D0 < IK_EPS) {
                elbow_ok = 0;
                if (info) {
                    for (wrist = 0; wrist < 2; wrist++) {
                        idx = shoulder * 4 + elbow * 2 + wrist;
                        info[idx].shoulder = IK_SOL_VALID;
                        info[idx].elbow = IK_SOL_DEGENERATE;
                        info[idx].wrist = IK_SOL_DEGENERATE;
                    }
                }
            }
            if (!elbow_ok) continue;

            c2 = (A * P + B * Y) / D0;
            s2 = (B * P - A * Y) / D0;
            theta2 = atan2(s2, c2);

            for (wrist = 0; wrist < 2; wrist++) {
                double r03[3][3], rt[3][3], r36[3][3];
                double theta_rad[6];
                double r13, r23, r11, r12, r21, r22;
                double sinth5;
                double theta4, theta5, theta6;
                double sol[6];

                idx = shoulder * 4 + elbow * 2 + wrist;

                theta_rad[0] = theta1;
                theta_rad[1] = theta2;
                theta_rad[2] = theta3;
                build_r0i(params, theta_rad, 2, r03);

                r_transpose(r03, rt);
                {
                    double rt_r[3][3] = {{rt[0][0], rt[0][1], rt[0][2]},
                                         {rt[1][0], rt[1][1], rt[1][2]},
                                         {rt[2][0], rt[2][1], rt[2][2]}};
                    double rr[3][3] = {{pose[0][0], pose[0][1], pose[0][2]},
                                       {pose[1][0], pose[1][1], pose[1][2]},
                                       {pose[2][0], pose[2][1], pose[2][2]}};
                    r_mul(rt_r, rr, r36);
                }

                {
                    double r33sq = 1.0 - r36[2][2] * r36[2][2];
                    if (r33sq < 0.0) r33sq = 0.0;
                    sinth5 = (wrist == 0) ? sqrt(r33sq) : -sqrt(r33sq);
                }
                r13 = r36[0][2];
                r23 = r36[1][2];
                r11 = r36[0][0];
                r12 = r36[0][1];
                r21 = r36[1][0];
                r22 = r36[1][1];

                if (info) {
                    info[idx].shoulder = IK_SOL_VALID;
                    info[idx].elbow = IK_SOL_VALID;
                }

                if (fabs(sinth5) < IK_WRIST_SINGULAR_SIN) {
                    theta5 = atan2(sinth5, r36[2][2]);
                    theta4 = ref_theta4;
                    theta6 = atan2(r21, r11) - theta4;

                    {
                        int k;
                        double th[6] = {theta1, theta2, theta3, theta4, theta5, theta6};
                        for (k = 0; k < 6; k++) {
                            double q = norm_angle(th[k] - params[k].theta_offset) / IK_DEG2RAD;
                            sol[k] = q;
                        }
                        memcpy(solutions[count], sol, sizeof(sol));
                        count++;
                    }
                    if (info) {
                        info[idx].wrist = IK_SOL_SINGULAR;
                        info[idx].valid = 1;
                    }
                } else {
                    theta5 = atan2(sinth5, r36[2][2]);
                    if (sinth5 > 0.0) {
                        theta4 = atan2( r23,  r13);
                    } else {
                        theta4 = atan2(-r23, -r13);
                    }
                    theta6 = atan2(-sin(theta4) * r11 + cos(theta4) * r21,
                                   -sin(theta4) * r12 + cos(theta4) * r22);

                    {
                        int k;
                        double th[6] = {theta1, theta2, theta3, theta4, theta5, theta6};
                        for (k = 0; k < 6; k++) {
                            double q = norm_angle(th[k] - params[k].theta_offset) / IK_DEG2RAD;
                            sol[k] = q;
                        }
                        memcpy(solutions[count], sol, sizeof(sol));
                        count++;
                    }
                    if (info) {
                        info[idx].wrist = IK_SOL_VALID;
                        info[idx].valid = 1;
                    }
                }
            }
        }
    }
    return count;
}

/* = ik_solve_ref(ref_joints=NULL) + 输出 info。 */
int ik_solve_ex(const DhParam *params, const double pose[4][4],
                double solutions[IK_MAX_SOLUTIONS][6],
                IkSolInfo info[IK_MAX_SOLUTIONS])
{
    return ik_solve_ref(params, pose, NULL, solutions, info);
}

/* 最简入口：不要 info。 */
int ik_solve(const DhParam *params, const double pose[4][4],
             double solutions[IK_MAX_SOLUTIONS][6])
{
    return ik_solve_ex(params, pose, solutions, NULL);
}

/* 按关节软限位过滤候选解，返回剩下的个数。limits=NULL 表示不过滤。
 * 容差 ±1e-6°，避免边界解因浮点误差被误杀。 */
int ik_filter_by_limits(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const JointLimit *limits, double filtered[IK_MAX_SOLUTIONS][6])
{
    int i, j, n = 0;
    if (limits == NULL) {
        for (i = 0; i < candidate_cnt; i++) {
            memcpy(filtered[n++], solutions[i], sizeof(solutions[i]));
        }
        return n;
    }
    for (i = 0; i < candidate_cnt; i++) {
        int ok = 1;
        for (j = 0; j < 6; j++) {
            if (solutions[i][j] < limits[j].min_deg - 1e-6 ||
                solutions[i][j] > limits[j].max_deg + 1e-6) {
                ok = 0;
                break;
            }
        }
        if (ok) {
            memcpy(filtered[n++], solutions[i], sizeof(solutions[i]));
        }
    }
    return n;
}

/* 从候选解里挑 Σ w·(q - current)² 最小的一组，写入 best，返回 0；无候选返回 -1。
 * weights=NULL 时权重全 1（即只比总关节位移）。 */
int ik_select_best(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                   const double *current_joints, const double *weights, double best[6])
{
    int i, j;
    int best_idx = -1;
    double best_cost = 0.0;

    if (candidate_cnt <= 0) {
        return -1;
    }
    for (i = 0; i < candidate_cnt; i++) {
        double cost = 0.0;
        for (j = 0; j < 6; j++) {
            double diff = solutions[i][j] - (current_joints ? current_joints[j] : 0.0);
            double wgt = weights ? weights[j] : 1.0;
            cost += wgt * diff * diff;
        }
        if (best_idx < 0 || cost < best_cost) {
            best_idx = i;
            best_cost = cost;
        }
    }
    for (j = 0; j < 6; j++) {
        best[j] = solutions[best_idx][j];
    }
    return 0;
}


/* 把角度折进 [0, 360)。 */
double ik_wrap_deg(double deg)
{
    double r = fmod(deg + 180.0, 360.0);
    if (r <= 0.0) {
        r += 360.0;
    }
    return r - 180.0;
}

/* 取与 ref_deg 最接近的等价角（相差 360 的整数倍）。
 * 用途：±180 分支切割时 float/double 可能挑到相反的代表值（相差 360 = 50 圈），
 * 所以【比对角度必须取模或先 unwrap】，否则会误判成大跳变。 */
double ik_unwrap_near(double deg, double ref_deg)
{
    return ref_deg + ik_wrap_deg(deg - ref_deg);
}

/* 把每组解整体平移到离 ref_joints 最近的 ±360k 分支上。
 * 不做这一步的话，359° 与 -1° 明明是同一姿态，却会被当成相差 360° 的两种解。 */
int ik_unwrap_solutions(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const double *ref_joints, double out[IK_MAX_SOLUTIONS][6])
{
    int i, j, n = 0;

    if (solutions == NULL || out == NULL || candidate_cnt <= 0) {
        return 0;
    }
    for (i = 0; i < candidate_cnt && i < IK_MAX_SOLUTIONS; i++) {
        for (j = 0; j < 6; j++) {
            double ref = (ref_joints != NULL) ? ref_joints[j] : 0.0;
            out[n][j] = ik_unwrap_near(solutions[i][j], ref);
        }
        n++;
    }
    return n;
}

/* 先 unwrap 再 select_best：保证相邻插补点选到【同一分支】。
 * 直接 select_best 会在 ±180° 分支间来回跳，表现为某轴突然翻转一整圈。
 * ⚠️ 比对角度必须取模：float/double 差异会让代表值落到相反一侧。 */
int ik_select_best_continuous(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                              const double *current_joints, const double *weights, double best[6])
{
    double unwrapped[IK_MAX_SOLUTIONS][6];
    int n;

    if (candidate_cnt <= 0 || best == NULL) {
        return -1;
    }
    if (candidate_cnt > IK_MAX_SOLUTIONS) {
        candidate_cnt = IK_MAX_SOLUTIONS;
    }
    n = ik_unwrap_solutions(solutions, candidate_cnt, current_joints, unwrapped);
    return ik_select_best(unwrapped, n, current_joints, weights, best);
}
