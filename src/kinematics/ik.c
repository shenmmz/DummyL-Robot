/*
 * ik.c —— 六轴机械臂解析逆运动学求解（IK）
 * ------------------------------------------------------------
 * 所属模块：运动学（kinematics）
 * 对外接口：ik_solve、ik_filter_by_limits、ik_select_best
 * 依赖模块：kinematics/dh
 */

#include "kinematics/ik.h"
#include "kinematics/dh.h"

#include <math.h>
#include <string.h>

#define IK_PI      3.14159265358979323846
#define IK_EPS     1e-10
#define IK_DEG2RAD (IK_PI / 180.0)

/* 归一化到 (-PI, PI] */
static double norm_angle(double a)
{
    while (a > IK_PI)  a -= 2.0 * IK_PI;
    while (a <= -IK_PI) a += 2.0 * IK_PI;
    return a;
}

/* 单关节旋转部分 Rz(theta) */
static void rz(double theta, double r[3][3])
{
    double c = cos(theta), s = sin(theta);
    r[0][0] = c;  r[0][1] = -s; r[0][2] = 0.0;
    r[1][0] = s;  r[1][1] = c;  r[1][2] = 0.0;
    r[2][0] = 0.0; r[2][1] = 0.0; r[2][2] = 1.0;
}

/* 3x3 矩阵乘法：out = a * b */
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

/* 3x3 矩阵转置：out = a^T */
static void r_transpose(const double a[3][3], double out[3][3])
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[i][j] = a[j][i];
        }
    }
}

/* 旋转部分 R0_i（theta 数组含 offset，rad） */
static void build_r0i(const DhParam *params, const double *theta_rad, int i, double out[3][3])
{
    double acc[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    int k;
    for (k = 0; k <= i; k++) {
        double r[3][3], tz[3][3], tx[3][3];
        double ca = cos(params[k].alpha), sa = sin(params[k].alpha);
        /* 标准 DH 旋转: Rz(θ) * Rx(α) */
        rz(theta_rad[k], r);
        tz[0][0] = 1.0; tz[0][1] = 0.0;  tz[0][2] = 0.0;
        tz[1][0] = 0.0; tz[1][1] = ca;   tz[1][2] = -sa;
        tz[2][0] = 0.0; tz[2][1] = sa;   tz[2][2] = ca;
        r_mul(r, tz, tx);
        r_mul(acc, tx, acc);
    }
    memcpy(out, acc, sizeof(acc));
}

/* 目标位姿的旋转部分与腕心：
 * w = p - d6 * a（a 为目标 z 轴向量，d6 为末端偏距） */
static void target_wrist(const double pose[4][4], double d6, double w[3])
{
    w[0] = pose[0][3] - d6 * pose[0][2];
    w[1] = pose[1][3] - d6 * pose[1][2];
    w[2] = pose[2][3] - d6 * pose[2][2];
}

/* ik_solve：解析逆运动学，目标位姿 -> 最多 8 组关节角解（肩/肘/腕各双解） */
int ik_solve(const DhParam *params, const double pose[4][4],
             double solutions[IK_MAX_SOLUTIONS][6])
{
    double a1 = params[0].a;
    double d1 = params[0].d;
    double a2 = params[1].a;
    double d3 = params[2].d;
    double d4 = params[3].d;
    double d6 = params[5].d;
    double w[3];
    double r, d_horiz, Y;
    double theta1, theta3, theta2;
    int shoulder, elbow, wrist;
    int count = 0;

    if (a2 <= IK_EPS || d4 <= IK_EPS) {
        return 0; /* 需 a2/d4 非零，实机 DH 确认前无解 */
    }

    target_wrist(pose, d6, w);   /* w = p4 = 关节4 原点（腕心） */
    r = sqrt(w[0] * w[0] + w[1] * w[1]);
    Y = w[2] - d1;               /* 腕心相对基座的高度余量（竖直分量） */
    if (r <= fabs(d3) + IK_EPS) {
        return 0; /* 奇异：腕心水平距离不足以容纳 d3 偏距 */
    }
    /* 标准 DH（α1=-90°）几何（θ_i = q_i + offset）：
     *   p4x = c1*(a1 + a2c2 + d4s23) - d3*s1
     *   p4y = s1*(a1 + a2c2 + d4s23) + d3*c1
     *   p4z = d1 - a2*s2 + d4*c23
     * 令 r = |p4_xy|，U = ±sqrt(r^2 - d3^2)（肩部两解），P = U - a1：
     *   θ1 = atan2(-d3*wx + U*wy, U*wx + d3*wy)
     *   s3 = (P^2 + Y^2 - a2^2 - d4^2) / (2 a2 d4)   （肘部由 c3=±sqrt(1-s3²) 双解）
     *   c2 = (A*P + B*Y) / (A^2 + B^2)，s2 = (B*P - A*Y) / (A^2 + B^2)
     *   A = a2 + d4*s3，B = d4*c3 */
    d_horiz = sqrt(r * r - d3 * d3);

    for (shoulder = 0; shoulder < 2; shoulder++) {
        /* shoulder=0: U = +d_horiz；shoulder=1: U = -d_horiz（肩部翻转） */
        double U = (shoulder == 0) ? d_horiz : -d_horiz;
        double P = U - a1;
        double s3v, c3abs;
        theta1 = atan2(-d3 * w[0] + U * w[1], U * w[0] + d3 * w[1]);

        s3v = (P * P + Y * Y - a2 * a2 - d4 * d4) / (2.0 * a2 * d4);
        if (s3v < -1.0 - 1e-9 || s3v > 1.0 + 1e-9) {
            continue; /* 该肩部姿态下目标超出肘部可达范围，无解（不 clamp 成伪解） */
        }
        if (s3v < -1.0) s3v = -1.0;
        if (s3v > 1.0)  s3v = 1.0;
        c3abs = sqrt(1.0 - s3v * s3v);

        for (elbow = 0; elbow < 2; elbow++) {
            double A, B, D0, c2, s2;
            double c3 = (elbow == 0) ? c3abs : -c3abs;
            theta3 = atan2(s3v, c3);

            /* 由 T0_3 平移 x/y 反解 θ2（α1=-90° 推导）：
             *   P = a2c2 + d4s23 = (a2 + d4s3)c2 + d4c3*s2
             *   Y = -a2s2 + d4c23 = d4c3*c2 - (a2 + d4s3)s2
             * 令 A = a2 + d4*s3，B = d4*c3，D0 = A^2 + B^2 > 0 */
            A = a2 + d4 * s3v;
            B = d4 * c3;
            D0 = A * A + B * B;
            if (D0 < IK_EPS) {
                continue; /* 退化（a2/d4 均为 0） */
            }
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
                int ok = 1;

                theta_rad[0] = theta1;
                theta_rad[1] = theta2;
                theta_rad[2] = theta3;
                build_r0i(params, theta_rad, 2, r03);

                /* R3_6 = R0_3^T * R_target */
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

                /* 球腕解算（标准 DH：α4=-90°，α5=+90°，a4=a5=a6=0）：
                 * R3_6 = Rz(θ4)·Rx(-90°)·Rz(θ5)·Rx(+90°)·Rz(θ6)
                 *      = [c4c5c6 - s4s6,  -c4c5s6 - s4c6,  c4s5;
                 *         s4c5c6 + c4s6,  -s4c5s6 + c4c6,  s4s5;
                 *         -s5c6,           s5s6,            c5]
                 * 对应关系：r13=+c4s5, r23=+s4s5, r31=-s5c6,
                 *           r32=+s5s6, r33=c5
                 * 双解：解A θ5∈(0,π)（sinθ5=+√…）；解B θ5∈(-π,0)（sinθ5=-√…）。
                 * θ4 = atan2(r23/sinθ5, r13/sinθ5)（除以 sinθ5 以兼容双解），
                 * θ6 用 θ4 旋转消去 c4/s4 后 atan2 求解。 */
                sinth5 = (wrist == 0) ? sqrt(1.0 - r36[2][2] * r36[2][2])
                                      : -sqrt(1.0 - r36[2][2] * r36[2][2]);
                r13 = r36[0][2];
                r23 = r36[1][2];
                r11 = r36[0][0];
                r12 = r36[0][1];
                r21 = r36[1][0];
                r22 = r36[1][1];
                if (fabs(sinth5) < IK_EPS) {
                    ok = 0; /* 腕奇异：θ4 与 θ6 同轴 */
                } else {
                    theta5 = atan2(sinth5, r36[2][2]);
                    theta4 = atan2(r23 / sinth5, r13 / sinth5);
                    theta6 = atan2(-sin(theta4) * r11 + cos(theta4) * r21,
                                   -sin(theta4) * r12 + cos(theta4) * r22);
                }

                if (ok) {
                    /* 输出关节角 q = theta - theta_offset（FK 会再加回） */
                    int k;
                    double th[6] = {theta1, theta2, theta3, theta4, theta5, theta6};
                    for (k = 0; k < 6; k++) {
                        double q = norm_angle(th[k] - params[k].theta_offset) / IK_DEG2RAD;
                        sol[k] = q;
                    }
                    memcpy(solutions[count], sol, sizeof(sol));
                    count++;
                }
            }
        }
    }
    return count;
}

/* ik_filter_by_limits：按关节限位过滤候选解，返回剩余解数 */
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

/* ik_select_best：按加权关节距离选最优解，结果写回 best，失败返回 -1 */
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
