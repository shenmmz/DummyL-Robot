

#include "kinematics/dh.h"

#include <math.h>

DhParam DH_TABLE[DH_JOINT_COUNT] = {
    /* theta_offset, d, a, alpha
     * ⚠️ 第 3 行肘部偏置 52 写在【a 列】(a3=-52)：沿 x3 ⇒ 与大臂共面（= DUMMY_T 口径）。
     *    若改回 d 列 (d3=52) 就是"出平面"档，home 正解 Y 差 52、Z 差 52（合计 73.539mm）。
     *    ik.c 的肘三角必须配套用 l_ew=hypot(a3,d4) / beta=atan2(-a3,d4)。 */
    { 0.0,                  140.0,   35.0,  -1.5707963267948966 },
    {-1.5707963267948966,     0.0,  146.0,   0.0                },
    { 1.5707963267948966,     0.0,  -52.0,   1.5707963267948966 },
    { 0.0,                  115.0,    0.0,  -1.5707963267948966 },
    { 0.0,                    0.0,    0.0,   1.5707963267948966 },
    { 0.0,                   91.5,    0.0,   0.0                },
};


void dh_set_tool_length(double tool_mm)
{
    DH_TABLE[5].d = 91.5 + tool_mm;
}

/* 单个关节的标准 DH 齐次变换：Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)。 */
void dh_transform(const DhParam *p, double theta_rad, double t[4][4])
{
    double ct = cos(theta_rad), st = sin(theta_rad);
    double ca = cos(p->alpha),   sa = sin(p->alpha);

    t[0][0] = ct;             t[0][1] = -st * ca;    t[0][2] = st * sa;    t[0][3] = p->a * ct;
    t[1][0] = st;             t[1][1] = ct * ca;     t[1][2] = -ct * sa;   t[1][3] = p->a * st;
    t[2][0] = 0.0;            t[2][1] = sa;          t[2][2] = ca;         t[2][3] = p->d;
    t[3][0] = 0.0;            t[3][1] = 0.0;         t[3][2] = 0.0;        t[3][3] = 1.0;
}

/* 4x4 齐次矩阵相乘 out = a*b（仅本文件用）。 */
static void mat4_mul(const double a[4][4], const double b[4][4], double out[4][4])
{
    int i, j, k;
    double r[4][4];
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            r[i][j] = 0.0;
            for (k = 0; k < 4; k++) {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            out[i][j] = r[i][j];
        }
    }
}

/* 六轴正解：关节角（度，机械角）→ 末端位姿矩阵。实测单次 <40us，不是性能瓶颈。 */
void dh_forward(const DhParam *params, const double *joints_deg, double pose[4][4])
{
    double t[4][4];
    double acc[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    int i;

    for (i = 0; i < DH_JOINT_COUNT; i++) {
        double theta = joints_deg[i] * (3.14159265358979323846 / 180.0) + params[i].theta_offset;
        dh_transform(&params[i], theta, t);
        mat4_mul(acc, t, acc);
    }
    for (i = 0; i < 4; i++) {
        int j;
        for (j = 0; j < 4; j++) {
            pose[i][j] = acc[i][j];
        }
    }
}

/* 从位姿矩阵拆出 XYZ + RPY（度），RPY 约定 Rz(yaw)*Ry(pitch)*Rx(roll)。 */
void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3])
{
    xyz[0] = pose[0][3];
    xyz[1] = pose[1][3];
    xyz[2] = pose[2][3];

    rpy[1] = atan2(-pose[2][0], sqrt(pose[0][0] * pose[0][0] + pose[1][0] * pose[1][0]));
    if (fabs(rpy[1] - 3.14159265358979323846 / 2.0) < DH_EPS ||
        fabs(rpy[1] + 3.14159265358979323846 / 2.0) < DH_EPS) {
        rpy[0] = 0.0;
        rpy[2] = atan2(pose[0][1], pose[1][1]);
    } else {
        rpy[0] = atan2(pose[2][1] / cos(rpy[1]), pose[2][2] / cos(rpy[1]));
        rpy[2] = atan2(pose[1][0] / cos(rpy[1]), pose[0][0] / cos(rpy[1]));
    }
}
