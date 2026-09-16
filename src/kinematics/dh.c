/*
 * dh.c —— 六轴机械臂标准 DH 参数表与正运动学（FK）
 * ------------------------------------------------------------
 * 所属模块：运动学（kinematics）
 * 对外接口：dh_transform、dh_forward、dh_pose_to_xyz_rpy
 * 依赖模块：无
 */

/* ============================================================
 * 标准 DH 参数表（机械臂结构参数，单位 mm / 度）
 * ------------------------------------------------------------
 * 符号约定（见 dh_forward 实现）：theta = q*π/180 + theta_offset
 *   theta_offset 即各轴 home 角；零位 q=0 时输出机械 home 姿态。
 * 各轴含义（a=沿x的连杆长, d=沿z的连杆偏距, alpha=连杆扭转角）：
 *   J1  a=35   基座到1轴的水平X偏移   d=140 基座垂直高度      alpha=-90°
 *   J2  a=146  大臂长度               d=0                  alpha= 0°  home=-90°
 *   J3  a=0    d=52   3轴→4轴中心距(肘部)  alpha= 90°  home= 90°
 *   J4  a=0    d=115  4轴→5轴小臂长度       alpha=-90°
 *   J5  a=0    d=0                      alpha= 90°
 *   J6  a=0    d=91.5 J5→末端距离(腕长)    alpha= 0°
 * ============================================================ */

#include "kinematics/dh.h"

#include <math.h>


const DhParam DH_TABLE[DH_JOINT_COUNT] = {
    /*  a(mm)    alpha(rad)      d(mm)      theta_offset(rad) */
    { 35.0,     -1.5707963267948966,  140.0,     0.0               }, /* 关节1 */
    { 146.0,     0.0,                   0.0,    -1.5707963267948966 }, /* 关节2 home=-90° */
    { 0.0,       1.5707963267948966,   52.0,     1.5707963267948966 }, /* 关节3 home=90° */
    { 0.0,      -1.5707963267948966,  115.0,     0.0               }, /* 关节4 */
    { 0.0,       1.5707963267948966,    0.0,     0.0               }, /* 关节5 */
    { 0.0,       0.0,                 183.0,      0.0               }, /* 关节6 腕长 J5→末端实测 *//*把 91.5改成183*/
};

/* 标准 DH 单关节变换：
 * T_i = Rot(z,θ) * Trans(z,d) * Trans(x,a) * Rot(x,α) */
void dh_transform(const DhParam *p, double theta_rad, double t[4][4])
{
    double ct = cos(theta_rad), st = sin(theta_rad);
    double ca = cos(p->alpha),   sa = sin(p->alpha);

    t[0][0] = ct;             t[0][1] = -st * ca;    t[0][2] = st * sa;    t[0][3] = p->a * ct;
    t[1][0] = st;             t[1][1] = ct * ca;     t[1][2] = -ct * sa;   t[1][3] = p->a * st;
    t[2][0] = 0.0;            t[2][1] = sa;          t[2][2] = ca;         t[2][3] = p->d;
    t[3][0] = 0.0;            t[3][1] = 0.0;         t[3][2] = 0.0;        t[3][3] = 1.0;
}

/* mat4_mul：4x4 齐次矩阵乘法，out = a * b */
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

/* dh_forward：正运动学，由 6 关节角（度）累乘 DH 变换求末端位姿矩阵 */
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

/* dh_pose_to_xyz_rpy：位姿矩阵转 XYZ 平移与 ZYX 欧拉角；输出顺序 [roll(X),pitch(Y),yaw(Z)]，与命令行(Rx,Ry,Rz) 一致 */
void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3])
{
    xyz[0] = pose[0][3];
    xyz[1] = pose[1][3];
    xyz[2] = pose[2][3];

    /* ZYX 欧拉角: ry = atan2(-r31, sqrt(r11^2+r21^2)) 等 */
    rpy[1] = atan2(-pose[2][0], sqrt(pose[0][0] * pose[0][0] + pose[1][0] * pose[1][0]));
    if (fabs(rpy[1] - 3.14159265358979323846 / 2.0) < DH_EPS ||
        fabs(rpy[1] + 3.14159265358979323846 / 2.0) < DH_EPS) {
        rpy[0] = 0.0;
        rpy[2] = atan2(pose[0][1], pose[1][1]);
    } else {
        rpy[0] = atan2(pose[2][1] / cos(rpy[1]), pose[2][2] / cos(rpy[1]));  /* roll  about X */
        rpy[2] = atan2(pose[1][0] / cos(rpy[1]), pose[0][0] / cos(rpy[1]));  /* yaw   about Z */
    }
}
