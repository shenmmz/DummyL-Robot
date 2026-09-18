/*
 * test_dh_verify —— DH 参数表与正运动学不变式（离线）
 *
 * 分两类，意义不同：
 *   A. 【结构不变式】纯数学，任何时候都必须成立
 *      - 每个 dh_transform 都必须是刚体变换：R 正交、det(R)=1、末行 0 0 0 1
 *      - 装工具长度 L，TCP 必须沿法兰 z 轴精确平移 L（不多不少、不偏方向）
 *   B. 【实机交叉验证】用真机回零后的实测位姿反查 DH 模型对不对
 *      - 这是唯一能把"模型算的"和"机器真的到的"对起来的测试，
 *        DH 表被改错（典型：d6 曾误写 183 = 2×91.5）时它会立刻炸。
 *
 * 运行：cmake --build build --target test_dh_verify && build/bin/test_dh_verify.exe
 */

#include <stdio.h>
#include <math.h>
#include "kinematics/dh.h"

/* -std=c11（非 gnu11）下 math.h 不暴露 M_PI，自己定义 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-46s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

/* 期待的 DH 设计值（来自 dh_params.h 头的参数来源说明，单位 mm / 度） */
static const double kA[6]     = { 35.0, 146.0,   0.0,   0.0,  0.0,   0.0 };
static const double kD[6]     = {140.0,   0.0,  52.0, 115.0,  0.0,  91.5 };
static const double kAlpha[6] = {-90.0,   0.0,  90.0, -90.0, 90.0,   0.0 };
static const double kOffset[6]= {  0.0, -90.0,  90.0,   0.0,  0.0,   0.0 };

int main(void)
{
    char buf[128];
    int j, i, k;
    double T[4][4], T0[4][4], TL[4][4];
    double q_zero[6] = {0, 0, 0, 0, 0, 0};
    const double L = 63.0;

    printf("=== A1. DH 表是否还是设计值 ===\n");
    for (j = 0; j < 6; j++) {
        int ok = (fabs(DH_TABLE[j].a - kA[j]) < 1e-6) &&
                 (fabs(DH_TABLE[j].d - kD[j]) < 1e-6) &&
                 (fabs(DH_TABLE[j].alpha - kAlpha[j] * M_PI / 180.0) < 1e-9) &&
                 (fabs(DH_TABLE[j].theta_offset - kOffset[j] * M_PI / 180.0) < 1e-9);
        snprintf(buf, sizeof buf, "a=%.1f d=%.1f alpha=%.0f° off=%.0f°",
                 DH_TABLE[j].a, DH_TABLE[j].d,
                 DH_TABLE[j].alpha * 180.0 / M_PI,
                 DH_TABLE[j].theta_offset * 180.0 / M_PI);
        { char t[64]; snprintf(t, sizeof t, "J%d 设计值", j + 1); check(t, ok, buf); }
    }

    printf("\n=== A2. dh_transform 必须是刚体变换 ===\n");
    dh_set_tool_length(0.0);
    {
        int all_ok = 1;
        for (j = 0; j < 6; j++) {
            double th, t[4][4], det = 0.0;
            for (int s = 0; s < 8; s++) {
                th = -M_PI + s * (2.0 * M_PI / 8.0);
                dh_transform(&DH_TABLE[j], th, t);
                /* R^T R = I */
                for (i = 0; i < 3; i++) {
                    for (k = 0; k < 3; k++) {
                        double v = 0.0;
                        for (int m = 0; m < 3; m++) v += t[m][i] * t[m][k];
                        double want = (i == k) ? 1.0 : 0.0;
                        if (fabs(v - want) > 1e-9) all_ok = 0;
                    }
                }
                /* det(R) = 1 */
                det = t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1])
                    - t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0])
                    + t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0]);
                if (fabs(det - 1.0) > 1e-9) all_ok = 0;
                /* 末行 */
                if (fabs(t[3][0]) > 1e-12 || fabs(t[3][1]) > 1e-12 ||
                    fabs(t[3][2]) > 1e-12 || fabs(t[3][3] - 1.0) > 1e-12) all_ok = 0;
            }
        }
        check("6 轴 × 8 个角度：正交 / det=1 / 末行", all_ok, NULL);
    }

    printf("\n=== A3. 工具长度：TCP 必须沿法兰 z 轴精确平移 L ===\n");
    {
        double q[6] = {25.0, -20.0, 110.0, 40.0, -30.0, 55.0};
        double p0[3], pL[3], zaxis[3], d[3], len, dot;
        dh_set_tool_length(0.0);
        dh_forward(DH_TABLE, q, T0);
        dh_set_tool_length(L);
        dh_forward(DH_TABLE, q, TL);
        dh_set_tool_length(0.0);

        for (i = 0; i < 3; i++) {
            p0[i] = T0[i][3];
            pL[i] = TL[i][3];
            zaxis[i] = T0[i][2];          /* 法兰 z 轴 = R 的第 3 列 */
            d[i] = pL[i] - p0[i];
        }
        len = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        dot = (d[0] * zaxis[0] + d[1] * zaxis[1] + d[2] * zaxis[2]) / (len > 0 ? len : 1);

        snprintf(buf, sizeof buf, "位移 %.6f mm（应为 %.3f）", len, L);
        check("平移量 == 工具长", fabs(len - L) < 1e-6, buf);
        snprintf(buf, sizeof buf, "cos=%.9f（应为 1）", dot);
        check("方向与法兰 z 轴一致", fabs(dot - 1.0) < 1e-9, buf);
    }

    printf("\n=== B. 实机交叉验证：回零位姿 ===\n");
    printf("  基准：2026-09-18 真机 `home` 后 getpos 实测\n");
    printf("        关节 {-0.04, 0.01, 89.96, 0.01, 0.03, 0.00}\n");
    printf("        位姿 X=241.56  Y=51.84  Z=286.04\n");
    {
        double qh[6] = {0.0, 0.0, 90.0, 0.0, 0.0, 0.0};
        double xyz[3], rpy[3];
        const double want[3] = {241.56, 51.84, 286.04};
        int ok;
        dh_forward(DH_TABLE, qh, T);
        dh_pose_to_xyz_rpy(T, xyz, rpy);
        snprintf(buf, sizeof buf, "模型 X=%.2f Y=%.2f Z=%.2f", xyz[0], xyz[1], xyz[2]);
        ok = (fabs(xyz[0] - want[0]) < 1.0) &&
             (fabs(xyz[1] - want[1]) < 1.0) &&
             (fabs(xyz[2] - want[2]) < 1.0);
        check("FK(home) 与真机实测一致（±1mm）", ok, buf);
    }
    {
        /* 第二个实机锚点（画线起点 150,2,114）放到 test_z170，
         * 那里用完整的 line_plan → line_solve → FK 链路来验，比单点更有意义。 */
        (void)q_zero;
    }

    printf("\n%s（失败项 %d）\n", g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
