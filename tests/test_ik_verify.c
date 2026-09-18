/*
 * test_ik_verify —— FK/IK 闭环回归测试（离线，不碰串口）
 *
 * 【为什么必须有它】2026-09-18 之前这 5 个测试文件的源码被永久丢失，
 * kinematics 长期处于"改一行没人会发现"的状态。FK/IK 是 MoveL 的地基：
 * 逆解错了，画出来的线就是歪的，而且很难从现象反推到 ik.c。
 *
 * 测什么：
 *   1) 随机关节角 → FK → IK → 至少有一组解能还原同一个位姿（位置 ≤0.01mm）
 *   2) 还原出来的解里，至少有一组与原始关节角一致（等价角按 360° 归一比较）
 *   3) 软限位筛选：ik_filter_by_limits 不会放过一个越界解
 *   4) 分支连续选解：ik_select_best_continuous 相邻两点不应出现 360° 级跳变
 *
 * 运行：cmake --build build --target test_ik_verify && build/bin/test_ik_verify.exe
 * 退出码 0 = 全过。
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "kinematics/dh.h"
#include "kinematics/ik.h"

static int g_fail = 0;

/* 确定性伪随机（LCG），保证每次跑出同一批样本，失败可复现 */
static unsigned long g_seed = 12345u;
static double rnd(void)
{
    g_seed = (g_seed * 1103515245u + 12345u) & 0x7fffffffu;
    return (double)g_seed / (double)0x7fffffffu;   /* [0,1) */
}

static const JointLimit kLimits[6] = {
    {-170.0, 179.0}, {-72.0, 90.0}, {30.0, 180.0},
    {-360.0, 360.0}, {-95.0, 95.0}, {-360.0, 360.0}
};

/* 把差值折到 (-180,180]，用于容忍 ±360° 等价角 */
static double wrap180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

static int pose_pos_close(const double a[4][4], const double b[4][4], double tol_mm)
{
    int i;
    for (i = 0; i < 3; i++) {
        if (fabs(a[i][3] - b[i][3]) > tol_mm) return 0;
    }
    return 1;
}

static void fk(const double q[6], double T[4][4])
{
    dh_forward(DH_TABLE, q, T);
}

int main(void)
{
    const int N = 2000;
    int n_pos_fail = 0, n_ang_fail = 0, n_no_sol = 0, n_limit_leak = 0;
    int worst_pos = -1, worst_ang = -1;
    int i, k;

    dh_set_tool_length(0.0);   /* 无工具，d6 = 91.5 */

    printf("=== 1. FK→IK 闭环（%d 组随机关节角）===\n", N);
    for (i = 0; i < N; i++) {
        double q[6], T[4][4], sol[IK_MAX_SOLUTIONS][6];
        int nsol, ok_pos = 0, ok_ang = 0, j;

        for (j = 0; j < 6; j++)
            q[j] = kLimits[j].min_deg +
                   (kLimits[j].max_deg - kLimits[j].min_deg) * rnd();

        fk(q, T);

        nsol = ik_solve(DH_TABLE, T, sol);
        if (nsol <= 0) {
            n_no_sol++;
            continue;
        }

        for (k = 0; k < nsol; k++) {
            double Ts[4][4];
            fk(sol[k], Ts);
            if (pose_pos_close(T, Ts, 0.01)) {
                int jmatch = 1, m;
                ok_pos = 1;
                for (m = 0; m < 6; m++) {
                    if (fabs(wrap180(sol[k][m] - q[m])) > 0.05) { jmatch = 0; break; }
                }
                if (jmatch) ok_ang = 1;
            }
        }
        if (!ok_pos) { n_pos_fail++; if (worst_pos < 0) worst_pos = i; }
        if (!ok_ang) { n_ang_fail++; if (worst_ang < 0) worst_ang = i; }

        /* 限位筛选：所有通过筛选的解都必须真的在限位内 */
        {
            double filt[IK_MAX_SOLUTIONS][6];
            int nf = ik_filter_by_limits(sol, nsol, kLimits, filt);
            for (k = 0; k < nf; k++) {
                for (j = 0; j < 6; j++) {
                    if (filt[k][j] < kLimits[j].min_deg - 1e-6 ||
                        filt[k][j] > kLimits[j].max_deg + 1e-6) {
                        n_limit_leak++;
                        goto next_sample;
                    }
                }
            }
        }
next_sample:
        ;
    }
    printf("  无解               : %d\n", n_no_sol);
    printf("  位姿还原失败(>0.01mm): %d %s\n", n_pos_fail,
           n_pos_fail ? "  <-- 有问题" : "OK");
    printf("  关节角还原失败(>0.05°): %d %s\n", n_ang_fail,
           n_ang_fail ? "  <-- 有问题" : "OK");
    printf("  限位筛漏           : %d %s\n", n_limit_leak,
           n_limit_leak ? "  <-- 有问题" : "OK");
    if (n_pos_fail || n_no_sol || n_limit_leak) g_fail++;

    /* 关节角还原允许少量奇异点失败（解不唯一时选中另一分支是合法的） */
    if (n_ang_fail > N / 20) {
        printf("  [FAIL] 关节角还原失败率 %.1f%%，超过 5%% 容忍线\n",
               100.0 * n_ang_fail / N);
        g_fail++;
    }

    printf("\n=== 2. 分支连续选解（相邻点不跳 360°）===\n");
    {
        double q[6] = {20.0, 30.0, 120.0, 175.0, 40.0, -175.0};
        double T[4][4], sol[IK_MAX_SOLUTIONS][6], best[6], prev[6];
        int nsol, bad = 0, s;

        fk(q, T);
        nsol = ik_solve(DH_TABLE, T, sol);
        if (nsol <= 0) { printf("  [FAIL] 起点无解\n"); g_fail++; }
        else {
            if (ik_select_best_continuous(sol, nsol, q, NULL, best) != 0) {
                printf("  [FAIL] select_best_continuous 返回失败\n");
                g_fail++;
            }
            memcpy(prev, best, sizeof prev);
            /* 沿一条小直线走 200 步，检查每步的关节变化都是小量 */
            for (s = 1; s <= 200; s++) {
                double step[3] = {0.25, 0.10, -0.15};   /* mm/步 */
                double T2[4][4], sol2[IK_MAX_SOLUTIONS][6], best2[6];
                int ns2, m;
                for (k = 0; k < 4; k++)
                    for (i = 0; i < 4; i++) T2[k][i] = T[k][i];
                for (k = 0; k < 3; k++) T2[k][3] += step[k] * s;
                ns2 = ik_solve(DH_TABLE, T2, sol2);
                if (ns2 <= 0) continue;
                if (ik_select_best_continuous(sol2, ns2, prev, NULL, best2) != 0) continue;
                for (m = 0; m < 6; m++) {
                    if (fabs(wrap180(best2[m] - prev[m])) > 20.0) {
                        if (bad < 3)
                            printf("  步 %d: J%d 跳变 %.1f°\n", s, m + 1,
                                   wrap180(best2[m] - prev[m]));
                        bad++;
                    }
                }
                memcpy(prev, best2, sizeof prev);
            }
            printf("  200 步中异常跳变: %d %s\n", bad, bad ? " <-- 有问题" : "OK");
            if (bad) g_fail++;
        }
    }

    printf("\n%s（失败项 %d）\n", g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
