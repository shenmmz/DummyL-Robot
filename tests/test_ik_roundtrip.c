/*
 * test_ik_roundtrip.c —— FK→IK→FK 闭环离线回归测试
 * ------------------------------------------------------------
 * 所属：tests（CTest）
 * 方法：固定种子生成随机关节角 → dh_forward 得目标位姿 → ik_solve 求全部候选解
 *       → 逐解 dh_forward 回代，比对位置与姿态误差。
 * 判据：
 *   有解率 ≥ 99%（原离线验证为 100%）；
 *   每个"有解"样本中至少 1 组解满足 位置误差 < 1e-6 mm 且 姿态矩阵元素误差 < 1e-9；
 *   全体解的最大位置误差 < 1e-6 mm。
 * 固定种子保证结果可复现，可在无硬件环境下随 CTest 运行。
 */

#include "kinematics/dh.h"
#include "kinematics/ik.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define CASES 2000
#define SEED  20260910u

static uint32_t g_seed = SEED;

/* 线性同余伪随机：[lo, hi) 均匀分布，结果可复现 */
static double frand(double lo, double hi)
{
    g_seed = g_seed * 1664525u + 1013904223u;
    return lo + (hi - lo) * ((double)(g_seed >> 8) / 16777216.0);
}

int main(void)
{
    static const double lo[6] = { -180.0, -120.0, -150.0, -180.0, -120.0, -180.0 };
    static const double hi[6] = {  180.0,  120.0,  150.0,  180.0,  120.0,  180.0 };
    double q[6], pose[4][4], sols[IK_MAX_SOLUTIONS][6];
    int solved = 0, matched = 0;
    double max_pos_err = 0.0, max_rot_err = 0.0;
    int n, i, j, k;

    printf("test_ik_roundtrip: %d 组 FK→IK→FK 闭环（种子 %u）\n", CASES, SEED);

    for (n = 0; n < CASES; n++) {
        int cnt, sample_ok = 0;

        for (i = 0; i < 6; i++) {
            q[i] = frand(lo[i], hi[i]);
        }

        dh_forward(DH_TABLE, q, pose);
        cnt = ik_solve(DH_TABLE, pose, sols);
        if (cnt <= 0) {
            continue;   /* 统计有解率，不视为失败 */
        }
        solved++;

        for (k = 0; k < cnt; k++) {
            double back[4][4];
            double dx, dy, dz, dpos, drot = 0.0;

            dh_forward(DH_TABLE, sols[k], back);

            dx = back[0][3] - pose[0][3];
            dy = back[1][3] - pose[1][3];
            dz = back[2][3] - pose[2][3];
            dpos = sqrt(dx * dx + dy * dy + dz * dz);

            for (i = 0; i < 3; i++) {
                for (j = 0; j < 3; j++) {
                    double e = fabs(back[i][j] - pose[i][j]);
                    if (e > drot) drot = e;
                }
            }

            if (dpos > max_pos_err) max_pos_err = dpos;
            if (drot > max_rot_err) max_rot_err = drot;
            if (dpos < 1e-6 && drot < 1e-9) sample_ok = 1;
        }
        if (sample_ok) matched++;
    }

    printf("  有解样本: %d/%d (%.1f%%)\n", solved, CASES, 100.0 * solved / CASES);
    printf("  闭环一致样本: %d/%d\n", matched, solved);
    printf("  最大位置误差: %.3e mm\n", max_pos_err);
    printf("  最大姿态矩阵元素误差: %.3e\n", max_rot_err);

    if (solved < (int)(CASES * 0.99)) {
        printf("test_ik_roundtrip: FAIL（有解率低于 99%%）\n");
        return 1;
    }
    if (matched != solved) {
        printf("test_ik_roundtrip: FAIL（%d 个有解样本回代不闭合）\n", solved - matched);
        return 1;
    }
    if (max_pos_err >= 1e-6) {
        printf("test_ik_roundtrip: FAIL（最大位置误差 %.3e mm 超阈值 1e-6）\n", max_pos_err);
        return 1;
    }
    printf("test_ik_roundtrip: PASS\n");
    return 0;
}
