/*
 * test_ik_singular —— 角度归一化 / 奇异位形 / 不可达（离线）
 *
 * 【为什么单独测它】
 *   - ik_wrap_deg / ik_unwrap_near 是"分支连续选解"的地基。这两个函数错一点，
 *     MoveL 会在半路突然选到腕部翻转的远分支，末端直接划一个大弧——
 *     现场看到的现象是"线画到一半甩出去"，很容易被误判成机械问题。
 *   - 奇异与不可达是 IK 的两类"合法失败"，必须返回明确状态而不是给出垃圾解，
 *     否则上层会拿着 NaN 去下发。
 *
 * 运行：cmake --build build --target test_ik_singular && build/bin/test_ik_singular.exe
 */

#include <stdio.h>
#include <math.h>
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-44s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

/* 与 ik.h 文档一致：归一化到 (-180°, 180°] */
static void expect_wrap(const char *tag, double in, double want)
{
    char buf[96];
    double got = ik_wrap_deg(in);
    snprintf(buf, sizeof buf, "in=%.1f → got=%.4f want=%.4f", in, got, want);
    check(tag, fabs(got - want) < 1e-6, buf);
}

int main(void)
{
    char buf[128];
    int i;

    dh_set_tool_length(0.0);

    printf("=== 1. ik_wrap_deg 角度归一化 ===\n");
    expect_wrap("0 不变",            0.0,    0.0);
    expect_wrap("90 不变",          90.0,   90.0);
    expect_wrap("190 → -170",      190.0, -170.0);
    expect_wrap("-190 → 170",     -190.0,  170.0);
    expect_wrap("360 → 0",         360.0,    0.0);
    expect_wrap("-360 → 0",       -360.0,    0.0);
    expect_wrap("540 → 180",       540.0,  180.0);
    expect_wrap("-170 不变",      -170.0, -170.0);

    printf("\n=== 2. ik_unwrap_near 取最近等价角 ===\n");
    {
        double a = ik_unwrap_near(-170.0, 190.0);   /* 应取 190 */
        snprintf(buf, sizeof buf, "unwrap(-170, ref=190) = %.4f", a);
        check("跨 180° 边界取近侧", fabs(a - 190.0) < 1e-6, buf);

        {
            double b = ik_unwrap_near(190.0, -170.0);
            snprintf(buf, sizeof buf, "unwrap(190, ref=-170) = %.4f", b);
            check("反向也成立", fabs(b + 170.0) < 1e-6, buf);
        }
        {
            /* 结果必须始终在 ref 的 ±180° 内 */
            int ok = 1;
            for (i = -720; i <= 720; i += 7) {
                double r = ik_unwrap_near((double)i, 37.0);
                if (fabs(r - 37.0) > 180.0 + 1e-6) ok = 0;
                if (fabs(fmod(r - (double)i, 360.0)) > 1e-6 &&
                    fabs(fabs(fmod(r - (double)i, 360.0)) - 360.0) > 1e-6) ok = 0;
            }
            check("207 个采样：|结果-ref| ≤ 180 且等价", ok, NULL);
        }
    }

    printf("\n=== 3. 腕部奇异：J5 = 0（J4/J6 轴共线）===\n");
    {
        double q[6] = {30.0, 20.0, 110.0, 50.0, 0.0, -20.0};
        double T[4][4], sol[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        int n, k, valid = 0;

        dh_forward(DH_TABLE, q, T);
        n = ik_solve_ex(DH_TABLE, T, sol, info);
        printf("  候选解 %d 组\n", n);
        for (k = 0; k < IK_MAX_SOLUTIONS; k++) {
            printf("    #%d 肩=%s 肘=%s 腕=%s %s\n", k,
                   ik_sol_status_str(info[k].shoulder),
                   ik_sol_status_str(info[k].elbow),
                   ik_sol_status_str(info[k].wrist),
                   info[k].valid ? "" : "(整组无效)");
        }
        /* 奇异位形下解不唯一，但至少要给出能还原位姿的解，且不能出 NaN */
        for (k = 0; k < n; k++) {
            double Ts[4][4];
            int bad = 0, r, c;
            dh_forward(DH_TABLE, sol[k], Ts);
            for (r = 0; r < 4; r++)
                for (c = 0; c < 4; c++)
                    if (!isfinite(Ts[r][c])) bad = 1;
            if (!bad && fabs(Ts[0][3] - T[0][3]) < 0.01 &&
                fabs(Ts[1][3] - T[1][3]) < 0.01 &&
                fabs(Ts[2][3] - T[2][3]) < 0.01) valid++;
        }
        snprintf(buf, sizeof buf, "能还原位姿的解 %d 组", valid);
        check("奇异下仍有有效解且无 NaN", n > 0 && valid > 0, buf);
    }

    printf("\n=== 4. 不可达：远在天边的位姿必须明确无解 ===\n");
    {
        double pose6[6] = {5000.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double T[4][4], sol[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        int n, k, nan_seen = 0;
        line_pose_to_matrix(pose6, T);
        n = ik_solve_ex(DH_TABLE, T, sol, info);
        for (k = 0; k < n; k++) {
            for (i = 0; i < 6; i++) if (!isfinite(sol[k][i])) nan_seen = 1;
        }
        snprintf(buf, sizeof buf, "返回 %d 组（期望 0）", n);
        check("X=5000mm 不可达 ⇒ 0 组解", n == 0 && !nan_seen, buf);
    }

    printf("\n%s（失败项 %d）\n", g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
