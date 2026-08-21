/*
 * test_fk_ik.c —— 正/逆运动学闭环单元测试（CTest）
 * ------------------------------------------------------------
 * 所属模块：测试（tests）
 * 对外接口：main（入口）
 * 依赖模块：kinematics/dh、kinematics/ik
 */

/*
 * test_fk_ik: 正运动学 / 解析逆运动学闭环测试
 * ------------------------------------------------------------
 * 使用 DummyL-Robot 真实 DH 参数（config/dh_params.h，用户提供结构
 * 尺寸：L_BS=140/D_BS=35/L_AM=146/L_FA=115/D_EW=52/L_WT=183），
 * 满足 IK 假设：球腕、alpha4=-90°、alpha5=+90°、a4=a5=a6=0。
 * 做 FK->IK->FK 闭环校验、限位筛选与最优解选择校验。
 */

#include "kinematics/dh.h"
#include "kinematics/ik.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TOL 1e-6

static int g_fail = 0;

static void check(int cond, const char *name)
{
    if (cond) {
        printf("通过 [%s]\n", name);
    } else {
        printf("失败 [%s]\n", name);
        g_fail++;
    }
}

/* 真实 DH 参数（单位 mm/rad，与 config/dh_params.h 一致） */
static const DhParam TEST_DH[6] = {
    {  35.0, -M_PI / 2, 140.0,       0.0 }, /* J1 D_BS=35/L_BS=140 */
    { 146.0,       0.0,   0.0, -M_PI / 2 }, /* J2 L_AM=146, home=-90° */
    {   0.0,  M_PI / 2,  52.0,  M_PI / 2 }, /* J3 D_EW=52, home=90° */
    {   0.0, -M_PI / 2, 115.0,       0.0 }, /* J4 L_FA=115 */
    {   0.0,  M_PI / 2,   0.0,       0.0 }, /* J5 */
    {   0.0,       0.0, 183.0,       0.0 }  /* J6 L_WT=183 */
};

static double pose_err(const double a[4][4], const double b[4][4])
{
    double e = 0.0;
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            double d = a[i][j] - b[i][j];
            e += d * d;
        }
    }
    return sqrt(e);
}

static void run_case(const double q[6], const char *name)
{
    double pose[4][4];
    double sols[IK_MAX_SOLUTIONS][6];
    double filtered[IK_MAX_SOLUTIONS][6];
    double best[6];
    int n, m, i, k;

    dh_forward(TEST_DH, q, pose);

    n = ik_solve(TEST_DH, pose, sols);
    check(n >= 1, name);

    /* 每组解 FK 闭环 */
    for (k = 0; k < n; k++) {
        char nm[64];
        double re_pose[4][4];
        dh_forward(TEST_DH, sols[k], re_pose);
        snprintf(nm, sizeof(nm), "%s 解%d 闭环", name, k + 1);
        check(pose_err(pose, re_pose) < TOL, nm);
    }

    /* 限位筛选：窄限位（-80..80 度），仅保留在界内的解 */
    {
        JointLimit lim[6];
        int all_in = 1;
        for (i = 0; i < 6; i++) {
            lim[i].min_deg = -80.0;
            lim[i].max_deg = 80.0;
        }
        m = ik_filter_by_limits(sols, n, lim, filtered);
        for (i = 0; i < m; i++) {
            for (k = 0; k < 6; k++) {
                if (filtered[i][k] < lim[k].min_deg - 1e-9 ||
                    filtered[i][k] > lim[k].max_deg + 1e-9) {
                    all_in = 0;
                }
            }
        }
        check(m <= n && all_in, "限位筛选结果均在界内");
    }

    /* 最优解：以第一组解为目标当前位形，应选中该组附近 */
    if (n > 0) {
        double cur[6];
        memcpy(cur, sols[0], sizeof(cur));
        check(ik_select_best(sols, n, cur, NULL, best) == 0, "最优解选择成功");
        {
            double err = 0.0;
            for (i = 0; i < 6; i++) {
                double d = best[i] - cur[i];
                err += d * d;
            }
            check(sqrt(err) < 1e-6, "最优解与当前位形一致");
        }
    }
}

int main(void)
{
    const double cases[][6] = {
        { 30.0, -60.0,  45.0,  30.0,  60.0,  20.0 },
        { 10.0, -30.0,  30.0,  20.0, -45.0,  40.0 },
        { -20.0, 45.0, -30.0,  40.0,  30.0, -50.0 },
        { 45.0,  30.0,  60.0, -60.0,  45.0, -30.0 },
        /* 注：全零位形为肘伸直+腕奇异（θ5=0），IK 数学上无解，故用小角度非奇异位形 */
        { 15.0, -15.0,  10.0,  10.0, -25.0,  35.0 }
    };
    int i;

    printf("== test_fk_ik ==\n");

    for (i = 0; i < (int)(sizeof(cases) / sizeof(cases[0])); i++) {
        char nm[64];
        snprintf(nm, sizeof(nm), "闭环用例%d", i + 1);
        run_case(cases[i], nm);
    }

    if (g_fail == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("共 %d 项失败\n", g_fail);
    return 1;
}
