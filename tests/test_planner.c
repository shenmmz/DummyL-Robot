/*
 * test_planner.c —— 梯形/S 曲线轨迹规划单元测试（CTest）
 * ------------------------------------------------------------
 * 所属模块：测试（tests）
 * 对外接口：main（入口）
 * 依赖模块：trajectory/planner
 */

/*
 * test_planner: 梯形 / S 曲线关节插补测试
 * ------------------------------------------------------------
 * 验证：端点位置精确、速度/加速度不超限、插补单调、
 * S 曲线总时长不小于梯形（加速度连续换来的时间代价）。
 */

#include "trajectory/planner.h"

#include <math.h>
#include <stdio.h>

#define TOL 1e-9

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

static void check_profile(double q0, double q1, double vmax, double amax,
                          double jmax, int profile, const char *name)
{
    TrajPlan plan;
    double q, v, a;
    double t, dt = 0.01;
    double prev_q;
    int monotone = 1;
    int v_ok = 1, a_ok = 1;

    check(planner_plan(q0, q1, vmax, amax, jmax, profile, &plan) == 0, name);

    /* 端点 */
    planner_eval(&plan, 0.0, &q, &v, &a);
    check(fabs(q - q0) < TOL && fabs(v) < TOL, "起点位置/速度");
    planner_eval(&plan, plan.t_total, &q, &v, &a);
    check(fabs(q - q1) < TOL && fabs(v) < TOL, "终点位置/速度");
    check(plan.t_total > 0.0, "总时长大于零");

    /* 全程扫描：单调 + 限幅 */
    prev_q = q0;
    for (t = 0.0; t <= plan.t_total + dt; t += dt) {
        planner_eval(&plan, t, &q, &v, &a);
        if ((q1 > q0 && q < prev_q - TOL) || (q1 < q0 && q > prev_q + TOL)) {
            monotone = 0;
        }
        if (fabs(v) > vmax + 1e-6) {
            v_ok = 0;
        }
        if (fabs(a) > amax + 1e-6) {
            a_ok = 0;
        }
        prev_q = q;
    }
    check(monotone, "位置单调");
    check(v_ok, "速度不超限");
    check(a_ok, "加速度不超限");
}

int main(void)
{
    TrajPlan trap, scurve;

    printf("== test_planner ==\n");

    /* 梯形：常规（含匀速段） */
    check_profile(0.0, 90.0, 60.0, 120.0, 0.0, PROFILE_TRAPEZOIDAL, "梯形-常规");

    /* 梯形：距离不足（三角轮廓） */
    check_profile(0.0, 10.0, 60.0, 120.0, 0.0, PROFILE_TRAPEZOIDAL, "梯形-三角");

    /* 梯形：反向运动 */
    check_profile(30.0, -40.0, 80.0, 150.0, 0.0, PROFILE_TRAPEZOIDAL, "梯形-反向");

    /* S 曲线：常规 */
    check_profile(0.0, 90.0, 60.0, 120.0, 600.0, PROFILE_S_CURVE, "S曲线-常规");

    /* S 曲线：短行程 */
    check_profile(0.0, 10.0, 60.0, 120.0, 600.0, PROFILE_S_CURVE, "S曲线-短行程");

    /* S 曲线：反向 */
    check_profile(45.0, -30.0, 60.0, 100.0, 500.0, PROFILE_S_CURVE, "S曲线-反向");

    /* S 曲线总时长 >= 梯形（同一组限幅参数） */
    planner_plan(0.0, 90.0, 60.0, 120.0, 600.0, PROFILE_TRAPEZOIDAL, &trap);
    planner_plan(0.0, 90.0, 60.0, 120.0, 600.0, PROFILE_S_CURVE, &scurve);
    check(scurve.t_total >= trap.t_total - TOL, "S曲线时长不小于梯形");

    if (g_fail == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("共 %d 项失败\n", g_fail);
    return 1;
}
