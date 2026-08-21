/*
 * planner.c —— 单关节梯形/S 曲线轨迹规划与求值
 * ------------------------------------------------------------
 * 所属模块：轨迹规划（trajectory）
 * 对外接口：planner_plan、planner_eval
 * 依赖模块：无
 */

#include "trajectory/planner.h"

#include <math.h>
#include <stddef.h>

#define PL_EPS 1e-12

/* ---------- 梯形规划 ---------- */

/* trap_plan：梯形速度规划，计算加减速/匀速段时间与峰值速度 */
static int trap_plan(double q0, double q1, double vmax, double amax, TrajPlan *plan)
{
    double dist = q1 - q0;
    double s_acc, t_acc, t_const;

    if (vmax <= 0.0 || amax <= 0.0 || fabs(dist) < PL_EPS) {
        plan->q0 = q0;
        plan->q1 = q1;
        plan->vmax = vmax;
        plan->amax = amax;
        plan->t_total = 0.0;
        plan->v_peak = 0.0;
        plan->T1 = plan->T2 = plan->T4 = 0.0;
        plan->profile = PROFILE_TRAPEZOIDAL;
        return 0;
    }

    /* 加速段位移（初末速度为零）: s_acc = v^2 / (2a) */
    s_acc = vmax * vmax / (2.0 * amax);
    if (2.0 * s_acc >= fabs(dist)) {
        /* 距离不足，三角轮廓：峰值速度降低 */
        double vp = sqrt(fabs(dist) * amax);
        t_acc = vp / amax;
        plan->v_peak = vp;
        plan->T1 = t_acc;
        plan->T2 = 0.0;
        plan->T4 = 0.0;
    } else {
        t_acc = vmax / amax;
        t_const = (fabs(dist) - 2.0 * s_acc) / vmax;
        plan->v_peak = vmax;
        plan->T1 = t_acc;
        plan->T2 = t_const;
        plan->T4 = 0.0;
    }
    plan->T3 = plan->T1;
    plan->T5 = plan->T6 = plan->T7 = 0.0;
    plan->q0 = q0;
    plan->q1 = q1;
    plan->vmax = vmax;
    plan->amax = amax;
    plan->jmax = 0.0;
    plan->t_total = plan->T1 + plan->T2 + plan->T3;
    plan->profile = PROFILE_TRAPEZOIDAL;
    return 0;
}

/* trap_eval：梯形规划求值，求 t 时刻的位置/速度/加速度 */
static void trap_eval(const TrajPlan *p, double t, double *q, double *v, double *a)
{
    double dir = (p->q1 >= p->q0) ? 1.0 : -1.0;
    double t1 = p->T1, t2 = p->T1 + p->T2;
    double tt = t;

    if (tt < 0.0) tt = 0.0;
    if (tt > p->t_total) tt = p->t_total;

    if (tt < t1) {
        if (q) *q = p->q0 + dir * 0.5 * p->amax * tt * tt;
        if (v) *v = dir * p->amax * tt;
        if (a) *a = dir * p->amax;
    } else if (tt < t2) {
        double s1 = p->q0 + dir * 0.5 * p->amax * t1 * t1;
        if (q) *q = s1 + dir * p->v_peak * (tt - t1);
        if (v) *v = dir * p->v_peak;
        if (a) *a = 0.0;
    } else {
        double t3 = p->t_total;
        double rem = t3 - tt;
        if (q) *q = p->q1 - dir * 0.5 * p->amax * rem * rem;
        if (v) *v = dir * p->amax * rem;
        if (a) *a = -dir * p->amax;
    }
}

/* ---------- S 曲线（jerk-limited，7 段对称） ---------- */

/* s_acc_displacement：S 曲线加速段位移（给定峰值 vpeak、jerk J、amax，含三角退化） */
static double s_acc_displacement(double vpeak, double amax, double jmax)
{
    double T1 = amax / jmax;
    double T2 = vpeak / amax - T1;
    double q1, q2;

    if (T2 < 0.0) {
        /* 达不到 amax 的短行程：纯加加速+减加速 */
        double T = sqrt(vpeak / jmax);
        return jmax * T * T * T;
    }
    q1 = jmax * T1 * T1 * T1 / 6.0;                    /* 加加速段 */
    q2 = q1 + (0.5 * jmax * T1 * T1) * T2 + 0.5 * amax * T2 * T2; /* 恒加速段末 */
    /* 减加速段位移 */
    return q2 + (0.5 * jmax * T1 * T1 + amax * T2) * T1 + 0.5 * amax * T1 * T1
               - jmax * T1 * T1 * T1 / 6.0;
}

/* s_plan：S 曲线（jerk 受限）7 段对称规划，距离不足退化为梯形 */
static int s_plan(double q0, double q1, double vmax, double amax, double jmax, TrajPlan *plan)
{
    double dist = q1 - q0;
    double vpeak, T1, T2, q_acc;

    if (vmax <= 0.0 || amax <= 0.0 || jmax <= 0.0 || fabs(dist) < PL_EPS) {
        return trap_plan(q0, q1, vmax, amax, plan); /* 零位移/非法参数退化为梯形 */
    }

    /* 求实际峰值速度：2*q_acc(vpeak) <= |dist| 的最大 vpeak <= vmax */
    vpeak = vmax;
    if (2.0 * s_acc_displacement(vpeak, amax, jmax) > fabs(dist)) {
        double lo = 0.0, hi = vmax;
        int iter;
        for (iter = 0; iter < 64; iter++) {
            vpeak = 0.5 * (lo + hi);
            if (2.0 * s_acc_displacement(vpeak, amax, jmax) > fabs(dist)) {
                hi = vpeak;
            } else {
                lo = vpeak;
            }
        }
        vpeak = lo;
    }

    T1 = amax / jmax;
    T2 = vpeak / amax - T1;
    if (T2 < 0.0) {
        /* 距离过短，达不到 amax：T1 = sqrt(vpeak/jmax)，T2=0 */
        T1 = sqrt(vpeak / jmax);
        T2 = 0.0;
    }

    q_acc = s_acc_displacement(vpeak, amax, jmax);

    plan->q0 = q0;
    plan->q1 = q1;
    plan->vmax = vmax;
    plan->amax = amax;
    plan->jmax = jmax;
    plan->v_peak = vpeak;
    plan->T1 = T1;
    plan->T2 = T2;
    plan->T3 = T1;
    plan->T4 = (fabs(dist) - 2.0 * q_acc) / (vpeak > PL_EPS ? vpeak : 1.0);
    plan->T5 = T1;
    plan->T6 = T2;
    plan->T7 = T1;
    plan->t_total = plan->T1 + plan->T2 + plan->T3 + plan->T4 + plan->T5 + plan->T6 + plan->T7;
    plan->profile = PROFILE_S_CURVE;
    return 0;
}

/* s_eval：S 曲线 7 段求值，求 t 时刻的位置/速度/加速度 */
static void s_eval(const TrajPlan *p, double t, double *q, double *v, double *a)
{
    double dir = (p->q1 >= p->q0) ? 1.0 : -1.0;
    double J = p->jmax;
    double tt = t;
    double s = 0.0, vel = 0.0, acc = 0.0;
    double T1 = p->T1, T2 = p->T2, T3 = p->T3, T4 = p->T4;
    double T5 = p->T5, T6 = p->T6;
    double v1, v3, v4;

    if (tt < 0.0) tt = 0.0;
    if (tt > p->t_total) tt = p->t_total;

    v1 = 0.5 * J * T1 * T1;                    /* 加加速段末速度 */
    v3 = v1 + J * T1 * T2;                     /* 恒加速段末速度 */
    v4 = v3 + 0.5 * J * T1 * T1;               /* 减加速段末速度 = 峰值速度（匀速段） */

    if (tt < T1) {
        acc = J * tt;
        vel = 0.5 * J * tt * tt;
        s = J * tt * tt * tt / 6.0;
    } else if (tt < T1 + T2) {
        double dt = tt - T1;
        acc = J * T1;
        vel = v1 + J * T1 * dt;
        s = J * T1 * T1 * T1 / 6.0 + v1 * dt + 0.5 * J * T1 * dt * dt;
    } else if (tt < T1 + T2 + T3) {
        double dt = tt - (T1 + T2);
        double s2 = J * T1 * T1 * T1 / 6.0 + v1 * T2 + 0.5 * J * T1 * T2 * T2;
        acc = J * T1 - J * dt;
        vel = v3 + J * T1 * dt - 0.5 * J * dt * dt;
        /* 减加速段（正向积分） */
        s = s2 + v3 * dt + 0.5 * J * T1 * dt * dt - J * dt * dt * dt / 6.0;
    } else if (tt < T1 + T2 + T3 + T4) {
        double dt = tt - (T1 + T2 + T3);
        acc = 0.0;
        vel = v4;
        s = s_acc_displacement(p->v_peak, p->amax, J) + v4 * dt;
    } else if (tt < T1 + T2 + T3 + T4 + T5) {
        double dt = tt - (T1 + T2 + T3 + T4);
        double s_acc = s_acc_displacement(p->v_peak, p->amax, J);
        acc = -J * dt;
        vel = v4 - 0.5 * J * dt * dt;
        s = s_acc + v4 * T4 + v4 * dt - J * dt * dt * dt / 6.0;
    } else if (tt < T1 + T2 + T3 + T4 + T5 + T6) {
        double dt = tt - (T1 + T2 + T3 + T4 + T5);
        double s_acc = s_acc_displacement(p->v_peak, p->amax, J);
        double s5 = s_acc + v4 * T4 + v4 * T5 - J * T5 * T5 * T5 / 6.0;
        double v5 = v4 - 0.5 * J * T5 * T5;
        acc = -J * T5;
        vel = v5 - J * T5 * dt;
        s = s5 + v5 * dt - 0.5 * J * T5 * dt * dt;
    } else {
        double dt = tt - (T1 + T2 + T3 + T4 + T5 + T6);
        double s_acc = s_acc_displacement(p->v_peak, p->amax, J);
        double s5 = s_acc + v4 * T4 + v4 * T5 - J * T5 * T5 * T5 / 6.0;
        double v5 = v4 - 0.5 * J * T5 * T5;
        double s6 = s5 + v5 * T6 - 0.5 * J * T5 * T6 * T6;
        double v6 = v5 - J * T5 * T6;
        acc = -J * T5 + J * dt;
        vel = v6 - J * T5 * dt + 0.5 * J * dt * dt;
        s = s6 + v6 * dt - 0.5 * J * T5 * dt * dt + J * dt * dt * dt / 6.0;
    }

    if (q) *q = p->q0 + dir * s;
    if (v) *v = dir * vel;
    if (a) *a = dir * acc;
}

/* ---------- 对外接口 ---------- */

/* planner_plan：对外规划接口，按 profile 选择 S 曲线或梯形 */
int planner_plan(double q0, double q1, double vmax, double amax,
                 double jmax, int profile, TrajPlan *plan)
{
    if (plan == NULL) {
        return -1;
    }
    if (profile == PROFILE_S_CURVE) {
        return s_plan(q0, q1, vmax, amax, jmax, plan);
    }
    return trap_plan(q0, q1, vmax, amax, plan);
}

/* planner_eval：对外求值接口，按 profile 求 t 时刻的位置/速度/加速度 */
int planner_eval(const TrajPlan *plan, double t, double *q, double *v, double *a)
{
    if (plan == NULL) {
        return -1;
    }
    if (plan->profile == PROFILE_S_CURVE) {
        s_eval(plan, t, q, v, a);
    } else {
        trap_eval(plan, t, q, v, a);
    }
    return 0;
}
