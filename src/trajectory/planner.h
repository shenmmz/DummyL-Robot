#ifndef PLANNER_H
#define PLANNER_H

/*
 * 点到点关节轨迹规划
 * ------------------------------------------------------------
 * 支持两种轮廓：
 *   1) 梯形（Trapezoidal）：匀加速-匀速-匀减速，三段；
 *   2) S 曲线（Jerk-limited）：7 段对称轮廓（加加速/恒加速/减加速/
 *      匀速/加减速/恒减速/减减速），速度与加速度连续。
 * 输入输出均为关节位置（度），时间（秒）。
 * 纯 C、无平台依赖，可离线单测。
 */

#define PROFILE_TRAPEZOIDAL 0
#define PROFILE_S_CURVE     1

typedef struct {
    double q0;       /* 起始位置（度） */
    double q1;       /* 目标位置（度） */
    double vmax;     /* 最大速度（度/秒） */
    double amax;     /* 最大加速度（度/秒^2） */
    double jmax;     /* 最大加加速度（度/秒^3），S 曲线使用 */
    double t_total;  /* 总时长（秒） */
    int    profile;  /* PROFILE_TRAPEZOIDAL / PROFILE_S_CURVE */

    /* 内部时间分段（秒），梯形只用 T1/T2/T4 */
    double T1, T2, T3, T4, T5, T6, T7;
    double v_peak;   /* 实际达到的峰值速度 */
} TrajPlan;

/* 规划：成功返回 0，参数非法返回 -1。
 * profile 为 PROFILE_TRAPEZOIDAL 或 PROFILE_S_CURVE。 */
int planner_plan(double q0, double q1, double vmax, double amax,
                 double jmax, int profile, TrajPlan *plan);

/* 求值：t 时刻位置/速度/加速度（可为 NULL 忽略）。
 * 返回 0 成功；t 超出总时长时按终点返回。 */
int planner_eval(const TrajPlan *plan, double t,
                 double *q, double *v, double *a);

#endif /* PLANNER_H */
