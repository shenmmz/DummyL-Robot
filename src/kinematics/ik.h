#ifndef IK_H
#define IK_H

/*
 * 球腕解耦解析逆运动学（IK）
 * ------------------------------------------------------------
 * 适用范围（写入此处的算法假设，DH 实机参数确认时需复核）：
 *   1) 6R 机械臂，后三轴交于一点（球腕），且 a4 = a5 = a6 = 0；
 *   2) 标准 DH 结构：alpha = {-90°, 0, 90°, -90°, 90°, 0}（与 dh.c 一致）；
 *   3) 关节1 轴线与关节2 轴线垂直。
 * 算法输出最多 8 组候选解（肩 ×2、肘 ×2、腕翻转 ×2），
 * 随后按关节软限位筛选，再按"当前关节位置加权变化最小"选择最优解。
 */

#include "kinematics/dh_params.h"

#define IK_MAX_SOLUTIONS 8

/* 关节软限位（度）。未确认时可在调用方传入 NULL 表示不限位 */
typedef struct {
    double min_deg;
    double max_deg;
} JointLimit;

/* 解析逆解：给定目标位姿 4x4 行主序矩阵，输出候选解（度）。
 * 返回实际解组数（0~8）；solutions[n][6] 为关节角（度）。
 * 若肩/腕退化（奇异）对应组可能被跳过。 */
int ik_solve(const DhParam *params, const double pose[4][4],
             double solutions[IK_MAX_SOLUTIONS][6]);

/* 限位筛选：从 candidate_cnt 组解中筛出全部满足限位的解，
 * 写入 filtered（最多 IK_MAX_SOLUTIONS 组），返回数量。
 * limits 为 NULL 时全部保留。 */
int ik_filter_by_limits(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const JointLimit *limits, double filtered[IK_MAX_SOLUTIONS][6]);

/* 最优解选择：在候选解中按各关节相对当前关节角变化量加权最小取最优。
 * current_joints[6] 当前关节角（度），weights[6] 权重（NULL 时全 1）。
 * 找到返回 0 并写入 best[6]，无解返回 -1。 */
int ik_select_best(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                   const double *current_joints, const double *weights, double best[6]);

/* ---------- 角度归一化 / 分支连续选解（moveL 逐点逆解使用）----------
 *
 * 背景：ik_solve 输出的关节角统一归一化到 (-180°, 180°]，同一机械构型的
 * 等价角（相差 360° 整数倍）会被折回边界侧，例如 J4 实际 180° 被输出为
 * -180°。若直接按原值计算"相对上一点的变化量"，会把 0.01° 的等价小位移
 * 误判成 360° 大位移，导致选到腕部翻转的远分支解，末端划弧。
 * 因此逐点逆解必须先做"去卷绕（unwrap）"，再做最小变化选解。 */

/* ik_wrap_deg：把角度归一化到 (-180°, 180°] */
double ik_wrap_deg(double deg);

/* ik_unwrap_near：给 deg 叠加 360° 的整数倍，取距 ref_deg 最近的等价角
 * （|结果-ref_deg| <= 180°）。 */
double ik_unwrap_near(double deg, double ref_deg);

/* ik_unwrap_solutions：批量把候选解相对 ref_joints 去卷绕后写入 out，
 * 返回写出的组数（candidate_cnt<0 记 0；ref_joints 为 NULL 时按 0 参考）。 */
int ik_unwrap_solutions(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const double *ref_joints, double out[IK_MAX_SOLUTIONS][6]);

/* ik_select_best_continuous：分支连续选解。
 * 候选解先逐关节相对 current_joints 去卷绕，再按加权变化量最小取最优，
 * best[6] 输出的是【去卷绕后】的解（可直接用于插值/下发，保证相邻点连续）。
 * 找到返回 0，candidate_cnt<=0 返回 -1。 */
int ik_select_best_continuous(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                              const double *current_joints, const double *weights, double best[6]);

#endif /* IK_H */
