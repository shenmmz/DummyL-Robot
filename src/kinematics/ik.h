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

/* IK 解状态：肩/肘/腕三段各标记，对应 Hg_Robot_Arm solFlag 概念 */
typedef enum {
    IK_SOL_VALID = 1,        /* 有效 */
    IK_SOL_OUT_OF_REACH = 0, /* 超出可达范围 */
    IK_SOL_SINGULAR = -1,    /* 奇异（自由度退化） */
    IK_SOL_DEGENERATE = -2   /* 几何退化（参数异常） */
} IkSolStatus;

/* 每组解的奇异标记：肩/肘/腕三段独立标记 + 整体有效标志 */
typedef struct {
    IkSolStatus shoulder;
    IkSolStatus elbow;
    IkSolStatus wrist;
    int valid;              /* 1=该组解完整有效，0=至少一段无效 */
} IkSolInfo;

/* ik_solve_ex：扩展逆解，输出候选解 + 每组解的奇异标记。
 * solutions 仅填充有效解（前 count 组），info 填充全部 8 个槽位。
 * info 为 NULL 时不输出标记（等价于 ik_solve）。
 * 返回有效解组数（0~8）。 */
int ik_solve_ex(const DhParam *params, const double pose[4][4],
                double solutions[IK_MAX_SOLUTIONS][6],
                IkSolInfo info[IK_MAX_SOLUTIONS]);

/* ik_solve_ref：带【腕奇异连续性参考】的逆解。
 *
 *   ref_joints  上一点的关节角（度），可 NULL。
 *               只在腕奇异退化分支里起作用：θ5≈0 时 θ4 与 θ6 无法分别测定
 *               （只有 θ4+θ6 可定），此时 θ4 沿用它、θ6 = (θ4+θ6) - θ4。
 *               不传参考的话 θ4 只能糙取 0 —— 路径一旦穿过奇异点，
 *               θ5 越过判定边界那一刻 θ4 就会从 0 跳到真实值，
 *               实测 **J4 单次跳变 89.8°**，末端甩出去。
 *
 * 【谁该用谁】
 *   单次求逆解（MoveJ、位姿查询）            → ik_solve_ex / ik_solve
 *   逐点求逆解（MoveL 的 line_solve）        → 必须 ik_solve_ref，传上一点关节角
 *
 * 其余约定与 ik_solve_ex 完全一致（返回值、槽位编号、info 填充规则）。 */
int ik_solve_ref(const DhParam *params, const double pose[4][4],
                 const double *ref_joints,
                 double solutions[IK_MAX_SOLUTIONS][6],
                 IkSolInfo info[IK_MAX_SOLUTIONS]);

/* ik_sol_status_str：IkSolStatus 转可读字符串，用于诊断输出 */
const char *ik_sol_status_str(IkSolStatus s);

/* ik_solve：逆解（向后兼容，等价于 ik_solve_ex + info=NULL）。
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
