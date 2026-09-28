#ifndef IK_H
#define IK_H


#include "kinematics/dh.h"

#define IK_MAX_SOLUTIONS 8

typedef struct {
    double min_deg;
    double max_deg;
} JointLimit;

typedef enum {
    IK_SOL_VALID = 1,
    IK_SOL_OUT_OF_REACH = 0,
    IK_SOL_SINGULAR = -1,
    IK_SOL_DEGENERATE = -2
} IkSolStatus;

typedef struct {
    IkSolStatus shoulder;
    IkSolStatus elbow;
    IkSolStatus wrist;
    int valid;
} IkSolInfo;

/* 全解逆解：返回解数（最多 IK_MAX_SOLUTIONS=8），info 回带每解的各段状态。 */
int ik_solve_ex(const DhParam *params, const double pose[4][4],
                double solutions[IK_MAX_SOLUTIONS][6],
                IkSolInfo info[IK_MAX_SOLUTIONS]);

/* 带参考位形的逆解（ref_joints 用于挑最接近的分支）。 */
int ik_solve_ref(const DhParam *params, const double pose[4][4],
                 const double *ref_joints,
                 double solutions[IK_MAX_SOLUTIONS][6],
                 IkSolInfo info[IK_MAX_SOLUTIONS]);

/* 解状态码转中文（给日志/告警用）。 */
const char *ik_sol_status_str(IkSolStatus s);

/* 简化逆解：只要解，不要状态。返回解数。
 * ✅ 2026-09-28 实测：口径统一到共面档后，本函数与 Hg_Robot_Arm 的 kin_ik
 *    【解集完全一致】—— 508 组位形最大最近解差 0.0000°、位置回代 0.000000mm
 *    （对端需开 -DKIN_FIX_ORIGINAL_BUGS 关掉它保留的原文 wrap bug，否则 146/508 组会崩）。
 *    ⚠️ 旧记录"喂真机位姿给它差 40°"是【口径不匹配】(52 在 d 列 vs a 列)造成的，已作废。
 *    口径不匹配时的真实数字：位置回代 73.539105mm，而关节角解集仍差 0.0000°
 *    ⇒ 只看姿态发现不了，必须量位置。
 * 实测单次解算 <40µs，不是瓶颈。 */
int ik_solve(const DhParam *params, const double pose[4][4],
             double solutions[IK_MAX_SOLUTIONS][6]);

/* 按关节限位过滤候选解，返回留下的个数。 */
int ik_filter_by_limits(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const JointLimit *limits, double filtered[IK_MAX_SOLUTIONS][6]);

/* 按加权距离挑离 current_joints 最近的一解，写入 best。 */
int ik_select_best(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                   const double *current_joints, const double *weights, double best[6]);


/* 角度归一化到 (-180, 180]。 */
double ik_wrap_deg(double deg);

/* 把 deg 解到离 ref_deg 最近的那一支（±360k）。 */
double ik_unwrap_near(double deg, double ref_deg);

/* 批量解缠：让每个候选解都贴近 ref_joints。 */
int ik_unwrap_solutions(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const double *ref_joints, double out[IK_MAX_SOLUTIONS][6]);

/* 连续版选优：先解缠再比距离，避免选出绕一大圈的等价解。 */
int ik_select_best_continuous(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                              const double *current_joints, const double *weights, double best[6]);

#endif
