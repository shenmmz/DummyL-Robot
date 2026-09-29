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

/* 简化逆解：只要解、不要状态，返回解数。 */
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
