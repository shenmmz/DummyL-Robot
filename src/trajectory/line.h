#ifndef LINE_H
#define LINE_H


#include "kinematics/dh_params.h"
#include "kinematics/ik.h"

#define LINE_MAX_POINTS 257
#define LINE_MAX_SEGS   (LINE_MAX_POINTS - 1)

typedef struct {
    int    count;
    double pose[LINE_MAX_POINTS][6];
} LinePath;

/* 6 维位姿(x,y,z,Rx,Ry,Rz) → 4x4 齐次矩阵。 */
void line_pose_to_matrix(const double pose6[6], double m[4][4]);

/* 按步长算需要几个点（含首尾，上限 LINE_MAX_POINTS=257）。 */
int line_count_for_distance(double dist_mm, double step_mm);

/* 位置线性插值 + 姿态 SLERP，生成 count 个位姿点。
 * ⚠️ end_pose 的姿态必须抄【当前 getpos】原值，否则画斜线。
 * 实测 80mm 线：抄 getpos 原值 ⇒ 0.0000mm；抄 home 的 115,90,115 ⇒ 7.71mm。 */
int line_plan(const double start_pose[6], const double end_pose[6],
              int count, LinePath *path);

/* 逐点位姿逆解成关节序列（带限位过滤 + 连续选优）。
 * 失败时经 fail_idx/fail_reason 回带第一个坏点与原因。
 * ⚠️ 若第 1 段就要求某轴转几十度，多半是路径擦过腕部奇异(J5≈0)或分支翻转。 */
int line_solve(const LinePath *path, const DhParam *dh, const JointLimit *limits,
               const double *start_joints, double (*q_out)[6],
               int *fail_idx, char *fail_reason);

/* 按各轴最大速度算每段时间，seg_dt 出参长度 count-1。 */
int line_time_table(const double (*q_seq)[6], int count, const double vmax_joint[6],
                    double *seg_dt, double *dt_total);

/* 求相邻点最大关节跳变量（度），用于判路径是否擦过奇异。 */
double line_max_joint_jump(const double (*q_seq)[6], int count,
                           int *out_joint, int *out_seg);

#endif
