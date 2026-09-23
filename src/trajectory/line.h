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

void line_pose_to_matrix(const double pose6[6], double m[4][4]);

int line_count_for_distance(double dist_mm, double step_mm);

int line_plan(const double start_pose[6], const double end_pose[6],
              int count, LinePath *path);

int line_solve(const LinePath *path, const DhParam *dh, const JointLimit *limits,
               const double *start_joints, double (*q_out)[6],
               int *fail_idx, char *fail_reason);

int line_time_table(const double (*q_seq)[6], int count, const double vmax_joint[6],
                    double *seg_dt, double *dt_total);

double line_max_joint_jump(const double (*q_seq)[6], int count,
                           int *out_joint, int *out_seg);

#endif
