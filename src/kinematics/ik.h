#ifndef IK_H
#define IK_H


#include "kinematics/dh_params.h"

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

int ik_solve_ex(const DhParam *params, const double pose[4][4],
                double solutions[IK_MAX_SOLUTIONS][6],
                IkSolInfo info[IK_MAX_SOLUTIONS]);

int ik_solve_ref(const DhParam *params, const double pose[4][4],
                 const double *ref_joints,
                 double solutions[IK_MAX_SOLUTIONS][6],
                 IkSolInfo info[IK_MAX_SOLUTIONS]);

const char *ik_sol_status_str(IkSolStatus s);

int ik_solve(const DhParam *params, const double pose[4][4],
             double solutions[IK_MAX_SOLUTIONS][6]);

int ik_filter_by_limits(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const JointLimit *limits, double filtered[IK_MAX_SOLUTIONS][6]);

int ik_select_best(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                   const double *current_joints, const double *weights, double best[6]);


double ik_wrap_deg(double deg);

double ik_unwrap_near(double deg, double ref_deg);

int ik_unwrap_solutions(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                        const double *ref_joints, double out[IK_MAX_SOLUTIONS][6]);

int ik_select_best_continuous(const double solutions[IK_MAX_SOLUTIONS][6], int candidate_cnt,
                              const double *current_joints, const double *weights, double best[6]);

#endif
