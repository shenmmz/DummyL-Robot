#ifndef DH_H
#define DH_H


#include "kinematics/dh_params.h"

#define DH_EPS 1e-6

void dh_transform(const DhParam *p, double theta_rad, double t[4][4]);

void dh_forward(const DhParam *params, const double *joints_deg, double pose[4][4]);

void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3]);

void dh_set_tool_length(double tool_mm);

#endif
