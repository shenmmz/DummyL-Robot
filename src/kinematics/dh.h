#ifndef DH_H
#define DH_H


#include "kinematics/dh_params.h"

#define DH_EPS 1e-6

/* 单关节 DH 变换矩阵。 */
void dh_transform(const DhParam *p, double theta_rad, double t[4][4]);

/* 六轴正解：关节角（度，机械角）→ 末端位姿矩阵。
 * 实测单次 <40us，不是性能瓶颈。 */
void dh_forward(const DhParam *params, const double *joints_deg, double pose[4][4]);

/* 位姿矩阵 → 位置(mm) + RPY(度)。 */
void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3]);

/* 设工具长度（在法兰基础上再加一段 d6）。ini [tool] tool_length。 */
void dh_set_tool_length(double tool_mm);

#endif
