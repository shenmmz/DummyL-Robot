#ifndef FK_H
#define FK_H

#include "kinematics/dh.h"

/* ============================================================
 * 正运动学（FK）：声明在本文件，实现在 fk.c。
 * 建模数据（DhParam / DH_JOINT_COUNT / DH_TABLE / dh_set_tool_length）
 * 在 dh.h + dh.c，本文件通过上面的 include 取得。
 * ============================================================ */

/* RPY 万向锁判据（dh_pose_to_xyz_rpy 用）。 */
#define DH_EPS 1e-6

/* 单个关节的标准 DH 齐次变换：Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)。 */
void dh_transform(const DhParam *p, double theta_rad, double t[4][4]);

/* 六轴正解：关节角（度，机械角）→ 末端位姿矩阵。
 * 实测单次 <40us，不是性能瓶颈。 */
void dh_forward(const DhParam *params, const double *joints_deg, double pose[4][4]);

/* 位姿矩阵 → 位置(mm) + RPY(度)，约定 Rz(yaw)*Ry(pitch)*Rx(roll)。 */
void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3]);

#endif
