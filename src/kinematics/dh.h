#ifndef DH_H
#define DH_H

/*
 * DH 建模与正运动学（FK）
 * 采用标准 DH 约定，关节角输入为角度（度），内部转弧度计算。
 * DH 参数表定义于 kinematics/dh_params.h。
 */

#include "kinematics/dh_params.h"

#define DH_EPS 1e-6

/* 单关节齐次变换：根据参数与关节角（rad，含 theta_offset）返回 4x4 行主序矩阵 */
void dh_transform(const DhParam *p, double theta_rad, double t[4][4]);

/* 正运动学：joints_deg[6] 为关节角（度），输出 4x4 行主序齐次矩阵 T0_6 */
void dh_forward(const DhParam *params, const double *joints_deg, double pose[4][4]);

/* 便捷：从 4x4 中取平移向量 (x,y,z) 与欧拉角 ZYX (rx,ry,rz)（rad） */
void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3]);

/* 设置六轴末端工具长度偏移(mm)：d6 = 法兰偏距 + 工具长。无工具传 0。 */
void dh_set_tool_length(double tool_mm);

#endif /* DH_H */
