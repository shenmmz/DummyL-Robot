#ifndef DH_PARAMS_H
#define DH_PARAMS_H

/*
 * DummyL-Robot 标准 DH 参数表
 * ----------------------------
 * 采用标准 DH 约定：
 *   a      : 连杆长度 (mm)     —— 沿 x_i 方向，从 z_{i-1} 到 z_i
 *   alpha  : 连杆扭转 (rad)    —— 绕 x_i 轴，从 z_{i-1} 转到 z_i
 *   d      : 连杆偏距 (mm)     —— 沿 z_{i-1} 方向，从 x_{i-1} 到 x_i
 *   theta_offset : 关节零位偏移 (rad)，含义为 theta = q + offset
 *                  （dh.c dh_forward 实现：theta = q*pi/180 + theta_offset）
 *
 * 【参数来源】用户提供的机械臂结构参数（单位 mm），经标准 DH 建模：
 *   J1 {a=35, d=140, alpha=-90°}   <- D_BS=35 基座x偏置 / L_BS=140 基座高度
 *   J2 {a=146, d=0,   alpha= 0°}   <- L_AM=146 大臂
 *   J3 {a=0,   d=52,  alpha= 90°}  <- D_EW=52 肘部偏置
 *   J4 {a=0,   d=115, alpha=-90°}  <- L_FA=115 前臂
 *   J5 {a=0,   d=0,   alpha= 90°}
 *   J6 {a=0,   d=183, alpha= 0°}   <- L_WT=183 腕部
 *   home 角度（J2=-90°, J3=90°, 其余 0）作为 theta_offset，
 *   使零位 (q=0) 时 FK 输出与机械 home 姿态一致。
 */

#include <stddef.h>

#define DH_JOINT_COUNT 6

typedef struct {
    double a;             /* 连杆长度 mm */
    double alpha;         /* 连杆扭转 rad */
    double d;             /* 连杆偏距 mm */
    double theta_offset;  /* 关节角偏移 rad */
} DhParam;

/* 关节 1..6 的标准 DH 参数表 */
extern const DhParam DH_TABLE[DH_JOINT_COUNT];

#endif /* DH_PARAMS_H */
