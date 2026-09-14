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
 * 【参数来源】机械臂结构参数（单位 mm），经标准 DH 建模：
 *   J1  a=35   基座到1轴的水平X偏移   d=140 基座垂直高度      alpha=-90°
 *   J2  a=146  大臂长度               d=0                  alpha= 0°  home=-90°
 *   J3  a=0    d=52   3轴→4轴中心距(肘部)  alpha= 90°  home= 90°
 *   J4  a=0    d=115  4轴→5轴小臂长度       alpha=-90°
 *   J5  a=0    d=0                      alpha= 90°
 *   J6  a=0    d=91.5 J5→末端距离(腕长)    alpha= 0°
 *     （d 原误写 183 = 2×91.5，已按 CAD 实测 91.50 修正）
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
