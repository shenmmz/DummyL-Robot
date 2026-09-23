#ifndef DH_PARAMS_H
#define DH_PARAMS_H


#include <stddef.h>

#define DH_JOINT_COUNT 6

typedef struct {
    double a;
    double alpha;
    double d;
    double theta_offset;
} DhParam;

/* 六轴 DH 参数表（定义在 dh.c）。d6 = 183.0mm = 腕心 → 【装夹爪后的末端】。
 * 口径拆分（second/include/kin.h 是唯一出处）：法兰面 91.5（不装工具）
 * + 夹爪 91.5 = 183。本机当前装着夹爪/笔 ⇒ 取 183 才与真机末端对得上。
 * 再往外的工具长度由 dh_set_tool_length 叠加（tool_length 指"183 之外"那一段）。
 * ⚠️ d6 曾误写 91.5（= 不装工具的法兰口径），2026-09-23 定案 183，依据两条：
 *    ① second/ 用 183 时真机终点正确，用 91.5 时末端沿工具轴偏 91.5mm；
 *    ② docs/FK实机校验清单.md 的理论零位 Z=584（140+146+115+183）只有取 183 才成立
 *       （取 91.5 得 492.5）。
 * ⚠️ 与参考项目 Hg_Robot_Arm 的参数不同（d3=52 在平面外）⇒ 它的 IK 不能控制本机。 */
extern DhParam DH_TABLE[DH_JOINT_COUNT];

#endif
