#ifndef DH_PARAMS_H
#define DH_PARAMS_H


#include <stddef.h>

#define DH_JOINT_COUNT 6

#define DH_D6_FLANGE_MM 91.5

typedef struct {
    double a;
    double alpha;
    double d;
    double theta_offset;
} DhParam;

/* 六轴 DH 参数表（定义在 dh.c）。d6=91.50mm 为法兰；实际工具长度另由
 * dh_set_tool_length 叠加。
 * ⚠️ 与参考项目 Hg_Robot_Arm 的参数不同（d3=52 在平面外）⇒ 它的 IK 不能控制本机。 */
extern DhParam DH_TABLE[DH_JOINT_COUNT];

#endif
