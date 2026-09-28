#ifndef DH_PARAMS_H
#define DH_PARAMS_H


#include <stddef.h>

#define DH_JOINT_COUNT 6

/* ⚠️ 字段顺序 = second/include/kin.c 的 g_DH 列序 {theta_offset, d, a, alpha}，
 *    方便两套代码逐行对照。全项目对 DhParam 的访问一律按【字段名】
 *    （p->a / params[2].d / params[k].theta_offset），无一处按位置 ⇒ 顺序可安全调整。 */
typedef struct {
    double theta_offset;
    double d;
    double a;
    double alpha;
} DhParam;

/* 六轴 DH 参数表（定义在 dh.c）。
 *
 * 【d6 口径】唯一出处是 second/include/kin.h：
 *   法兰面 91.5（裸臂） + 夹爪/工具 91.5 = 183（装夹爪后的末端）。
 *   代码当前 = **91.5 + ini [tool] tool_length**（2026-09-28 用户拍板用裸臂口径
 *   ⇒ ini 里装多少就填多少，不要再叠加那 91.5）。
 *   ⚠️ 本注释此前写"2026-09-23 定案 183"，与代码不符，已更正为如实描述。
 *   ⚠️ 换 d6 是纯重参数化：同一组关节角下末端沿【法兰轴】平移 Δd6。
 *      home（法兰轴 = +X）：91.5→183 使 X 241.5→333.0；q=0 位形 Z 492.5→584.0。
 *
 * 【肘部口径】2026-09-28 起 = 共面档：52 写在 a 列（a3=−52、d3=0），与参考项目
 *   Hg_Robot_Arm 的 DUMMY_T 同口径；ik.c 肘三角配套用 l_ew=hypot(a3,d4) / beta=atan2(-a3,d4)。 */
extern DhParam DH_TABLE[DH_JOINT_COUNT];

#endif
