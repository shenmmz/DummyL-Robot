#ifndef DH_H
#define DH_H

/* ============================================================
 * 建模：DH 参数表 + 工具长度标定。表定义在 dh.c。
 * 正解（FK）的声明在 fk.h，实现在 fk.c。
 * ============================================================ */

#define DH_JOINT_COUNT 6

/* DH 参数表的每行：theta_offset, d, a, alpha。 */
typedef struct {
    double theta_offset;
    double d;
    double a;
    double alpha;
} DhParam;

/* DH 参数表，定义在 dh.c。d6 = 91.5（法兰面，腕心+91.5，裸臂口径）+ tool_mm。 */
extern DhParam DH_TABLE[DH_JOINT_COUNT];

/* 设工具长度：d6 = 91.5（法兰面，腕心+91.5，裸臂口径）+ tool_mm。ini [tool] tool_length。 */
void dh_set_tool_length(double tool_mm);

#endif
