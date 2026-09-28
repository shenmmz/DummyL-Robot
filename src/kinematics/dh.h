#ifndef DH_H
#define DH_H

/* ============================================================
 * 建模与正解的公共头（原 dh_params.h 已并入本文件，2026-09-28）。
 *   实现：dh.c = 建模（DH_TABLE + dh_set_tool_length）
 *         fk.c = 正解（dh_transform / dh_forward / dh_pose_to_xyz_rpy）
 *   调用方（commands.c / main.c / ik.c / line.c）只需 include 本头。
 * ============================================================ */

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

/* RPY 万向锁判据（dh_pose_to_xyz_rpy 用）。 */
#define DH_EPS 1e-6

/* 单关节 DH 变换矩阵（fk.c）。 */
void dh_transform(const DhParam *p, double theta_rad, double t[4][4]);

/* 六轴正解：关节角（度，机械角）→ 末端位姿矩阵（fk.c）。
 * 实测单次 <40us，不是性能瓶颈。 */
void dh_forward(const DhParam *params, const double *joints_deg, double pose[4][4]);

/* 位姿矩阵 → 位置(mm) + RPY(度)（fk.c）。 */
void dh_pose_to_xyz_rpy(const double pose[4][4], double xyz[3], double rpy[3]);

/* 设工具长度：d6 = 91.5（法兰面，腕心+91.5，裸臂口径）+ tool_mm。ini [tool] tool_length（dh.c）。 */
void dh_set_tool_length(double tool_mm);

#endif
