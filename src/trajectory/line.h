#ifndef LINE_H
#define LINE_H

/*
 * line.h —— 笛卡尔直线插补（moveL 轨迹层）
 * ------------------------------------------------------------
 * 职责：
 *   1) line_plan          把笛卡尔直线段离散为中间位姿序列
 *                         （位置线性插值，姿态四元数 SLERP 插值）；
 *   2) line_solve         逐点求逆解，并按【分支连续】选解：候选解相对上一点
 *                         做角度归一化（±180° 等价角去卷绕）后取变化量最小者，
 *                         避免选中腕部翻转分支导致末端划弧；
 *   3) line_time_table    按关节限速生成分段时间表（等间隔段长 → 末端匀速）。
 * 依赖模块：kinematics（dh_params / dh / ik）
 *
 * 坐标系与单位：位姿 {X,Y,Z,Rx,Ry,Rz} = {mm, mm, mm, deg, deg, deg}，
 * 姿态为 ZYX 欧拉角（与 dh_pose_to_xyz_rpy / ik_solve 同口径）。
 */

#include "kinematics/dh_params.h"
#include "kinematics/ik.h"

/* 插补点上限（含起终点）；段数 = 点数 - 1 */
#define LINE_MAX_POINTS 257
#define LINE_MAX_SEGS   (LINE_MAX_POINTS - 1)

/* 直线路径：count 个位姿，pose[i][6] 为 {x,y,z,rx,ry,rz} */
typedef struct {
    int    count;
    double pose[LINE_MAX_POINTS][6];
} LinePath;

/* line_pose_to_matrix：6 维位姿 -> 4x4 齐次矩阵（行主序；姿态按 ZYX 欧拉角 [Rx,Ry,Rz]=(roll,pitch,yaw) 解释，R = Rz·Ry·Rx） */
void line_pose_to_matrix(const double pose6[6], double m[4][4]);

/* line_count_for_distance：按步长 step_mm 计算插补点数。
 * 结果保证 >=2（起终点各一）且 <= LINE_MAX_POINTS；step_mm<=0 或 dist<=0 时返回 2。 */
int line_count_for_distance(double dist_mm, double step_mm);

/* line_plan：把 start_pose -> end_pose 的直线段离散为 count 个位姿写入 path。
 * 位置线性插值；姿态采用四元数 SLERP 球面插值，确保中间位姿姿态连续
 * （无 RPY 线性插值的 ±180° 跳变问题）。
 * count 会被夹到 [2, LINE_MAX_POINTS]。成功返回 0，参数非法返回 -1。 */
int line_plan(const double start_pose[6], const double end_pose[6],
              int count, LinePath *path);

/* line_solve：逐点逆解 + 分支连续选解。
 *   dh           DH 参数表（通常为 DH_TABLE）
 *   limits       关节软限位（NULL 表示不限位）
 *   start_joints 起点参考关节角（度），用于第 0 点的分支选择
 *   q_out        count x 6 关节角（度）输出，与 path->pose 一一对应
 *   fail_idx     [输出] 失败点下标（可为 NULL）
 *   fail_reason  [输出] 失败原因描述（可为 NULL；缓冲区需 >= 64 字节）
 * 全部点求解成功返回 0；任一点 IK 无解或候选解全部越软限位返回 -1，
 * 并把该点下标写入 *fail_idx、原因写入 *fail_reason。
 * 失败时已写入的 q_out 无意义。 */
int line_solve(const LinePath *path, const DhParam *dh, const JointLimit *limits,
               const double *start_joints, double (*q_out)[6],
               int *fail_idx, char *fail_reason);

/* line_time_table：按关节限速生成分段时间表。
 *   q_seq      count x 6 关节角（度，应已去卷绕连续）
 *   vmax_joint 6 个关节的角速度上限（deg/s，须 > 0）
 *   seg_dt     输出 count-1 个段时长（秒）；等间隔（末端匀速）
 *   dt_total   输出总时长（秒），可为 NULL
 * 取所有段中"最紧关节"所需时间为统一段长，保证匀速且不超关节限速。
 * 成功返回 0；参数非法返回 -1。位移为 0（无需运动）时 dt_total = 0。 */
int line_time_table(const double (*q_seq)[6], int count, const double vmax_joint[6],
                    double *seg_dt, double *dt_total);

#endif /* LINE_H */
