#ifndef ARC_H
#define ARC_H

/* 三点式空间圆弧插补。与 line.c 同属 trajectory 模块，产出同样的 LinePath，
 * 因此下游 line_solve（逐点 IK）/ line_time_table / 跳变闸全部原样复用——
 * 本文件只负责把"沿直线生成航点"换成"沿圆弧生成航点"。 */

#include "trajectory/line.h"   /* LinePath / line_count_for_distance / line_plan */

/* 过三点的空间圆弧：
 *   start_pose[6] = 起点位姿(x,y,z,Rx,Ry,Rz)，一般取当前 getpos；
 *   via[3]        = 弧必经的中间点(仅位置,mm)，决定凸向与半径；
 *   end_pose[6]   = 终点位姿(含显式姿态)。
 * 位置沿真实圆弧等步长采样；姿态由 start→end 四元数 SLERP（与直线一致）。
 * step_mm 为弧长方向的名义步长，内部用 line_count_for_distance(弧长,步长) 定点数。
 *
 * out_arc_len_mm 出参回带整段弧长(mm)，供上层日志/护栏用，可传 NULL。
 * 返回：
 *   0  正常沿弧插补；
 *   1  三点近共线 ⇒ 圆弧退化，已转调 line_plan 按直线走；
 *  <=-1 参数非法 / 出现 NaN / 有点重合定不出圆（path 不保证有效）。 */
int arc_plan_3pt(const double start_pose[6], const double via[3],
                 const double end_pose[6], double step_mm,
                 LinePath *path, double *out_arc_len_mm);

#endif
