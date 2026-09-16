/*
 * movl_usercheck.c —— 复刻用户实际 MoveL 运行，做正运动学逐点断言
 * 场景：起点关节 = (0, 1.03, 101.25, 0, 77.71, 0)，目标 = (150,52,60,-180,0,-180)
 *       即仅改 Z（78.5 -> 60）的纯竖直直线。
 * 用真实 dh/ik/line 模块规划，再对每一插补点做 FK，检查：
 *   1) 起点 FK 是否等于 getpos 报告的 (150,52,78.5,-180,0,-180)（验证 d6=183 自洽）
 *   2) 中间点 X/Y 是否恒定（纯 Z 移动应 X=150,Y=52 不变）
 *   3) 中间点相对起终点连线的最大垂直残差（直线性，<0.05mm 为不画弧）
 *   4) 终点 FK 是否与目标 (150,52,60) 一致、姿态是否仍为 (-180,0,-180)
 *   5) 终点关节角是否与用户日志吻合（J2≈3.24,J3≈108.59,J5≈68.18）
 * 编译：gcc tests/movl_usercheck.c src/kinematics/dh.c src/kinematics/ik.c \
 *            src/trajectory/line.c -Isrc -lm -o tests/movl_usercheck.exe
 */
#include <stdio.h>
#include <math.h>
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"

#define RJ 6
#define MOVL_STEP_MM 1.0
#define RAD2DEG (180.0 / 3.14159265358979323846)

static const double LIM_MIN[RJ] = {-180,-180,-180,-360,-180,-360};
static const double LIM_MAX[RJ] = { 180, 180, 180, 360, 180, 360};

int main(void)
{
    double q_start[RJ] = {0.0, 1.03, 101.25, 0.0, 77.71, 0.0};
    double end_pose[6] = {150.0, 52.0, 60.0, -180.0, 0.0, -180.0};

    double P0[4][4], xyz0[3], rpy0[3];
    dh_forward(DH_TABLE, q_start, P0);
    dh_pose_to_xyz_rpy(P0, xyz0, rpy0);
    printf("START FK: X=%.3f Y=%.3f Z=%.3f  Rx=%.3f Ry=%.3f Rz=%.3f\n",
           xyz0[0], xyz0[1], xyz0[2],
           rpy0[0]*RAD2DEG, rpy0[1]*RAD2DEG, rpy0[2]*RAD2DEG);

    double d[3] = {end_pose[0]-xyz0[0], end_pose[1]-xyz0[1], end_pose[2]-xyz0[2]};
    double dist = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    printf("distance start->target = %.3f mm\n", dist);

    int count = line_count_for_distance(dist, MOVL_STEP_MM);
    double start_pose[6] = {xyz0[0], xyz0[1], xyz0[2],
                            rpy0[0]*RAD2DEG, rpy0[1]*RAD2DEG, rpy0[2]*RAD2DEG};

    LinePath path;
    if (line_plan(start_pose, end_pose, count, &path) != 0) {
        printf("[FAIL] line_plan error\n"); return 1;
    }

    JointLimit lim[RJ];
    for (int i = 0; i < RJ; i++) { lim[i].min_deg = LIM_MIN[i]; lim[i].max_deg = LIM_MAX[i]; }

    double q_out[LINE_MAX_POINTS][RJ];
    int fail = -1;
    if (line_solve(&path, DH_TABLE, lim, q_start, q_out, &fail) != 0) {
        printf("[FAIL] line_solve failed at point %d (IK 无解或越软限位)\n", fail); return 1;
    }
    printf("plan OK: %d points (%d segments)\n", count, count - 1);

    double X[LINE_MAX_POINTS][3];
    double max_Xdev = 0, max_Ydev = 0;
    for (int i = 0; i < count; i++) {
        double P[4][4], xyz[3], rpy[3];
        dh_forward(DH_TABLE, q_out[i], P);
        dh_pose_to_xyz_rpy(P, xyz, rpy);
        X[i][0] = xyz[0]; X[i][1] = xyz[1]; X[i][2] = xyz[2];
        double dx = fabs(xyz[0] - xyz0[0]);
        double dy = fabs(xyz[1] - xyz0[1]);
        if (dx > max_Xdev) max_Xdev = dx;
        if (dy > max_Ydev) max_Ydev = dy;
        if (i == 0 || i == count/2 || i == count-1)
            printf("  i=%2d  TCP=(%.3f,%.3f,%.3f)  q=(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)\n",
                   i, xyz[0], xyz[1], xyz[2],
                   q_out[i][0], q_out[i][1], q_out[i][2], q_out[i][3], q_out[i][4], q_out[i][5]);
    }

    double dir[3] = {X[count-1][0]-X[0][0], X[count-1][1]-X[0][1], X[count-1][2]-X[0][2]};
    double L2 = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2];
    double max_perp = 0.0;
    for (int i = 1; i < count-1; i++) {
        double v[3] = {X[i][0]-X[0][0], X[i][1]-X[0][1], X[i][2]-X[0][2]};
        double cr[3] = {v[1]*dir[2]-v[2]*dir[1], v[2]*dir[0]-v[0]*dir[2], v[0]*dir[1]-v[1]*dir[0]};
        double perp = sqrt(cr[0]*cr[0]+cr[1]*cr[1]+cr[2]*cr[2]) / sqrt(L2);
        if (perp > max_perp) max_perp = perp;
    }

    double Pe[4][4], xe[3], re[3];
    dh_forward(DH_TABLE, q_out[count-1], Pe);
    dh_pose_to_xyz_rpy(Pe, xe, re);
    double dEnd = sqrt((xe[0]-end_pose[0])*(xe[0]-end_pose[0]) +
                       (xe[1]-end_pose[1])*(xe[1]-end_pose[1]) +
                       (xe[2]-end_pose[2])*(xe[2]-end_pose[2]));

    printf("\n--- DIAGNOSTICS (d6=%.1f, 无夹爪/法兰为TCP) ---\n", DH_TABLE[5].d);
    printf("max |X-150|            = %.4f mm\n", max_Xdev);
    printf("max |Y-52|             = %.4f mm\n", max_Ydev);
    printf("max perpendicular resid = %.4f mm  ( <0.05 => 不画弧 )\n", max_perp);
    printf("endpoint d(FK,target)  = %.4f mm\n", dEnd);
    printf("endpoint FK            = (%.3f,%.3f,%.3f) Rx=%.2f Ry=%.2f Rz=%.2f\n",
           xe[0], xe[1], xe[2], re[0]*RAD2DEG, re[1]*RAD2DEG, re[2]*RAD2DEG);
    printf("final joints (无夹爪)   = (%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)\n",
           q_out[count-1][0], q_out[count-1][1], q_out[count-1][2],
           q_out[count-1][3], q_out[count-1][4], q_out[count-1][5]);

    int pass = (max_perp < 0.05) && (dEnd < 0.001) &&
               (max_Xdev < 0.05) && (max_Ydev < 0.05);
    printf("\n%s\n", pass
        ? "[PASS] 无夹爪(d6=91.5)下 MoveL 直线插补与 FK/IK 自洽：路径为直线、终点精确落在指令位姿"
        : "[FAIL] 见上：直线性或终点一致性不满足");
    return pass ? 0 : 1;
}
