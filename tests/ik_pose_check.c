/*
 * ik_pose_check.c —— 单点位姿 ↔ 关节角 双向核对（纯离线，不动臂）
 * ------------------------------------------------------------
 * 用途：用户实测一个位姿（getpos 给出的 X/Y/Z/Rx/Ry/Rz + 当时的 J1..J6），
 *       本工具回答两个问题：
 *   Q1  FK 自洽吗？  实际关节角 FK 出来的位姿，是否等于 getpos 报的位姿？
 *   Q2  IK 能解出来吗？  从该位姿反解，8 组候选里【有没有】实际那一组？
 *
 * 为什么重要：2026-09-19 用户实测 —— 下发 movej:0,0,90,0,90,0（目标 J3=90、
 * J5=90），失能后实测 J3=133.07、J5=47.54，而 J2+J3+J5 仍是 180.79
 * （法兰仍竖直朝下，倾角 0.89°）。也就是说【同一个"竖直朝下"姿态对应
 * 完全不同的 (J3,J5) 分配】。如果 IK 解不出实际的那一组，程序规划出来的
 * 路径就会跟臂的真实构型对不上 ⇒ 表现为"乱跑/画斜线"。
 *
 * 用法：ik_pose_check.exe  X,Y,Z,Rx,Ry,Rz  J1,J2,J3,J4,J5,J6
 * 例：  ik_pose_check.exe 117.96,51.36,115.73,-179.59,-0.79,-179.61 0.01,0.18,133.07,-0.56,47.54,0
 *
 * 编译：gcc tests/ik_pose_check.c src/kinematics/dh.c src/kinematics/ik.c \
 *            src/trajectory/line.c -Isrc -lm -o tests/ik_pose_check.exe
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"
#include "config/robot_config.h"

#define RJ 6
static const double LIM_MIN[RJ] = ROBOT_JOINT_LIMIT_MIN_DEG;
static const double LIM_MAX[RJ] = ROBOT_JOINT_LIMIT_MAX_DEG;

static int parse6(const char *s, double v[6])
{
    char buf[256];
    if (strlen(s) >= sizeof(buf)) return -1;
    strcpy(buf, s);
    int n = 0;
    char *tok = strtok(buf, ",");
    while (tok != NULL && n < 6) {
        char *end = NULL;
        v[n] = strtod(tok, &end);
        if (end == tok) return -1;
        n++;
        tok = strtok(NULL, ",");
    }
    return (n == 6) ? 0 : -1;
}

static void print_j(const char *tag, const double q[6])
{
    printf("%s %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
           tag, q[0], q[1], q[2], q[3], q[4], q[5]);
}

static void fk_xyz(const double q[6], double xyz[3])
{
    double m[4][4];
    dh_forward(DH_TABLE, q, m);
    xyz[0] = m[0][3]; xyz[1] = m[1][3]; xyz[2] = m[2][3];
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: %s  X,Y,Z,Rx,Ry,Rz  J1,J2,J3,J4,J5,J6\n", argv[0]);
        printf("  参数1 = getpos 报的位姿   参数2 = 同一时刻 getpos 报的关节角\n");
        return 2;
    }
    double pose[6], actual[6];
    if (parse6(argv[1], pose)   != 0) { printf("[错误] 参数1 需 6 个位姿分量\n"); return 2; }
    if (parse6(argv[2], actual) != 0) { printf("[错误] 参数2 需 6 个关节角\n"); return 2; }

    printf("位姿 vs 关节角 双向核对（离线）\n");
    printf("------------------------------------------------------------\n");
    printf("给定位姿 : X=%.2f Y=%.2f Z=%.2f  Rx=%.2f Ry=%.2f Rz=%.2f\n",
           pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    printf("实际关节 :");
    print_j("         ", actual);

    /* Q1：实际关节角 FK 出来，等于给定位姿吗 */
    double xyz[3];
    fk_xyz(actual, xyz);
    double dpos = sqrt((xyz[0]-pose[0])*(xyz[0]-pose[0])
                     + (xyz[1]-pose[1])*(xyz[1]-pose[1])
                     + (xyz[2]-pose[2])*(xyz[2]-pose[2]));
    printf("\nQ1 FK 自洽: 实际关节 FK => X=%.2f Y=%.2f Z=%.2f，与给定位姿差 %.3f mm  %s\n",
           xyz[0], xyz[1], xyz[2], dpos, dpos < 0.5 ? "PASS" : "[FAIL 关节角抄错或DH不对]");

    /* Q2：从位姿反解，8 组里有没有实际那一组 */
    double m[4][4];
    line_pose_to_matrix(pose, m);
    double sols[IK_MAX_SOLUTIONS][6];
    IkSolInfo info[IK_MAX_SOLUTIONS];
    int n = ik_solve_ex(DH_TABLE, m, sols, info);
    printf("\nQ2 IK 反解: 共 %d 组候选解\n", n);
    printf("   #  J1      J2      J3      J4      J5      J6   | 限位 | 与实际最大差 | FK回代\n");

    int best = -1; double best_err = 1e18;
    for (int i = 0; i < n; i++) {
        double maxd = 0.0;
        for (int j = 0; j < RJ; j++) {
            double d = fabs(sols[i][j] - actual[j]);
            if (d > maxd) maxd = d;
        }
        int inlim = 1;
        for (int j = 0; j < RJ; j++)
            if (sols[i][j] < LIM_MIN[j] - 1e-9 || sols[i][j] > LIM_MAX[j] + 1e-9) inlim = 0;

        double x2[3]; fk_xyz(sols[i], x2);
        double fke = sqrt((x2[0]-pose[0])*(x2[0]-pose[0])
                        + (x2[1]-pose[1])*(x2[1]-pose[1])
                        + (x2[2]-pose[2])*(x2[2]-pose[2]));
        printf("  %2d", i);
        print_j(" ", sols[i]);
        printf("   |  %s  |  %8.2f°  | %.3f mm\n", inlim ? "OK " : "越限", maxd, fke);
        if (inlim && maxd < best_err) { best_err = maxd; best = i; }
    }

    printf("\n================ 判定 ================\n");
    if (best < 0) {
        printf("没有一组候选解同时满足【全在软限位内】。\n");
        printf("⇒ 该位姿按当前 DH/IK 模型【不可达】，与实际能摆出来的构型矛盾。\n");
        return 1;
    }
    printf("最接近实际的候选解 = 第 %d 组，与实际关节角最大差 %.2f°\n", best, best_err);
    if (best_err <= 1.0) {
        printf("PASS —— IK 能解出实际构型（差 <1°），模型自洽。\n");
        printf("若运动仍跑偏，问题在【分支选择】（ik_select_best_continuous 选了另一支），\n");
        printf("不在 IK/DH 模型本身。\n");
        return 0;
    }
    printf("[FAIL] IK 解不出实际构型（差 %.2f° ≫ 1°）。\n", best_err);
    printf("⇒ 说明同一个位姿，臂实际摆出来的 (J3,J5) 分配 与 IK 解出的不一致。\n");
    printf("   程序按自己那组去规划 ⇒ 下发的关节角目标与臂真实构型差这么多 ⇒\n");
    printf("   表现就是'乱跑/画斜线'。**这是模型层问题，不是下发模式问题。**\n");
    return 1;
}
