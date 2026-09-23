/*
 * movl_reach.c —— MoveL【下发前】可达性预检（纯离线，不接硬件、不动臂）
 * ------------------------------------------------------------
 * 为什么需要它：2026-09-19 事故 —— 下发 movel 到 (93,52,129)，程序是在
 * 【运动过程中】才发现"第 0 个插补点逆解失败/越软限位"，急停时臂已经跑出去
 * **182mm**、三轴同时越软限位（J2=-73.57 / J3=182.33 / J5=-114.00）。
 * 根因不是规划算法（离线验证规划层残差 0.0000mm，是准的），而是
 * **目标位姿本来就不可达，下发前没人检查**。
 *
 * 本工具把这道检查提到下发前，纯离线跑，零风险。
 *
 * 用法：movl_reach.exe  J1,J2,J3,J4,J5,J6  X,Y,Z,Rx,Ry,Rz
 *   参数1 = 起点关节角（度），直接抄 `getpos` 的那一行
 *   参数2 = 目标位姿，就是你要下发的 movel 前 6 个参数
 *
 * 例：  movl_reach.exe 0,0,110,0,70,0  143,2,114,180,0,180
 *
 * 编译：gcc tests/movl_reach.c src/kinematics/dh.c src/kinematics/ik.c \
 *            src/trajectory/line.c -Isrc -lm -o tests/movl_reach.exe
 *
 * 判据（四条全过才 PASS，任一条挂掉就【不要下发】）：
 *   ① 起点 FK 打印出来 —— 必须和你 getpos 看到的 X/Y/Z 一致（抄错角会全盘错）
 *   ② 每个插补点 IK 有解 且 六轴都在软限位内
 *   ③ 相邻插补点关节角跳变 < MAX_JUMP_DEG（默认 30°，与 ini [safety] 一致）
 *   ④ FK 回代位置 与 规划位姿 偏差 < 0.01mm（IK/FK 自洽）
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
#define MOVL_STEP_MM  1.0      /* 与 commands.c 保持一致 */
#define MAX_JUMP_DEG  30.0     /* 与 ini [safety] max_jump_deg 保持一致 */
#define FK_TOL_MM     0.01

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

/* 由起点关节角 FK 出起点位姿（供 line_plan 使用） */
static void fk_pose(const double q[6], double out[6])
{
    double m[4][4], xyz[3], rpy[3];
    dh_forward(DH_TABLE, q, m);
    dh_pose_to_xyz_rpy(m, xyz, rpy);
    out[0] = xyz[0]; out[1] = xyz[1]; out[2] = xyz[2];
    /* dh_pose_to_xyz_rpy 的 RPY 是【弧度】，位姿口径是度 */
    out[3] = rpy[0] * 180.0 / 3.14159265358979323846;
    out[4] = rpy[1] * 180.0 / 3.14159265358979323846;
    out[5] = rpy[2] * 180.0 / 3.14159265358979323846;
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: %s  J1,J2,J3,J4,J5,J6  X,Y,Z,Rx,Ry,Rz\n", argv[0]);
        printf("  参数1 = 起点关节角（抄 getpos）  参数2 = 目标位姿（movel 前 6 个参数）\n");
        printf("  例: %s 0,0,110,0,70,0 143,2,114,180,0,180\n", argv[0]);
        return 2;
    }

    double q_start[RJ], end_pose[6];
    if (parse6(argv[1], q_start) != 0) { printf("[错误] 参数1 需 6 个关节角，逗号分隔\n"); return 2; }
    if (parse6(argv[2], end_pose) != 0) { printf("[错误] 参数2 需 6 个位姿分量，逗号分隔\n"); return 2; }

    /* ① 起点 FK 自检 */
    double start_pose[6];
    fk_pose(q_start, start_pose);
    printf("MoveL 下发前可达性预检（离线，不动臂）\n");
    printf("------------------------------------------------------------\n");
    printf("起点关节角: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           q_start[0], q_start[1], q_start[2], q_start[3], q_start[4], q_start[5]);
    printf("① 起点 FK : X=%.2f Y=%.2f Z=%.2f  Rx=%.2f Ry=%.2f Rz=%.2f\n",
           start_pose[0], start_pose[1], start_pose[2],
           start_pose[3], start_pose[4], start_pose[5]);
    printf("   ↑ 请核对与 getpos 一致；不一致说明关节角抄错了，后面结论全废\n");

    double d2 = 0.0;
    for (int i = 0; i < 3; i++) { double v = end_pose[i] - start_pose[i]; d2 += v * v; }
    double dist = sqrt(d2);
    printf("目标位姿   : X=%.2f Y=%.2f Z=%.2f  Rx=%.2f Ry=%.2f Rz=%.2f   位移 %.2f mm\n",
           end_pose[0], end_pose[1], end_pose[2],
           end_pose[3], end_pose[4], end_pose[5], dist);
    if (dist < 0.01) { printf("[错误] 位移几乎为 0，没什么可规划的\n"); return 2; }

    /* 规划 */
    int count = line_count_for_distance(dist, MOVL_STEP_MM);
    LinePath path;
    if (line_plan(start_pose, end_pose, count, &path) != 0) {
        printf("[FAIL] line_plan 失败（位姿非有限或点数非法）\n"); return 1;
    }

    JointLimit limits[RJ];
    for (int i = 0; i < RJ; i++) { limits[i].min_deg = LIM_MIN[i]; limits[i].max_deg = LIM_MAX[i]; }

    double (*q_seq)[6] = (double (*)[6])calloc((size_t)path.count, sizeof(*q_seq));
    if (q_seq == NULL) { printf("[错误] 内存不足\n"); return 2; }

    int  fail_idx = -1;
    char fail_reason[128] = {0};
    if (line_solve(&path, DH_TABLE, limits, q_start, q_seq, &fail_idx, fail_reason) != 0) {
        printf("\n[FAIL] ② 第 %d 个插补点求解失败：%s\n", fail_idx,
               fail_reason[0] ? fail_reason : "(无原因)");
        printf("   ⇒ 共 %d 个插补点，目标位姿【不可达】或路径上有越限位点。\n", path.count);
        printf("   ⇒ 【不要下发这条 movel】，换个目标位置或先 movej 挪到附近再试。\n");
        free(q_seq);
        return 1;
    }

    /* ②③④ 逐点检查 */
    double max_jump = 0.0, max_fk = 0.0;
    int    jump_at = -1, fk_at = -1, lim_at = -1;
    double worst_deg = 0.0; int lim_axis = -1;

    for (int i = 0; i < path.count; i++) {
        for (int j = 0; j < RJ; j++) {
            if (q_seq[i][j] < LIM_MIN[j] - 1e-9 || q_seq[i][j] > LIM_MAX[j] + 1e-9) {
                double over = (q_seq[i][j] < LIM_MIN[j]) ? (LIM_MIN[j] - q_seq[i][j])
                                                         : (q_seq[i][j] - LIM_MAX[j]);
                if (over > worst_deg) { worst_deg = over; lim_at = i; lim_axis = j; }
            }
            if (i > 0) {
                double dj = fabs(q_seq[i][j] - q_seq[i - 1][j]);
                if (dj > max_jump) { max_jump = dj; jump_at = i; }
            }
        }
        double m[4][4];
        dh_forward(DH_TABLE, q_seq[i], m);
        double ex = m[0][3] - path.pose[i][0];
        double ey = m[1][3] - path.pose[i][1];
        double ez = m[2][3] - path.pose[i][2];
        double e = sqrt(ex * ex + ey * ey + ez * ez);
        if (e > max_fk) { max_fk = e; fk_at = i; }
    }

    int ok = 1;
    printf("\n插补点数    : %d（步长 %.1f mm）\n", path.count, MOVL_STEP_MM);

    if (lim_at >= 0) {
        printf("② 软限位    : [FAIL] 第 %d 点 关节%d 越限 %.2f°\n", lim_at, lim_axis + 1, worst_deg);
        ok = 0;
    } else {
        printf("② 软限位    : PASS（全部 %d 点在限位内）\n", path.count);
    }

    if (max_jump > MAX_JUMP_DEG) {
        printf("③ 关节跳变  : [FAIL] 最大 %.2f° @第%d点（阈值 %.0f°）⇒ 臂会甩\n",
               max_jump, jump_at, MAX_JUMP_DEG);
        ok = 0;
    } else {
        printf("③ 关节跳变  : PASS（最大 %.2f° @第%d点，阈值 %.0f°）\n", max_jump, jump_at, MAX_JUMP_DEG);
    }

    if (max_fk > FK_TOL_MM) {
        printf("④ FK 回代   : [FAIL] 最大偏差 %.4f mm @第%d点（阈值 %.2f）\n", max_fk, fk_at, FK_TOL_MM);
        ok = 0;
    } else {
        printf("④ FK 回代   : PASS（最大偏差 %.4f mm）\n", max_fk);
    }

    printf("\n================ 判定 ================\n");
    if (ok) {
        printf("PASS —— 这条 movel 路径全程可达、无跳变、IK/FK 自洽，可以下发。\n");
        printf("（注意：本工具只保证【规划层】没问题；实际是否走直线还取决于\n");
        printf("  下发模式：smooth 单段会弯 ~9mm/100mm，step 分 3 段约 1mm。）\n");
        free(q_seq);
        return 0;
    }
    printf("FAIL —— 【不要下发这条 movel】。\n");
    free(q_seq);
    return 1;
}
