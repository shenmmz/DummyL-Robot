/*
 * test_z170 —— MoveL 规划链路端到端回归（离线）
 *
 * 复现用户场景：笛卡尔空间一条纯 Z 下降（笔垂直），走完整条规划链路
 *   pose6 → line_plan（位置线性 + 姿态 SLERP）→ line_solve（逐点 IK + 连续选解）
 *   → FK 回算
 *
 * 每一步各验一条，任何一环坏了都能单独定位：
 *   1) line_count_for_distance：点数 = ceil(dist/step)+1，且被夹在 [2,257]
 *   2) line_plan：首尾必须精确等于起终点；中间点必须**严格落在直线上**
 *      （这是"直线插补"的核心不变式，SLERP 只管姿态、不该污染位置）
 *   3) line_solve：全部点有解，且 FK(q[i]) 还原出 path.pose[i]（≤0.01mm）
 *   4) 关节连续：相邻点不得出现 >20° 跳变（跳了就说明选到腕翻转远分支）
 *   5) 笔保持竖直：整条路径法兰 z 轴必须始终 ≈ (0,0,-1)
 *
 * 运行：cmake --build build --target test_z170 && build/bin/test_z170.exe
 */

#include <stdio.h>
#include <math.h>
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-46s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

int main(void)
{
    /* 纯 Z 下降：X/Y 与姿态全程不变，只有 Z 从 72.5 → 22.5
     * ⚠️ 原来是 164 → 114（用户画线场景）。d6 从 91.5 定案到 183 后，同一个
     *    物理姿态的 TCP 读数整体下移 91.5mm ⇒ 这里同步减 91.5 才是同一条轨迹。
     *    不减的话起点 Z=164 在 d6=183 下【不可达】（X=150,Y=52 笔朝下时合规
     *    Z 上限只有 ~113mm，受 J5 软限位 ±95° 卡住）。 */
    const double start[6] = {150.0, 52.0,  72.5, -180.0, 0.0, -180.0};
    const double end[6]   = {150.0, 52.0,  22.5, -180.0, 0.0, -180.0};
    const JointLimit limits[6] = {
        {-170.0, 179.0}, {-72.0, 90.0}, {30.0, 180.0},
        {-360.0, 360.0}, {-95.0, 95.0}, {-360.0, 360.0}
    };
    LinePath path;
    double q[LINE_MAX_POINTS][6];
    double start_q[6] = {0.0, 0.0, 90.0, 0.0, 0.0, 0.0};
    char buf[160];
    int count, i, fail_idx, ok;
    char reason[64];
    double dist, max_off_line = 0.0, max_fk_err = 0.0, max_jump = 0.0, max_tilt = 0.0;

    dh_set_tool_length(0.0);

    dist = sqrt((end[0]-start[0])*(end[0]-start[0]) +
                (end[1]-start[1])*(end[1]-start[1]) +
                (end[2]-start[2])*(end[2]-start[2]));
    printf("纯 Z 下降 %.1f mm：(%.0f,%.0f,%.0f) → (%.0f,%.0f,%.0f)\n\n",
           dist, start[0], start[1], start[2], end[0], end[1], end[2]);

    printf("=== 1. line_count_for_distance ===\n");
    count = line_count_for_distance(dist, 2.0);
    snprintf(buf, sizeof buf, "dist=%.1f step=2.0 ⇒ %d 点（期望 26）", dist, count);
    check("点数计算", count == 26, buf);
    {
        int c0 = line_count_for_distance(0.0, 2.0);
        int c1 = line_count_for_distance(10000.0, 0.5);
        snprintf(buf, sizeof buf, "0mm→%d 点, 10000mm/step0.5→%d 点（上限 %d）",
                 c0, c1, LINE_MAX_POINTS);
        check("边界：0 与超长都被夹住", c0 >= 2 && c1 <= LINE_MAX_POINTS, buf);
    }

    printf("\n=== 2. line_plan：首尾精确 + 中间点严格在直线上 ===\n");
    if (line_plan(start, end, count, &path) != 0) {
        printf("  [FAIL] line_plan 返回失败\n");
        g_fail++;
        return 1;
    }
    ok = (fabs(path.pose[0][0] - start[0]) < 1e-9 &&
          fabs(path.pose[0][1] - start[1]) < 1e-9 &&
          fabs(path.pose[0][2] - start[2]) < 1e-9 &&
          fabs(path.pose[count-1][0] - end[0]) < 1e-9 &&
          fabs(path.pose[count-1][1] - end[1]) < 1e-9 &&
          fabs(path.pose[count-1][2] - end[2]) < 1e-9);
    check("首尾精确等于起终点", ok, NULL);

    /* 点到直线距离：|(p-p0) × dir| / |dir| */
    {
        double dir[3], len;
        int j;
        for (j = 0; j < 3; j++) dir[j] = end[j] - start[j];
        len = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
        for (i = 0; i < count; i++) {
            double v[3], cx[3], n;
            for (j = 0; j < 3; j++) v[j] = path.pose[i][j] - start[j];
            cx[0] = v[1]*dir[2] - v[2]*dir[1];
            cx[1] = v[2]*dir[0] - v[0]*dir[2];
            cx[2] = v[0]*dir[1] - v[1]*dir[0];
            n = sqrt(cx[0]*cx[0] + cx[1]*cx[1] + cx[2]*cx[2]) / len;
            if (n > max_off_line) max_off_line = n;
        }
    }
    snprintf(buf, sizeof buf, "最大离直线距离 %.3e mm", max_off_line);
    check("所有插补点落在直线上", max_off_line < 1e-9, buf);

    printf("\n=== 3. line_solve + FK 回算 ===\n");
    if (line_solve(&path, DH_TABLE, limits, start_q, q, &fail_idx, reason) != 0) {
        printf("  [FAIL] 第 %d 点求解失败：%s\n", fail_idx, reason);
        g_fail++;
        return 1;
    }
    check("全部点 IK 有解且在软限位内", 1, NULL);

    for (i = 0; i < count; i++) {
        double T[4][4];
        double e;
        dh_forward(DH_TABLE, q[i], T);
        e = sqrt((T[0][3]-path.pose[i][0])*(T[0][3]-path.pose[i][0]) +
                 (T[1][3]-path.pose[i][1])*(T[1][3]-path.pose[i][1]) +
                 (T[2][3]-path.pose[i][2])*(T[2][3]-path.pose[i][2]));
        if (e > max_fk_err) max_fk_err = e;
    }
    snprintf(buf, sizeof buf, "FK 还原最大误差 %.3e mm", max_fk_err);
    check("FK(q[i]) 还原 path.pose[i]", max_fk_err < 0.01, buf);

    printf("\n=== 4. 关节连续性（不得跳到远分支）===\n");
    for (i = 1; i < count; i++) {
        int j;
        for (j = 0; j < 6; j++) {
            double d = fabs(q[i][j] - q[i-1][j]);
            if (d > max_jump) max_jump = d;
        }
    }
    snprintf(buf, sizeof buf, "相邻点最大关节变化 %.3f°", max_jump);
    check("无 20° 以上跳变", max_jump < 20.0, buf);

    printf("\n=== 5. 笔保持竖直（法兰 z 轴始终朝下）===\n");
    for (i = 0; i < count; i++) {
        double T[4][4];
        double dot;
        dh_forward(DH_TABLE, q[i], T);
        /* 法兰 z 轴 = R 的第 3 列，期望 (0,0,-1) */
        dot = T[0][2]*0.0 + T[1][2]*0.0 + T[2][2]*(-1.0);
        if (fabs(1.0 - dot) > max_tilt) max_tilt = fabs(1.0 - dot);
    }
    snprintf(buf, sizeof buf, "最大偏离竖直 %.3e（1-cos）", max_tilt);
    check("全程竖直", max_tilt < 1e-6, buf);

    printf("\n  起点关节：");
    for (i = 0; i < 6; i++) printf("%s%.2f", i ? " " : "", q[0][i]);
    printf("\n  终点关节：");
    for (i = 0; i < 6; i++) printf("%s%.2f", i ? " " : "", q[count-1][i]);
    printf("\n");

    printf("\n%s（失败项 %d）\n", g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
