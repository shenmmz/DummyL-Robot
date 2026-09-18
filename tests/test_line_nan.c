/*
 * test_line_nan —— 直线插补的 NaN 防线（离线）
 * ------------------------------------------------------------
 * 【2026-09-18 事故：34mm 乱甩的真正根因，不是腕部奇异】
 *
 * 现象：home 位形下发 MoveL，末端偏差 34mm、臂乱甩、最后超时急停，
 *       日志里出现「还差 ±2147228222 步」这种天文数字。
 *
 * 根因链（本测试逐环节守住）：
 *   ① home 姿态 Ry 恰好 = 90°，正落在万向锁（gimbal lock）上。
 *   ② line_plan 用四元数 SLERP 插值姿态，再 quat_to_euler 转回 RPY：
 *        pitch = asin(2*(q0*q2 - q1*q3))
 *      pitch=90° 时该自变量理论上是 1，但插值出来的四元数带浮点误差，
 *      实测算成 **1.0000000000000002** ⇒ asin 越界直接返回 **NaN**。
 *      （探针实测：11 个点里前 5 个 sp=1 精确，后 6 个 sp=1.0000000000000002）
 *   ③ NaN 位姿 → IK 解出 NaN 关节角 → DEG2STEPS 转 int32 变成
 *      ±2147483647 附近的垃圾 → 写进驱动器的目标位置是天文数字
 *      → 电机朝那个方向猛冲、永远到不了位 → 超时急停 → 臂被甩出去。
 *
 * 三道防线（本测试逐条验证）：
 *   A. line.c  quat_to_euler：asin 自变量先 clamp 到 [-1,1]（根治）
 *   B. line.c  line_plan：逐点查位姿有限性，非有限即返回失败（不让 NaN 出门）
 *   C. cli     movej_issue：下发前查目标角有限性，非有限即拒绝下发
 *      （最后一道闸：任何一路 NaN 的代价都是撞机，宁可不动也不发垃圾）
 *
 * 【为什么必须单独测】
 *   home 位形是本机的默认停放姿态（J5=0 且 Ry=90°），一开机就踩在这上面。
 *   这种"默认状态即触发"的 bug，只有专门构造万向锁位姿才测得出来。
 *
 * 运行：cmake --build build --target test_line_nan && build/bin/test_line_nan.exe
 */

#include <stdio.h>
#include <math.h>
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "kinematics/dh_params.h"
#include "trajectory/line.h"
#include "config/robot_config.h"

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-46s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

/* 走完整条 line_plan → line_solve 链路，检查：
 *   ① 每个插补位姿都有限（防线 A+B）
 *   ② 每个逆解关节角都有限（防线 C 的输入侧）
 * 返回 1 = 全程干净 */
static int run_path(const char *name, const double start6[6], const double end6[6],
                    const double q_start[6], int count, char *why, size_t why_sz)
{
    LinePath path;
    double q_seq[LINE_MAX_POINTS][6];
    JointLimit lim[6];
    const double lmin[6] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[6] = ROBOT_JOINT_LIMIT_MAX_DEG;
    int i, j, fail_idx = -1;
    char fail_reason[64] = {0};

    for (j = 0; j < 6; j++) { lim[j].min_deg = lmin[j]; lim[j].max_deg = lmax[j]; }

    if (line_plan(start6, end6, count, &path) != 0) {
        snprintf(why, why_sz, "line_plan 返回失败（NaN 防线拦下）");
        return 0;
    }
    for (i = 0; i < path.count; i++)
        for (j = 0; j < 6; j++)
            if (!isfinite(path.pose[i][j])) {
                snprintf(why, why_sz, "插补点 %d 的 pose[%d] 非有限（%.6g）", i, j, path.pose[i][j]);
                return 0;
            }
    if (line_solve(&path, DH_TABLE, lim, q_start, q_seq, &fail_idx, fail_reason) != 0) {
        snprintf(why, why_sz, "line_solve 失败 @%d：%s", fail_idx, fail_reason);
        return 0;
    }
    for (i = 0; i < path.count; i++)
        for (j = 0; j < 6; j++)
            if (!isfinite(q_seq[i][j])) {
                snprintf(why, why_sz, "逆解点 %d 的 J%d 非有限（%.6g）", i, j + 1, q_seq[i][j]);
                return 0;
            }
    (void)name;
    return 1;
}

int main(void)
{
    /* home 位形：J1..J6 = 0,0,90,0,0,0。**其姿态 Ry == 90°，正踩在万向锁上** ——
     * 这就是 2026-09-18 那次 34mm 事故的现场姿态。 */
    double q_home[6] = { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 };
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double m[4][4], start6[6], rpy[3];
    char why[128];

    dh_set_tool_length(0.0);
    dh_forward(DH_TABLE, q_home, m);
    dh_pose_to_xyz_rpy(m, start6, rpy);
    start6[3] = rpy[0] * RAD2DEG;
    start6[4] = rpy[1] * RAD2DEG;
    start6[5] = rpy[2] * RAD2DEG;

    printf("home 位姿：X=%.2f Y=%.2f Z=%.2f  Rx=%.4f Ry=%.4f Rz=%.4f\n",
           start6[0], start6[1], start6[2], start6[3], start6[4], start6[5]);
    printf("⇒ Ry %s 90°，%s\n\n",
           fabs(fabs(start6[4]) - 90.0) < 1e-6 ? "==" : "!=",
           fabs(fabs(start6[4]) - 90.0) < 1e-6 ? "正踩万向锁（事故现场）" : "未踩万向锁");

    printf("=== 1. 万向锁位姿下插补不得产出 NaN ===\n");
    {
        /* 姿态不变的纯平移：事故现场原样（-Z 10mm） */
        struct { const char *nm; double d[3]; } cases[] = {
            { "-Z 10mm（事故现场原样）", {  0.0,  0.0, -10.0 } },
            { "+X 50mm",                 { 50.0,  0.0,   0.0 } },
            { "+Y 30mm",                 {  0.0, 30.0,   0.0 } },
            { "+Z 20mm",                 {  0.0,  0.0,  20.0 } },
        };
        size_t k;
        for (k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
            double end6[6];
            int j;
            for (j = 0; j < 6; j++) end6[j] = start6[j];
            end6[0] += cases[k].d[0];
            end6[1] += cases[k].d[1];
            end6[2] += cases[k].d[2];
            {
                int ok = run_path(cases[k].nm, start6, end6, q_home, 11, why, sizeof why);
                check(cases[k].nm, ok, ok ? "全程位姿与逆解均有限" : why);
            }
        }
    }

    printf("\n=== 2. 用【另一组等价 RPY】描述同一姿态，也不能出 NaN ===\n");
    {
        /* 万向锁下 Rx/Rz 不唯一（Rx-Rz 相同即可），真机 getpos 读回的是
         * (-90, 90, -90)，FK 算出来的是 (0, 90, 0) —— 同一个姿态两种写法。
         * 混用这两种写法做插值最容易把 asin 顶出界，必须都试。 */
        double alt6[6];
        double end6[6];
        int j;
        for (j = 0; j < 3; j++) alt6[j] = start6[j];
        alt6[3] = -90.0; alt6[4] = 90.0; alt6[5] = -90.0;

        for (j = 0; j < 6; j++) end6[j] = alt6[j];
        end6[2] -= 10.0;
        {
            int ok = run_path("等价 RPY 写法", alt6, end6, q_home, 11, why, sizeof why);
            check("起点 (Rx,Ry,Rz)=(-90,90,-90)", ok, ok ? "全程有限" : why);
        }
        /* 起点用 FK 写法、终点用 getpos 写法 —— 混着来 */
        {
            int ok = run_path("混用", start6, end6, q_home, 11, why, sizeof why);
            check("起点 FK 写法 + 终点 getpos 写法", ok, ok ? "全程有限" : why);
        }
    }

    printf("\n=== 3. 扫出一个真正落在 Ry≈-90° 的位形，另一侧万向锁同样要守住 ===\n");
    {
        /* 万向锁是 Ry = ±90° 两侧都有。第 1、2 节只覆盖了 +90°
         * （home 姿态），这一节扫出一个 Ry≈-90° 的位形补上另一侧。 */
        const double j1s[] = { 0.0, 45.0 };
        const double j2s[] = { -60.0, -30.0, 0.0, 30.0, 60.0 };
        const double j3s[] = { 30.0, 60.0, 90.0, 120.0, 150.0 };
        const double j5s[] = { -90.0, -45.0, 0.0, 45.0, 90.0 };
        size_t a, b, c, d;
        int found = 0;

        for (a = 0; a < 2 && !found; a++)
        for (b = 0; b < 5 && !found; b++)
        for (c = 0; c < 5 && !found; c++)
        for (d = 0; d < 5 && !found; d++) {
            double q2[6] = { j1s[a], j2s[b], j3s[c], 0.0, j5s[d], 0.0 };
            double s6[6], e6[6];
            int j;
            dh_forward(DH_TABLE, q2, m);
            dh_pose_to_xyz_rpy(m, s6, rpy);
            s6[3] = rpy[0] * RAD2DEG; s6[4] = rpy[1] * RAD2DEG; s6[5] = rpy[2] * RAD2DEG;
            if (s6[4] > -89.5 || s6[4] < -90.5) continue;   /* 只要 Ry≈-90° */
            found = 1;
            printf("  找到 J=[%.0f,%.0f,%.0f,0,%.0f,0] ⇒ Ry=%.4f°\n",
                   q2[0], q2[1], q2[2], q2[4], s6[4]);
            for (j = 0; j < 6; j++) e6[j] = s6[j];
            e6[2] -= 8.0;
            {
                int ok = run_path("Ry=-90", s6, e6, q2, 11, why, sizeof why);
                check("Ry≈-90° 侧万向锁插补", ok, ok ? "全程有限" : why);
            }
        }
        if (!found) check("未扫到 Ry≈-90° 的位形", 0, "请扩大扫描范围，别让这一侧裸奔");
    }

    printf("\n=== 4. asin 越界的直接验证：pitch 必须恒为 ±90°，不能变 NaN ===\n");
    {
        /* 极端构造：起点终点描述【同一个姿态】（万向锁下两种 RPY 写法），
         * SLERP 应恒等，pitch 全程必须 = +90°。这条直接盯住 asin 的自变量 ——
         * 事故时它算出 1.0000000000000002，asin 越界返回 NaN。 */
        double s6[6], e6[6];
        LinePath path;
        int i, bad = 0;
        double worst_dev = 0.0;

        for (i = 0; i < 3; i++) s6[i] = start6[i];
        s6[3] = 0.0;   s6[4] = 90.0;  s6[5] = 0.0;
        for (i = 0; i < 3; i++) e6[i] = start6[i];
        e6[3] = -90.0; e6[4] = 90.0;  e6[5] = -90.0;
        e6[2] -= 10.0;

        if (line_plan(s6, e6, 21, &path) != 0) {
            check("恒等姿态插值", 0, "line_plan 失败");
        } else {
            for (i = 0; i < path.count; i++) {
                double p = path.pose[i][4];
                double dev;
                if (!isfinite(p)) { bad = 1; continue; }
                dev = fabs(p - 90.0);
                if (dev > worst_dev) worst_dev = dev;
            }
            {
                char buf[128];
                snprintf(buf, sizeof buf, "%d 点 pitch 全部有限，最大偏离 90° = %.3e°",
                         path.count, worst_dev);
                check("pitch 全程有限且 = +90°", !bad && worst_dev < 1e-9, buf);
            }
        }
    }

    printf("\n%s（失败项 %d）\n",
           g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
