/*
 * test_ik_wrist —— 腕部奇异（J5≈0）专项回归测试（离线）
 * ------------------------------------------------------------
 * 【为什么单独建一个文件，而不是塞进 test_ik_singular】
 *   2026-09-18 真机事故：home 位形（J1..J6 = 0,0,90,0,0,0，其中 **J5=0**）
 *   下发 MoveL，末端实测偏差 **34.38mm**，臂直接乱甩。
 *
 *   根因：ik.c 的腕奇异判据用的是 IK_EPS = 1e-10，要 |sinθ5| < 1e-10
 *   才触发，即 θ5 < 5.7e-9° —— 等于【根本没有检测】。于是
 *       theta4 = atan2(r23/sinth5, r13/sinth5)
 *   在 sinth5=0 时是 0/0，J4/J6 在 8 组解之间乱跳。
 *
 *   test_ik_singular 第 3 节抓不到它：那条只断言"至少有 1 组解能还原位姿"，
 *   而事故时 IK 确实能给出能还原位姿的解（就是那 8 组乱跳的解之一）。
 *   **必须专门测解的"稳定性"，光测"有没有解"没用。**
 *
 * 【修了两版，第二版才对 —— 这段教训写在这里，避免有人再改回去】
 *   第一版把判定阈值定在 sin(2°)≈0.0349，属于矫枉过正。正常公式在
 *   θ5=1° 时数值上很健康（sinθ5=0.017，而 r36 的浮点噪声只 1e-16），
 *   根本不需要退化处理。强行判成奇异、把 θ5 压成 0，反倒自己造了两个新毛病：
 *     ① 姿态被硬改 ⇒ 经 d6 杠杆放大成 **1.82mm** 末端偏差（当时 d6=91.5，现已定案 183）；
 *     ② θ5 越过 2° 边界那一刻，θ4 从"强制的 0"跳到"真实的 -89.795°"
 *        ⇒ **J4 单步跳变 89.8°**，臂照甩不误。
 *   第二版的阈值改由【数值分辨极限】决定（见 ik.c 的 IK_WRIST_SINGULAR_SIN
 *   注释）：sinth5 = sqrt(1-cos²θ5) 在 double 下低于 ~4.7e-8 就恒等于 0，
 *   信息在减法里被抵消光。故阈值取 1e-7，且：
 *     - θ5 保留真实值（atan2 良态，不压 0）；
 *     - θ4 锚到【上一点的 θ4】（ik_solve_ref 传入），θ6 = (θ4+θ6) - θ4。
 *
 * 【本测试不测什么】
 *   从【精确奇异点】出发的 MoveL，J4 本来就需要一步转 89.9° —— 那是奇异
 *   位形的物理本质（θ5 一离开 0，θ4 就被位姿唯一锁定），不是数值 bug。
 *   所以第 4 节只测"末端精确 + 首段之后恢复连续"，不断言"首段不跳"。
 *   规避手段在规划层（movl_plan 的 |J5|<5° 预警），不在这里。
 *
 * 运行：cmake --build build --target test_ik_wrist && build/bin/test_ik_wrist.exe
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
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

/* 关节软限位：与 cmd_movel 里用的一模一样 */
static void make_limits(JointLimit lim[6])
{
    const double lmin[6] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[6] = ROBOT_JOINT_LIMIT_MAX_DEG;
    int j;
    for (j = 0; j < 6; j++) {
        lim[j].min_deg = lmin[j];
        lim[j].max_deg = lmax[j];
    }
}

/* FK -> 4x4 矩阵；顺便检查是否含 NaN */
static int fk_ok(const double q[6], double T[4][4])
{
    int r, c;
    dh_forward(DH_TABLE, q, T);
    for (r = 0; r < 4; r++)
        for (c = 0; c < 4; c++)
            if (!isfinite(T[r][c])) return 0;
    return 1;
}

/* 两个位姿矩阵的最大逐元素误差（平移单位 mm，旋转无量纲） */
static double pose_max_err(const double A[4][4], const double B[4][4])
{
    double m = 0.0;
    int r, c;
    for (r = 0; r < 4; r++)
        for (c = 0; c < 4; c++) {
            double d = fabs(A[r][c] - B[r][c]);
            if (d > m) m = d;
        }
    return m;
}

/* ====== 1. 奇异判据的边界 ======
 * 阈值 1e-7 由数值分辨极限推出，不是拍脑袋取的角度。
 * 这一节守住"别再放宽"—— 放宽就会退化成第一版的 1.82mm 事故。 */
static void test_threshold(void)
{
    const double j5_cases[]  = { 0.0, 0.0005, 0.01, 1.0, 30.0 };
    const int    want_sing[] = {   1,      0,    0,   0,    0 };
    size_t k;

    printf("=== 1. 腕奇异判据边界（阈值 |sinθ5| < 1e-7）===\n");
    printf("  %-10s %-8s %-8s %-8s %s\n", "J5(°)", "解数", "奇异", "有效", "期望");
    for (k = 0; k < sizeof(j5_cases) / sizeof(j5_cases[0]); k++) {
        double q[6] = { 30.0, 20.0, 110.0, 50.0, j5_cases[k], -20.0 };
        double T[4][4], sol[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        int n, i, n_sing = 0, n_valid = 0, ok;
        char buf[110];

        if (!fk_ok(q, T)) { check("FK 出 NaN", 0, NULL); continue; }
        n = ik_solve_ex(DH_TABLE, T, sol, info);
        for (i = 0; i < n; i++) {
            if (info[i].wrist == IK_SOL_SINGULAR) n_sing++;
            if (info[i].wrist == IK_SOL_VALID)    n_valid++;
        }
        printf("  %-10.4f %-8d %-8d %-8d %s\n", j5_cases[k], n, n_sing, n_valid,
               want_sing[k] ? "应含奇异" : "不应有奇异");
        snprintf(buf, sizeof buf, "J5=%.4f°：解 %d 组，奇异 %d / 有效 %d",
                 j5_cases[k], n, n_sing, n_valid);
        /* 只要求"该判奇异的那条分支被判了"，不要求全部分支都判 —— 
         * 8 组解里只有真实 θ5≈0 的那一个肩肘组合才奇异，其余是正常解。 */
        ok = want_sing[k] ? (n_sing > 0) : (n_sing == 0 && n_valid > 0);
        check("  └ 判定正确", ok, buf);
    }
}

/* ====== 2. 奇异点：wrist 双分支必须合流，且 θ4 锚到参考角 ====== */
static void test_converge(void)
{
    double q_home[6] = { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 };   /* home，J5=0 */
    double T[4][4];
    char buf[160];

    printf("\n=== 2. 奇异点解收敛 + θ4 锚定参考（home，J5=0）===\n");
    if (!fk_ok(q_home, T)) { check("FK 出 NaN", 0, NULL); return; }

    /* --- 2a. 不传参考：两条 wrist 分支必须给出【同一组解】 --- */
    {
        double sol[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        int n, i, n_sing = 0, pair_same = 0;
        double sing_j4 = 0.0, sing_j5 = 0.0;
        int have_pair = 0;

        n = ik_solve_ref(DH_TABLE, T, NULL, sol, info);
        printf("  [无参考] 候选解 %d 组：\n", n);
        for (i = 0; i < n; i++) {
            printf("    #%d J=[%8.3f %8.3f %8.3f | %9.3f %9.3f %9.3f] 腕=%s\n", i,
                   sol[i][0], sol[i][1], sol[i][2], sol[i][3], sol[i][4], sol[i][5],
                   ik_sol_status_str(info[i].wrist));
            if (info[i].wrist == IK_SOL_SINGULAR) {
                n_sing++;
                if (!have_pair) { sing_j4 = sol[i][3]; sing_j5 = sol[i][4]; have_pair = 1; }
                else if (fabs(sol[i][3] - sing_j4) < 1e-9 &&
                         fabs(sol[i][4] - sing_j5) < 1e-9) pair_same = 1;
            }
        }
        snprintf(buf, sizeof buf, "%d 组中奇异 %d 组（应为 2：同一肩肘组合的 wrist 双分支）",
                 n, n_sing);
        check("奇异解恰为 2 组（wrist 双分支）", n_sing == 2, buf);
        check("两条 wrist 分支解完全相同（已合流）", pair_same, NULL);
        snprintf(buf, sizeof buf, "奇异解 J4=%.6f J5=%.6f", sing_j4, sing_j5);
        check("无参考时 θ4 取 0、θ5 保留真实值 0",
              fabs(sing_j4) < 1e-9 && fabs(sing_j5) < 1e-9, buf);
    }

    /* --- 2b. 传参考：θ4 必须跟着参考走，而不是糙取 0 ---
     * 这条是第一版 bug 的照妖镜：硬取 0 会让路径在越过判定边界时
     * J4 一步跳 89.8°。 */
    {
        const double ref_cases[] = { -90.0, -89.795, 45.0, 180.0, -170.0 };
        size_t k;
        for (k = 0; k < sizeof(ref_cases) / sizeof(ref_cases[0]); k++) {
            double ref[6] = { 0.0, 0.0, 90.0, ref_cases[k], 0.0, 0.0 };
            double sol[IK_MAX_SOLUTIONS][6];
            IkSolInfo info[IK_MAX_SOLUTIONS];
            int n, i, n_sing = 0, all_match = 1;
            double worst_err = 0.0;

            n = ik_solve_ref(DH_TABLE, T, ref, sol, info);
            for (i = 0; i < n; i++) {
                if (info[i].wrist != IK_SOL_SINGULAR) continue;
                n_sing++;
                if (fabs(sol[i][3] - ref_cases[k]) > 1e-9) all_match = 0;
                {
                    double Ts[4][4], e;
                    if (!fk_ok(sol[i], Ts)) worst_err = 1e30;
                    else {
                        e = pose_max_err(Ts, T);
                        if (e > worst_err) worst_err = e;
                    }
                }
            }
            snprintf(buf, sizeof buf, "参考 J4=%.3f° ⇒ 奇异解 J4 同值，位姿误差 %.2e",
                     ref_cases[k], worst_err);
            check("  └ θ4 锚到参考值", n_sing == 2 && all_match, buf);
            snprintf(buf, sizeof buf, "参考 J4=%.3f°（误差 %.2e）", ref_cases[k], worst_err);
            check("  └ 锚定后位姿仍精确（<1e-6）", worst_err < 1e-6, buf);
        }
    }

    /* --- 2c. θ6 必须补上差额：θ4+θ6 守恒 --- */
    {
        double ref[6] = { 0.0, 0.0, 90.0, -75.0, 0.0, 0.0 };
        double sol[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        int n, i;
        double sum_ref = -75.0 + 75.0;   /* 参考解里 θ4+θ6 = 0（home J4=J6=0） */
        int sum_ok = 1;

        n = ik_solve_ref(DH_TABLE, T, ref, sol, info);
        for (i = 0; i < n; i++) {
            if (info[i].wrist != IK_SOL_SINGULAR) continue;
            if (fabs((sol[i][3] + sol[i][5]) - sum_ref) > 1e-6) sum_ok = 0;
        }
        snprintf(buf, sizeof buf, "奇异解 θ4+θ6 应为 %.3f°（home 时为 0）", sum_ref);
        check("θ4+θ6 守恒（末端姿态不受 θ4 取值影响）", sum_ok, buf);
    }
}

/* ====== 3. 干净路径（全程避开奇异区）：末端必须精确 + 关节必须连续 ======
 * 这是"过流保护会不会拖累精度"等一切后续回归的基线。
 * 起点 J5=20°（远离奇异），走三条直线。 */
static void test_clean_path(void)
{
    struct Case { const char *name; double d[3]; } cases[] = {
        { "-Z 10mm", {  0.0,  0.0, -10.0 } },
        { "+X 50mm", { 50.0,  0.0,   0.0 } },
        { "+Y 30mm", {  0.0, 30.0,   0.0 } },
    };
    size_t ci;

    printf("\n=== 3. 干净路径（起点 J5=20°，全程远离奇异）===\n");
    for (ci = 0; ci < sizeof(cases) / sizeof(cases[0]); ci++) {
        const double RAD2DEG = 180.0 / 3.14159265358979323846;
        double q_start[6] = { 0.0, 0.0, 90.0, 0.0, 20.0, 0.0 };
        double start6[6], end6[6], rpy[3], m[4][4];
        LinePath path;
        double q_seq[LINE_MAX_POINTS][6];
        JointLimit lim[6];
        int count, i, j, fail_idx = -1;
        char fail_reason[64] = {0};
        double worst_pos = 0.0, worst_jump = 0.0;
        int worst_pos_at = -1, worst_jump_at = -1, worst_jump_j = -1;
        char buf[160];

        make_limits(lim);
        dh_forward(DH_TABLE, q_start, m);
        dh_pose_to_xyz_rpy(m, start6, rpy);
        start6[3] = rpy[0] * RAD2DEG;
        start6[4] = rpy[1] * RAD2DEG;
        start6[5] = rpy[2] * RAD2DEG;
        for (j = 0; j < 6; j++) end6[j] = start6[j];
        end6[0] += cases[ci].d[0];
        end6[1] += cases[ci].d[1];
        end6[2] += cases[ci].d[2];

        count = line_count_for_distance(fabs(cases[ci].d[0]) +
                                        fabs(cases[ci].d[1]) +
                                        fabs(cases[ci].d[2]), 0.5);
        if (line_plan(start6, end6, count, &path) != 0 ||
            line_solve(&path, DH_TABLE, lim, q_start, q_seq, &fail_idx, fail_reason) != 0) {
            snprintf(buf, sizeof buf, "规划/逆解失败 @%d：%s", fail_idx, fail_reason);
            check(cases[ci].name, 0, buf);
            continue;
        }

        for (i = 0; i < path.count; i++) {
            double Tm[4][4], Tq[4][4], d;
            line_pose_to_matrix(path.pose[i], Tm);
            if (!fk_ok(q_seq[i], Tq)) { worst_pos = 1e30; worst_pos_at = i; break; }
            d = sqrt(pow(Tq[0][3] - Tm[0][3], 2) +
                     pow(Tq[1][3] - Tm[1][3], 2) +
                     pow(Tq[2][3] - Tm[2][3], 2));
            if (d > worst_pos) { worst_pos = d; worst_pos_at = i; }
        }
        for (i = 1; i < path.count; i++)
            for (j = 0; j < 6; j++) {
                double dj = fabs(q_seq[i][j] - q_seq[i - 1][j]);
                if (dj > worst_jump) { worst_jump = dj; worst_jump_at = i; worst_jump_j = j; }
            }

        printf("  %s：%d 个插补点\n", cases[ci].name, path.count);
        snprintf(buf, sizeof buf, "末端最大偏离 %.4f mm @点%d", worst_pos, worst_pos_at);
        check("  └ 末端贴着直线（<0.01mm）", worst_pos < 0.01, buf);
        snprintf(buf, sizeof buf, "相邻点最大跳变 %.4f°（J%d @点%d）",
                 worst_jump, worst_jump_j + 1, worst_jump_at);
        check("  └ 关节全程连续（<2°）", worst_jump < 2.0, buf);
    }
}

/* ====== 4. 从精确奇异点出发（home，J5=0）======
 * 不断言"首段不跳" —— 那是奇异位形的物理本质。只要求：
 *   ① 末端精确跟随（这是硬指标）
 *   ② 首段之后恢复连续（不许一路反复跳） */
static void test_from_singular(void)
{
    struct Case { const char *name; double d[3]; } cases[] = {
        { "-Z 10mm", {  0.0,  0.0, -10.0 } },
        { "+X 50mm", { 50.0,  0.0,   0.0 } },
        { "+Y 30mm", {  0.0, 30.0,   0.0 } },
    };
    size_t ci;

    printf("\n=== 4. 从精确奇异点出发（home，J5=0）—— 事故现场原样 ===\n");
    for (ci = 0; ci < sizeof(cases) / sizeof(cases[0]); ci++) {
        const double RAD2DEG = 180.0 / 3.14159265358979323846;
        double q_start[6] = { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 };
        double start6[6], end6[6], rpy[3], m[4][4];
        LinePath path;
        double q_seq[LINE_MAX_POINTS][6];
        JointLimit lim[6];
        int count, i, j, fail_idx = -1;
        char fail_reason[64] = {0};
        double worst_pos = 0.0, first_jump = 0.0, rest_jump = 0.0;
        int worst_pos_at = -1, rest_at = -1, rest_j = -1;
        char buf[160];

        make_limits(lim);
        dh_forward(DH_TABLE, q_start, m);
        dh_pose_to_xyz_rpy(m, start6, rpy);
        start6[3] = rpy[0] * RAD2DEG;
        start6[4] = rpy[1] * RAD2DEG;
        start6[5] = rpy[2] * RAD2DEG;
        for (j = 0; j < 6; j++) end6[j] = start6[j];
        end6[0] += cases[ci].d[0];
        end6[1] += cases[ci].d[1];
        end6[2] += cases[ci].d[2];

        count = line_count_for_distance(fabs(cases[ci].d[0]) +
                                        fabs(cases[ci].d[1]) +
                                        fabs(cases[ci].d[2]), 0.5);
        if (line_plan(start6, end6, count, &path) != 0 ||
            line_solve(&path, DH_TABLE, lim, q_start, q_seq, &fail_idx, fail_reason) != 0) {
            snprintf(buf, sizeof buf, "规划/逆解失败 @%d：%s", fail_idx, fail_reason);
            check(cases[ci].name, 0, buf);
            continue;
        }

        for (i = 0; i < path.count; i++) {
            double Tm[4][4], Tq[4][4], d;
            line_pose_to_matrix(path.pose[i], Tm);
            if (!fk_ok(q_seq[i], Tq)) { worst_pos = 1e30; worst_pos_at = i; break; }
            d = sqrt(pow(Tq[0][3] - Tm[0][3], 2) +
                     pow(Tq[1][3] - Tm[1][3], 2) +
                     pow(Tq[2][3] - Tm[2][3], 2));
            if (d > worst_pos) { worst_pos = d; worst_pos_at = i; }
        }
        for (j = 0; j < 6; j++) {
            double dj = fabs(q_seq[1][j] - q_seq[0][j]);
            if (dj > first_jump) first_jump = dj;
        }
        for (i = 2; i < path.count; i++)
            for (j = 0; j < 6; j++) {
                double dj = fabs(q_seq[i][j] - q_seq[i - 1][j]);
                if (dj > rest_jump) { rest_jump = dj; rest_at = i; rest_j = j; }
            }

        printf("  %s：%d 个插补点，首段最大跳变 %.3f°（奇异点出发的固有代价）\n",
               cases[ci].name, path.count, first_jump);
        snprintf(buf, sizeof buf, "末端最大偏离 %.4f mm @点%d（事故时 34.38mm）",
                 worst_pos, worst_pos_at);
        check("  └ 末端贴着直线（<0.05mm）", worst_pos < 0.05, buf);
        snprintf(buf, sizeof buf, "第2段起最大跳变 %.4f°（J%d @点%d）",
                 rest_jump, rest_j + 1, rest_at);
        check("  └ 首段之后恢复连续（<2°）", rest_jump < 2.0, buf);
    }
}

/* ====== 5. 穿越奇异点：θ5 从 +3° 连续走到 -3° ======
 * 这是最刁钻的一条：θ5 变号意味着必须切换 wrist 分支，而两个分支的
 * θ4 相差 180°。选解器（最小关节变化）必须自己切到连续的那支。
 * 出现 180° 级跳变 = 分支没切对 = 臂会甩。 */
static void test_cross_singular(void)
{
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double qA[6] = { 5.0, 2.0, 88.0, -85.0,  3.0, 85.0 };
    double qB[6] = { 5.0, 2.0, 88.0, -85.0, -3.0, 85.0 };
    double mA[4][4], mB[4][4], start6[6], end6[6], rpy[3];
    LinePath path;
    double q_seq[LINE_MAX_POINTS][6];
    JointLimit lim[6];
    int i, j, fail_idx = -1, count;
    char fail_reason[64] = {0};
    double worst_pos = 0.0, worst_jump = 0.0;
    int worst_jump_at = -1, worst_jump_j = -1;
    int crossed = 0;
    char buf[160];

    printf("\n=== 5. 穿越奇异点（θ5 从 +3° 走到 -3°）===\n");
    make_limits(lim);

    if (!fk_ok(qA, mA) || !fk_ok(qB, mB)) { check("FK 出 NaN", 0, NULL); return; }
    dh_pose_to_xyz_rpy(mA, start6, rpy);
    start6[3] = rpy[0] * RAD2DEG; start6[4] = rpy[1] * RAD2DEG; start6[5] = rpy[2] * RAD2DEG;
    dh_pose_to_xyz_rpy(mB, end6, rpy);
    end6[3] = rpy[0] * RAD2DEG;   end6[4] = rpy[1] * RAD2DEG;   end6[5] = rpy[2] * RAD2DEG;

    count = 60;
    if (line_plan(start6, end6, count, &path) != 0 ||
        line_solve(&path, DH_TABLE, lim, qA, q_seq, &fail_idx, fail_reason) != 0) {
        snprintf(buf, sizeof buf, "规划/逆解失败 @%d：%s", fail_idx, fail_reason);
        check("穿越路径可解", 0, buf);
        return;
    }

    for (i = 0; i < path.count; i++) {
        double Tm[4][4], Tq[4][4], d;
        line_pose_to_matrix(path.pose[i], Tm);
        if (!fk_ok(q_seq[i], Tq)) { worst_pos = 1e30; break; }
        d = sqrt(pow(Tq[0][3] - Tm[0][3], 2) +
                 pow(Tq[1][3] - Tm[1][3], 2) +
                 pow(Tq[2][3] - Tm[2][3], 2));
        if (d > worst_pos) worst_pos = d;
    }
    for (i = 1; i < path.count; i++) {
        if ((q_seq[i - 1][4] > 0.0) != (q_seq[i][4] > 0.0)) crossed = 1;
        for (j = 0; j < 6; j++) {
            double dj = fabs(q_seq[i][j] - q_seq[i - 1][j]);
            if (dj > worst_jump) { worst_jump = dj; worst_jump_at = i; worst_jump_j = j; }
        }
    }

    printf("  %d 个插补点，θ5 由 %+.3f° 走到 %+.3f°%s\n",
           path.count, q_seq[0][4], q_seq[path.count - 1][4],
           crossed ? "（已穿越 0）" : "（未穿越 0）");
    snprintf(buf, sizeof buf, "θ5 首 %.3f° → 末 %.3f°", q_seq[0][4], q_seq[path.count - 1][4]);
    check("路径确实穿越了 θ5=0", crossed, buf);
    snprintf(buf, sizeof buf, "末端最大偏离 %.4f mm", worst_pos);
    check("末端贴着路径（<0.05mm）", worst_pos < 0.05, buf);
    snprintf(buf, sizeof buf, "相邻点最大跳变 %.4f°（J%d @点%d）",
             worst_jump, worst_jump_j + 1, worst_jump_at);
    check("无 180° 级分支翻转（<30°）", worst_jump < 30.0, buf);
}

/* ====== 6. 对照：非奇异位形不得被误判 ======
 * 阈值若再放宽，会把正常位形也判成奇异、把 θ5 压成 0，末端姿态就错了。 */
static void test_no_false_positive(void)
{
    struct Case { const char *name; double q[6]; } cases[] = {
        { "J5=+20°", { 20.0, -30.0, 100.0, 40.0,  20.0, -15.0 } },
        { "J5=-20°", { 20.0, -30.0, 100.0, 40.0, -20.0, -15.0 } },
        { "J5=+70°", { 10.0, -20.0,  95.0, 10.0,  70.0,  30.0 } },
        { "J5=-85°", {  0.0, -10.0,  80.0, -30.0, -85.0,  60.0 } },
    };
    size_t k;

    printf("\n=== 6. 对照：非奇异位形不得被误判 ===\n");
    for (k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
        double T[4][4], sol[IK_MAX_SOLUTIONS][6];
        IkSolInfo info[IK_MAX_SOLUTIONS];
        int n, i, n_sing = 0, hit = 0;
        double best = 1e30;
        char buf[128];

        if (!fk_ok(cases[k].q, T)) { check(cases[k].name, 0, "FK 出 NaN"); continue; }
        n = ik_solve_ex(DH_TABLE, T, sol, info);
        for (i = 0; i < n; i++) {
            if (info[i].wrist == IK_SOL_SINGULAR) n_sing++;
            {
                double Ts[4][4], e;
                if (fk_ok(sol[i], Ts)) {
                    e = pose_max_err(Ts, T);
                    if (e < best) best = e;
                    if (e < 1e-6) hit = 1;
                }
            }
        }
        snprintf(buf, sizeof buf, "解 %d 组，奇异 %d 组，最小位姿误差 %.2e", n, n_sing, best);
        check(cases[k].name, n > 0 && n_sing == 0 && hit, buf);
    }
}

/* ====== 7. 规划层"甩臂"判据：line_max_joint_jump ======
 *
 * 【为什么要有这一节】2026-09-18 用户拍板：home 位形（J5=0，正踩腕部奇异）
 * 出发沿 +Y 走，第 1 段就要 J4 转 89.98° —— 这是奇异位形的物理本质，IK 层
 * 消不掉。原先只打印告警照常执行，现在改成超过阈值（默认 30°）直接拒绝。
 * 判据抽成 line_max_joint_jump() 就是为了让它能被离线测到 —— 埋在 CLI 里
 * 的 movl_plan 是没法单测的。
 *
 * 【测两件事】
 *   ① 病态路径必须**超阈值**（否则闸门形同虚设）
 *   ② 正常路径必须**远低于阈值**（否则闸门误伤正常作业）
 * 只测①是典型的"测试为结论服务"：一个永远返回 90° 的函数也能过①。 */
static void test_jump_gate(void)
{
    struct Case {
        const char *name;
        double q0[6];      /* 起点关节角 */
        double d[3];       /* 笛卡尔位移 mm */
        int  want_reject;  /* 1 = 期望被拒绝（>30°）；0 = 期望放行 */
    } cases[] = {
        /* 从精确奇异点出发沿 +Y：θ5 一离开 0，θ4 立刻被位姿锁死 ⇒ 首段 89.98° */
        { "病态：home(J5=0) +Y 30mm", { 0,0,90,0,0,0 }, {  0, 30,   0 }, 1 },
        { "病态：home(J5=0) +Y 50mm", { 0,0,90,0,0,0 }, {  0, 50,   0 }, 1 },
        /* 同一奇异起点但沿 -Z：不走奇异方向，正常 */
        { "正常：home(J5=0) -Z 50mm", { 0,0,90,0,0,0 }, {  0,  0, -50 }, 0 },
        { "正常：home(J5=0) +X 50mm", { 0,0,90,0,0,0 }, { 50,  0,   0 }, 0 },
        /* 起点 J5=20°（远离奇异），随便走都该正常 */
        { "正常：J5=20° -Z 50mm",     { 0,0,90,0,20,0 }, {  0,  0, -50 }, 0 },
        { "正常：J5=20° +Y 50mm",     { 0,0,90,0,20,0 }, {  0, 50,   0 }, 0 },
    };
    const double JUMP_LIMIT = 30.0;   /* 与 MOVL_JUMP_MAX_DEG / ini max_jump_deg 一致 */
    size_t ci;

    printf("\n=== 7. 规划层甩臂判据 line_max_joint_jump（拒绝阈值 %.0f°）===\n",
           JUMP_LIMIT);
    for (ci = 0; ci < sizeof(cases) / sizeof(cases[0]); ci++) {
        const double RAD2DEG = 180.0 / 3.14159265358979323846;
        double start6[6], end6[6], rpy[3], m[4][4];
        double q_seq[LINE_MAX_POINTS][6];
        LinePath path;
        JointLimit lim[6];
        int count, fail_idx = -1, wj = 0, ws = 0;
        char fail_reason[64] = {0};
        double jump;
        char buf[192];

        make_limits(lim);
        dh_forward(DH_TABLE, cases[ci].q0, m);
        dh_pose_to_xyz_rpy(m, start6, rpy);
        start6[3] = rpy[0] * RAD2DEG;
        start6[4] = rpy[1] * RAD2DEG;
        start6[5] = rpy[2] * RAD2DEG;
        end6[0] = start6[0] + cases[ci].d[0];
        end6[1] = start6[1] + cases[ci].d[1];
        end6[2] = start6[2] + cases[ci].d[2];
        end6[3] = start6[3];
        end6[4] = start6[4];
        end6[5] = start6[5];

        count = line_count_for_distance(fabs(cases[ci].d[0]) +
                                        fabs(cases[ci].d[1]) +
                                        fabs(cases[ci].d[2]), 1.0);  /* MOVL_STEP_MM */
        if (line_plan(start6, end6, count, &path) != 0 ||
            line_solve(&path, DH_TABLE, lim, cases[ci].q0, q_seq,
                       &fail_idx, fail_reason) != 0) {
            snprintf(buf, sizeof buf, "规划/逆解失败 @%d：%s", fail_idx, fail_reason);
            check(cases[ci].name, 0, buf);
            continue;
        }

        jump = line_max_joint_jump(q_seq, path.count, &wj, &ws);

        /* 顺带量一下"这一段实际要花多久" —— 时间表是按关节限速算的，
         * 段时长会被自动拉长。别想当然地写成"几毫秒跑完 90°"。 */
        {
            const uint16_t red[6] = { 50, 100, 50, 50, 50, 50 };  /* ROBOT_REDUCTION_TABLE */
            double vmax[6], seg_dt[LINE_MAX_SEGS], dt_total = 0.0;
            int k;
            for (k = 0; k < 6; k++) vmax[k] = 60.0 * 6.0 / (double)red[k];  /* 60 rpm 电机轴 */
            if (line_time_table(q_seq, path.count, vmax, seg_dt, &dt_total) == 0 && ws >= 1) {
                snprintf(buf, sizeof buf,
                         "单段最大跳变 %.2f°（J%d 第%d段/%d，该段 %.1f s）⇒ %s",
                         jump, wj, ws, path.count, seg_dt[ws - 1],
                         (jump > JUMP_LIMIT) ? "拒绝" : "放行");
            } else {
                snprintf(buf, sizeof buf, "单段最大跳变 %.2f°（J%d 第%d段/%d）⇒ %s",
                         jump, wj, ws, path.count,
                         (jump > JUMP_LIMIT) ? "拒绝" : "放行");
            }
        }
        check(cases[ci].name, (jump > JUMP_LIMIT) == cases[ci].want_reject, buf);
    }

    /* 纯函数的退化输入：不能崩，也不能返回"很大"把正常路径误判成甩臂 */
    {
        double q[2][6] = { {0,0,90,0,0,0}, {0,0,91,0,0,0} };
        double j1 = line_max_joint_jump(q, 2, NULL, NULL);
        double j0 = line_max_joint_jump(NULL, 2, NULL, NULL);
        double jn = line_max_joint_jump(q, 1, NULL, NULL);
        char buf[128];
        snprintf(buf, sizeof buf, "2点=%.2f° / NULL=%.2f / count<2=%.2f", j1, j0, jn);
        check("退化输入不崩且返回 0 或正常值",
              fabs(j1 - 1.0) < 1e-9 && j0 == 0.0 && jn == 0.0, buf);
    }
}

int main(void)
{
    dh_set_tool_length(0.0);

    test_threshold();
    test_converge();
    test_clean_path();
    test_from_singular();
    test_cross_singular();
    test_no_false_positive();
    test_jump_gate();

    printf("\n%s（失败项 %d）\n",
           g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
