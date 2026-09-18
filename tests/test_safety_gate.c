/*
 * test_safety_gate —— 下发前的目标合理性闸门（离线）
 * ------------------------------------------------------------
 * 【为什么要这套闸门】本机出过两次"把垃圾目标写给驱动器"的事故，
 * 两次的目标值**都是完全合法的 int32**，所以"查 NaN"根本挡不住：
 *
 *   ① 2026-09-18：home 姿态 Ry=90° 正踩万向锁，quat_to_euler 里
 *      asin 自变量算出 1.0000000000000002 越界 ⇒ NaN ⇒ 转 int32 成
 *      **±2147483647** ⇒ 电机朝天文数字猛冲、永不到位、超时急停，
 *      末端甩出 34mm。（真因与修复见 tests/test_line_nan.c）
 *   ② 同一天另一次：脚本写 0x00E8 时高低字写反，目标变成 **3.28 亿**
 *      （≈118000°）⇒ J5 猛冲顶死、相位过流、把电源拉垮、六轴零点全丢。
 *
 * 共同点：值合法、量级离谱。只有"物理上不可能这么大/这么远"能识别。
 *
 * 【四道防线，本测试覆盖其中可离线测的部分】
 *   A. api/motor_reg.c  motor_move_steps_ok()：|steps| > 1e8 拒绝
 *      —— 出口闸门，所有写 0x00E8 的路径都绕不过  ★ 本测试
 *   B. cli/commands.c   movej_issue()：目标角 NaN/Inf 拒绝  （需真机）
 *   C. control/robot.c  robot_movej()：目标角 NaN/Inf 拒绝  （需真机）
 *   D. cli/commands.c   movej_issue()：单次位移 > [safety] max_step_deg 拒绝
 *      —— 阈值由 ini 读，解析部分  ★ 本测试；判定部分（需真机）
 *
 * 运行：cmake --build build --target test_safety_gate && build/bin/test_safety_gate.exe
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "api/motor_reg.h"
#include "utils/ini_rw.h"
#include "config/robot_config.h"

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-48s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

/* 相对路径的两种跑法都要能工作：
 *   从项目根跑：  build/bin/test_safety_gate.exe   （CWD = 项目根）
 *   从 build/ 跑：ctest / ./bin/test_safety_gate.exe（CWD = build）
 * 【2026-09-18 踩坑】第一版把路径写死成 "build/_tmp.ini" 和
 * "src/config/..."，从项目根跑全绿、ctest 一跑就红 —— 根目录对不上。
 * 测试自己不能依赖 CWD，否则"通过"只是"碰巧在哪个目录下跑"的假象。 */
static const char *k_prefix[] = { "", "../", "../../" };

/* 依次试前缀，把第一个能打开的相对路径写进 out；for_write=1 时以写方式试 */
static int resolve_path(const char *rel, char *out, size_t n, int for_write)
{
    size_t i;
    for (i = 0; i < sizeof k_prefix / sizeof k_prefix[0]; i++) {
        FILE *f;
        snprintf(out, n, "%s%s", k_prefix[i], rel);
        f = fopen(out, for_write ? "w" : "r");
        if (f != NULL) { fclose(f); return 1; }
    }
    return 0;
}

/* 写一个临时 ini 文件供解析测试用 */
static int write_tmp_ini(const char *path, const char *body)
{
    FILE *f = fopen(path, "w");
    if (f == NULL) return 0;
    fputs(body, f);
    fclose(f);
    return 1;
}

int main(void)
{
    printf("=== 1. 出口闸门：真事故里那些数必须被拦下 ===\n");
    {
        struct { const char *nm; int32_t v; int want; } cases[] = {
            { "① NaN 转 int32（INT32_MAX）",           INT32_MAX,     0 },
            { "① NaN 转 int32（INT32_MIN）",           INT32_MIN,     0 },
            { "① 日志里的「还差 2147228222 步」",      2147228222,    0 },
            { "① 日志里的「还差 -2147354590 步」",    -2147354590,    0 },
            { "② 字序写反的 3.28 亿步",                328000000,     0 },
            { "② 字序写反的 78852992 步",              78852992,      1 },  /* 见下方说明 */
            { "正常：J1 走 90°（12.5 万步）",           125000,        1 },
            { "正常：J2 走 90°（25 万步）",             250000,        1 },
            { "正常：连续同向转 10 圈（J2）",           10000000,      1 },
            { "临界：恰在 +1e8",                        100000000,     1 },
            { "临界：+1e8 再加 1",                      100000001,     0 },
            { "临界：恰在 -1e8",                       -100000000,     1 },
            { "临界：-1e8 再减 1",                     -100000001,     0 },
            { "零",                                     0,             1 },
        };
        size_t k;
        for (k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
            char buf[96];
            int got = motor_move_steps_ok(cases[k].v);
            snprintf(buf, sizeof buf, "%d 步 → %s（期望 %s）",
                     (int)cases[k].v, got ? "放行" : "拒绝",
                     cases[k].want ? "放行" : "拒绝");
            check(cases[k].nm, got == cases[k].want, buf);
        }
        printf("  注：78852992 步（≈J5 转 56°）量级本身正常 —— 那次事故里它是\n"
               "      【速度】被写坏，不是位置。量级闸门只管位置，速度另有范围检查。\n"
               "      说明「量级正常」不等于「一定安全」，闸门只是最后一道网，不是万能药。\n");
    }

    printf("\n=== 2. 阈值量纲自检：1e8 步到底相当于多少圈 ===\n");
    {
        /* 闸门值必须"远大于任何合法行程"，否则会误伤正常作业。
         * 编码器 10000 步/圈，减速比见 ROBOT_REDUCTION_TABLE。 */
        const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
        const double lmin[6] = ROBOT_JOINT_LIMIT_MIN_DEG;
        const double lmax[6] = ROBOT_JOINT_LIMIT_MAX_DEG;
        int j, all_ok = 1;
        char buf[128];

        printf("  %-6s %-8s %-14s %-14s %s\n", "关节", "减速比", "限位全宽(°)", "闸门=多少圈", "余量");
        for (j = 0; j < 6; j++) {
            double steps_per_rev = 10000.0 * (double)red[j];
            double revs = (double)ROBOT_ABS_MOVE_STEPS_LIMIT / steps_per_rev;
            double span = lmax[j] - lmin[j];
            double margin = revs / (span / 360.0);
            printf("  J%-5d %-8u %-14.0f %-14.1f %.0f×\n",
                   j + 1, red[j], span, revs, margin);
            if (margin < 10.0) all_ok = 0;
        }
        snprintf(buf, sizeof buf, "最少也留了 10 倍余量（相对限位全宽）");
        check("闸门余量足够（≥10× 限位全宽）", all_ok, buf);
    }

    printf("\n=== 3. ini [safety] max_step_deg 解析 ===\n");
    {
        char p[256];
        double v;
        char buf[128];

        if (!resolve_path("build/_tmp_safety.ini", p, sizeof p, 1)) {
            check("能定位临时文件目录", 0, "三种前缀都写不了，放弃");
            goto skip_ini;
        }

        /* 正常 */
        if (write_tmp_ini(p, "[safety]\n# 注释行\nmax_step_deg = 45\n")) {
            int r = ini_read_max_step_deg(p, &v);
            snprintf(buf, sizeof buf, "读到 %.1f（期望 45）", v);
            check("正常解析", r == 1 && fabs(v - 45.0) < 1e-9, buf);
        } else check("正常解析", 0, "无法写临时文件");

        /* 段内注释不能误当成配置 */
        if (write_tmp_ini(p, "[safety]\n# max_step_deg = 999\nmax_step_deg = 30\n")) {
            int r = ini_read_max_step_deg(p, &v);
            snprintf(buf, sizeof buf, "读到 %.1f（期望 30，不该是注释里的 999）", v);
            check("跳过段内注释行", r == 1 && fabs(v - 30.0) < 1e-9, buf);
        }

        /* 别的段里同名 key 不能串味 */
        if (write_tmp_ini(p, "[movel]\nmax_step_deg = 999\n[safety]\nmax_step_deg = 60\n")) {
            int r = ini_read_max_step_deg(p, &v);
            snprintf(buf, sizeof buf, "读到 %.1f（期望 60，不该拿 [movel] 的 999）", v);
            check("只认 [safety] 段", r == 1 && fabs(v - 60.0) < 1e-9, buf);
        }

        /* 只有别的段 ⇒ 找不到，回退默认 */
        if (write_tmp_ini(p, "[movel]\nbow_mm = 0.20\n")) {
            int r = ini_read_max_step_deg(p, &v);
            check("缺 [safety] 段 ⇒ 返回 0（调用方回退默认）", r == 0, NULL);
        }

        /* 配置成 0 / 负数 = 无效，必须回退默认而不是"关闭闸门" */
        if (write_tmp_ini(p, "[safety]\nmax_step_deg = 0\n")) {
            int r = ini_read_max_step_deg(p, &v);
            check("配成 0 ⇒ 无效（不能解释为「不限制」）", r == 0, NULL);
        }
        if (write_tmp_ini(p, "[safety]\nmax_step_deg = -5\n")) {
            int r = ini_read_max_step_deg(p, &v);
            check("配成负数 ⇒ 无效", r == 0, NULL);
        }

        /* 文件不存在 */
        {
            int r = ini_read_max_step_deg("build/_no_such_file.ini", &v);
            check("文件不存在 ⇒ 返回 0", r == 0, NULL);
        }
        remove(p);
    }
skip_ini:

    printf("\n=== 4. 真实 ini 必须能读到（防止配了没生效）===\n");
    {
        char ini[256];
        double v = -1.0;
        int r = 0;
        char buf[160];

        if (resolve_path("src/config/robot_config.ini", ini, sizeof ini, 0))
            r = ini_read_max_step_deg(ini, &v);
        else
            check("能定位项目 ini", 0, "三种前缀都找不到 src/config/robot_config.ini");

        snprintf(buf, sizeof buf, "读到 %.1f°（路径 %s）", v,
                 r ? ini : "未找到");
        check("项目 ini 的 [safety] max_step_deg", r == 1 && v > 0.0, buf);
    }

    printf("\n%s（失败项 %d）\n",
           g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
