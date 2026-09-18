/*
 * test_stall.c —— 堵转/碰撞保护的离线单测
 * ------------------------------------------------------------
 * 覆盖两块【安全逻辑】，都不碰串口、不需要机械臂在场：
 *   1) monitor_stall_hit：电流是否算超阈（纯函数，抽出来自 monitor.c）
 *   2) ini_read_stall_current：[stall] 段逐轴阈值的解析
 *
 * 为什么必须单测：堵转判定是唯一的安全兜底，误报会打断正常运动、
 * 漏报就是撞了不停。这类逻辑不能只在真机上"试试看"。
 *
 * 最核心的一条是【逐轴独立性】：同一个电流值，在大轴阈值下算正常、
 * 在小轴阈值下算堵转。旧设计是全局单值，做不到这件事。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/monitor.h"
#include "utils/ini_rw.h"
#include "config/robot_config.h"

static int g_fail = 0;
static int g_check = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_check++;                                                          \
        if (!(cond)) {                                                      \
            g_fail++;                                                       \
            printf("  [FAIL] ");                                            \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                   \
        }                                                                   \
    } while (0)

/* 临时 ini 路径：ctest 的工作目录是 build/，写这里不影响源码树 */
#define TMP_INI "test_stall_tmp.ini"

static void write_tmp_ini(const char *text)
{
    FILE *f = fopen(TMP_INI, "wb");
    if (f == NULL) {
        printf("  [FAIL] 无法写临时文件 %s\n", TMP_INI);
        g_fail++;
        return;
    }
    fputs(text, f);
    fclose(f);
}

/* ---------- 1) monitor_stall_hit 判定边界 ---------- */
static void test_stall_hit(void)
{
    printf("[1] monitor_stall_hit 判定边界\n");

    /* 阈值 <=0 = 该轴未配置 = 不检测（这是默认状态，绝不能报警） */
    CHECK(monitor_stall_hit(0, 0) == 0, "阈值0/电流0 应不报警");
    CHECK(monitor_stall_hit(5000, 0) == 0, "阈值0（未配置）时电流再大也不报警");
    CHECK(monitor_stall_hit(5000, -1) == 0, "负阈值应视为未配置");

    /* 读数失败(<0) 不得误报：漏报可以接受，误报会打断正常运动 */
    CHECK(monitor_stall_hit(-1, 500) == 0, "读电流失败(-1) 应不报警");
    CHECK(monitor_stall_hit(-9999, 500) == 0, "读电流失败(任意负值) 应不报警");

    /* 严格大于才算超阈：等于不算 */
    CHECK(monitor_stall_hit(499, 500) == 0, "499 < 500 应不报警");
    CHECK(monitor_stall_hit(500, 500) == 0, "等于阈值(500==500) 应不报警");
    CHECK(monitor_stall_hit(501, 500) == 1, "501 > 500 应报警");

    /* 边界外的极端值 */
    CHECK(monitor_stall_hit(1, 1) == 0, "1==1 边界");
    CHECK(monitor_stall_hit(2, 1) == 1, "2>1 边界");
}

/* ---------- 2) 逐轴独立性：全局单值做不到的事 ---------- */
static void test_per_axis(void)
{
    /* 模拟真实配置：大轴(底座/大臂)阈值高，小轴(腕部)阈值低 */
    const int th[6]    = {1500, 1500, 1200, 600, 600, 400};
    const int cur[6]   = { 900,  900,  900, 900, 900, 900};
    const int want[6]  = {   0,    0,    0,   1,   1,   1};
    int j;

    printf("[2] 逐轴独立性（同一电流 900mA，大轴正常 / 小轴报警）\n");
    for (j = 0; j < 6; j++) {
        CHECK(monitor_stall_hit(cur[j], th[j]) == want[j],
              "关节%d 电流%d / 阈值%d 期望%d 实得%d",
              j + 1, cur[j], th[j], want[j], monitor_stall_hit(cur[j], th[j]));
    }

    /* 反证：若退化成"全局单值"，这 6 个结果必然全同。
     * 这里断言"结果不全同"，一旦有人改回单值就会立刻失败。 */
    {
        int all_same = 1;
        for (j = 1; j < 6; j++) {
            if (monitor_stall_hit(cur[j], th[j]) != monitor_stall_hit(cur[0], th[0])) {
                all_same = 0;
                break;
            }
        }
        CHECK(all_same == 0, "六轴判定结果全同 ⇒ 阈值退化成了全局单值");
    }
}

/* ---------- 3) ini [stall] 解析 ---------- */
static void test_ini(void)
{
    int th[6];
    int i;

    printf("[3] ini [stall] 逐轴阈值解析\n");

    /* 正常：六轴齐全 */
    write_tmp_ini("[serial]\nport = COM4\n[stall]\n"
                  "j1 = 1500\nj2 = 1500\nj3 = 1200\nj4 = 600\nj5 = 600\nj6 = 400\n"
                  "[joint_zero]\nq0 = 1\n");
    memset(th, 0xAA, sizeof(th));
    CHECK(ini_read_stall_current(TMP_INI, th) == 1, "六轴齐全应返回 1");
    {
        const int want[6] = {1500, 1500, 1200, 600, 600, 400};
        for (i = 0; i < 6; i++) {
            CHECK(th[i] == want[i], "j%d 期望 %d 实得 %d", i + 1, want[i], th[i]);
        }
    }

    /* 缺一项 ⇒ 整体返回 0（不能拿半套阈值去跑保护） */
    write_tmp_ini("[stall]\nj1 = 1500\nj2 = 1500\nj3 = 1200\nj4 = 600\nj5 = 600\n");
    memset(th, 0, sizeof(th));
    CHECK(ini_read_stall_current(TMP_INI, th) == 0, "缺 j6 应返回 0");

    /* 无 [stall] 段 ⇒ 返回 0，调用方回退编译期默认 */
    write_tmp_ini("[serial]\nport = COM4\n[tool]\ntool_length = 0\n");
    memset(th, 0, sizeof(th));
    CHECK(ini_read_stall_current(TMP_INI, th) == 0, "无 [stall] 段应返回 0");

    /* 段内注释不得被当成配置读走 */
    write_tmp_ini("[stall]\n# j1 = 9999 （这是注释）\n; j2 = 8888\n"
                  "j1 = 111\nj2 = 222\nj3 = 333\nj4 = 444\nj5 = 555\nj6 = 666\n");
    memset(th, 0, sizeof(th));
    CHECK(ini_read_stall_current(TMP_INI, th) == 1, "带注释行仍应解析成功");
    CHECK(th[0] == 111, "注释里的 j1 = 9999 不应生效，实得 %d", th[0]);
    CHECK(th[1] == 222, "注释里的 j2 = 8888 不应生效，实得 %d", th[1]);

    /* 其它段的同名键不得串段（[other] 里的 j1 不该被读成 [stall] 的 j1） */
    write_tmp_ini("[other]\nj1 = 999\n[stall]\n"
                  "j1 = 111\nj2 = 222\nj3 = 333\nj4 = 444\nj5 = 555\nj6 = 666\n");
    memset(th, 0, sizeof(th));
    CHECK(ini_read_stall_current(TMP_INI, th) == 1, "跨段同名键场景应解析成功");
    CHECK(th[0] == 111, "[other] 段的 j1=999 不应污染 [stall]，实得 %d", th[0]);

    /* 文件不存在 ⇒ 返回 0，不崩 */
    memset(th, 0, sizeof(th));
    CHECK(ini_read_stall_current("no_such_file_xyz.ini", th) == 0, "文件不存在应返回 0");

    remove(TMP_INI);
}

/* ---------- 4) 编译期默认表 ---------- */
static void test_default_table(void)
{
    const int def[ROBOT_JOINT_COUNT] = ROBOT_STALL_CURRENT_MA_TABLE;
    int i, on = 0;

    printf("[4] 编译期默认表 ROBOT_STALL_CURRENT_MA_TABLE\n");
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        if (def[i] > 0) on++;
    }
    printf("    j1..j6 = %d,%d,%d,%d,%d,%d mA ⇒ %d 轴默认启用\n",
           def[0], def[1], def[2], def[3], def[4], def[5], on);
    /* 断言的不是"必须为 0"，而是"要么全关、要么全开"。
     * 半套阈值（部分轴 0 部分轴有值）最危险：用户以为开了保护，
     * 实际上没配的那几轴撞上去照样不停。 */
    CHECK(on == 0 || on == ROBOT_JOINT_COUNT,
          "默认表不应出现半套阈值（当前 %d/6 轴启用）", on);
}

int main(void)
{
    printf("=== test_stall：堵转保护离线单测 ===\n");
    test_stall_hit();
    test_per_axis();
    test_ini();
    test_default_table();

    printf("\n断言 %d 条，失败 %d 条 ⇒ %s\n",
           g_check, g_fail, (g_fail == 0) ? "全部通过" : "存在失败");
    return (g_fail == 0) ? 0 : 1;
}
