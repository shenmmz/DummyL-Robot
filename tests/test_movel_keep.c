/*
 * test_movel_keep.c —— MoveL "省略姿态 / keep 关键字" 解析行为单测
 * ------------------------------------------------------------
 * 为什么锁死它：2026-09-19 真机事故 —— movel 的 Rx,Ry,Rz 抄错（抄成别的
 * 姿态下的值）是"画斜线"的头号根因。实测 80mm 线：姿态抄对笔尖偏离
 * 0.0000mm，抄成 115,90,115 ⇒ 偏离 7.71mm，而法兰直线度始终 0.0000mm。
 *
 * 于是加了两种"不用抄姿态"的写法，本测试锁死它们的行为，尤其锁死
 * 【向后兼容】：6 个数字段不带 keep 时必须仍解释为 X,Y,Z,Rx,Ry,Rz。
 * 这条一旦被改坏，用户所有老命令会静默变成"速度参数"。
 *
 * 用法：ctest -R movel_keep
 */
#include <stdio.h>
#include <string.h>

#include "utils/cmd_parser.h"

static int g_fail = 0;

static int check(const char *what, int cond)
{
    if (!cond) {
        printf("  [FAIL] %s\n", what);
        g_fail++;
    } else {
        printf("  [ok]   %s\n", what);
    }
    return cond;
}

static int parse_ok(const char *line, ParsedCmd *c)
{
    memset(c, 0, sizeof(*c));
    return cmd_parse(line, c);
}

int main(void)
{
    ParsedCmd c;

    printf("=== ① MoveL:X,Y,Z  只给位置 ⇒ 姿态保持，速度用默认 ===\n");
    if (check("解析成功", parse_ok("MoveL:143.06,132,155.16", &c) == CMD_MOVEL)) {
        check("keep_pose = 1", c.keep_pose == 1);
        check("X/Y/Z 正确", c.cartesian[0] == 143.06 && c.cartesian[1] == 132.0
                            && c.cartesian[2] == 155.16);
        check("速度默认 60 rpm", c.speeds[0] == 60.0);
        check("加减速默认 80/90", c.accel_ms[0] == 80 && c.decel_ms[0] == 90);
        check("模式默认 smooth（不分段，零段间停顿；2026-09-23 用户拍板改）",
              c.movl_mode == MOVL_MODE_SMOOTH);
    }

    printf("=== ② MoveL:X,Y,Z,SPD,ACC,DEC,keep ⇒ 姿态保持 + 自定义速度 ===\n");
    if (check("解析成功", parse_ok("MoveL:143.06,132,155.16,30,80,90,keep", &c) == CMD_MOVEL)) {
        check("keep_pose = 1", c.keep_pose == 1);
        check("速度 30 rpm", c.speeds[0] == 30.0);
        check("加减速 80/90", c.accel_ms[0] == 80 && c.decel_ms[0] == 90);
        check("X/Y/Z 正确（没被速度串位）", c.cartesian[0] == 143.06 && c.cartesian[2] == 155.16);
    }

    printf("=== ③ MoveL:X,Y,Z,Rx,Ry,Rz（旧写法）⇒ 姿态不保持，向后兼容 ===\n");
    if (check("解析成功", parse_ok("MoveL:143.06,132,155.16,-180,0,-180", &c) == CMD_MOVEL)) {
        check("keep_pose = 0", c.keep_pose == 0);
        check("姿态角为 -180,0,-180",
              c.cartesian[3] == -180.0 && c.cartesian[4] == 0.0 && c.cartesian[5] == -180.0);
        check("速度仍默认 60", c.speeds[0] == 60.0);
    }

    printf("=== ④ MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC（9 段完整写法）===\n");
    if (check("解析成功", parse_ok("MoveL:143.06,132,155.16,-180,0,-180,30,80,90", &c) == CMD_MOVEL)) {
        check("keep_pose = 0", c.keep_pose == 0);
        check("姿态角正确", c.cartesian[3] == -180.0 && c.cartesian[5] == -180.0);
        check("速度 30", c.speeds[0] == 30.0);
    }

    printf("=== ⑤ 模式关键字仍可用（与 keep 组合）===\n");
    if (check("MoveL:X,Y,Z,step ⇒ 姿态保持 + step",
              parse_ok("MoveL:143.06,132,155.16,step", &c) == CMD_MOVEL)) {
        check("keep_pose = 1", c.keep_pose == 1);
        check("mode = step", c.movl_mode == MOVL_MODE_STEP);
    }
    if (check("MoveL:X,Y,Z,keep,step ⇒ 两者都识别",
              parse_ok("MoveL:143.06,132,155.16,keep,step", &c) == CMD_MOVEL)) {
        check("keep_pose = 1", c.keep_pose == 1);
        check("mode = step", c.movl_mode == MOVL_MODE_STEP);
    }

    printf("=== ⑥ 向后兼容红线：6 段不带 keep 必须当姿态角，不能当速度 ===\n");
    if (check("MoveL:X,Y,Z,30,80,90 解析成功",
              parse_ok("MoveL:143.06,132,155.16,30,80,90", &c) == CMD_MOVEL)) {
        check("keep_pose = 0（不当作 keep）", c.keep_pose == 0);
        check("后三段进了 cartesian（姿态角）",
              c.cartesian[3] == 30.0 && c.cartesian[4] == 80.0 && c.cartesian[5] == 90.0);
        check("速度仍是默认 60（没被 30 覆盖）", c.speeds[0] == 60.0);
    }

    printf("=== ⑦ 非法输入仍被拒 ===\n");
    check("MoveL:143.06,132（只有 2 段）被拒",
          parse_ok("MoveL:143.06,132", &c) == CMD_UNKNOWN);
    check("MoveL:X,Y,Z,SPD,ACC,DEC,keep,bogus（未知模式）被拒",
          parse_ok("MoveL:143.06,132,155.16,30,80,90,keep,bogus", &c) == CMD_UNKNOWN);

    printf("----------------------------------------\n");
    if (g_fail == 0) {
        printf("PASS: MoveL keep 语法全部符合预期\n");
        return 0;
    }
    printf("FAIL: %d 项不符合预期\n", g_fail);
    return 1;
}
