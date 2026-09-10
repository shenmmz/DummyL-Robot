/*
 * test_cmd_parser.c —— CLI 命令解析离线单元测试
 * ------------------------------------------------------------
 * 所属：tests（CTest）
 * 覆盖：新增 fk / ik 命令的解析与参数个数校验，以及既有命令（movej/status/exit/空行）回归。
 * 说明：参数非法时 cmd_parse 会打印中文警告，属预期输出；本测试只校验返回值与字段。
 */

#include "utils/cmd_parser.h"

#include <math.h>
#include <stdio.h>

static int g_fail = 0;

#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            printf("  [FAIL] %s:%d ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                           \
            printf("\n");                                  \
            g_fail++;                                      \
        }                                                  \
    } while (0)

int main(void)
{
    ParsedCmd c;
    int rc;

    printf("test_cmd_parser: 命令解析\n");

    /* fk：无参数 = 打印预设验证姿态表 */
    rc = cmd_parse("fk", &c);
    CHECK(rc == CMD_FK, "fk 应解析为 CMD_FK（得到 %d）", rc);
    CHECK(c.val_count == 0, "fk 无参数时 val_count 应为 0（得到 %d）", c.val_count);

    /* fk：6 个关节角 */
    rc = cmd_parse("fk:10:20:30:40:50:60", &c);
    CHECK(rc == CMD_FK && c.val_count == 6,
          "fk 六参数应解析成功（rc=%d, n=%d）", rc, c.val_count);
    CHECK(fabs(c.vals[0] - 10.0) < 1e-12 && fabs(c.vals[3] - 40.0) < 1e-12 &&
              fabs(c.vals[5] - 60.0) < 1e-12,
          "fk 参数值解析错误：%.3f %.3f %.3f", c.vals[0], c.vals[3], c.vals[5]);

    /* fk：负数与小数 */
    rc = cmd_parse("fk:-45.5:0:0:0:0:0", &c);
    CHECK(rc == CMD_FK && fabs(c.vals[0] + 45.5) < 1e-12,
          "fk 负数/小数解析失败（rc=%d, v0=%.3f）", rc, c.vals[0]);

    /* fk：参数个数非法 */
    CHECK(cmd_parse("fk:1:2:3", &c) == CMD_UNKNOWN, "fk 仅三个参数应被拒绝");
    CHECK(cmd_parse("fk:1:2:3:4:5:6:7", &c) == CMD_UNKNOWN, "fk 七个参数应被拒绝");

    /* ik：目标位姿 6 参数（位置 mm + RPY 度） */
    rc = cmd_parse("ik:100:-50:200:180:0:0", &c);
    CHECK(rc == CMD_IK && c.val_count == 6,
          "ik 六参数应解析成功（rc=%d, n=%d）", rc, c.val_count);
    CHECK(fabs(c.vals[0] - 100.0) < 1e-12 && fabs(c.vals[1] + 50.0) < 1e-12 &&
              fabs(c.vals[3] - 180.0) < 1e-12,
          "ik 参数值解析错误：%.3f %.3f %.3f", c.vals[0], c.vals[1], c.vals[3]);

    CHECK(cmd_parse("ik:1:2:3", &c) == CMD_UNKNOWN, "ik 参数不足应被拒绝");
    CHECK(cmd_parse("ik:1:2:3:4:5:6:7", &c) == CMD_UNKNOWN, "ik 参数过多应被拒绝");

    /* 既有命令回归 */
    rc = cmd_parse("movej:3:45:200", &c);
    CHECK(rc == CMD_MOVEJ && c.joint == 3 && fabs(c.angle_deg - 45.0) < 1e-12,
          "movej 解析回归失败（rc=%d, joint=%d, angle=%.3f）", rc, c.joint, c.angle_deg);
    CHECK(cmd_parse("status", &c) == CMD_STATUS, "status 解析回归失败");
    CHECK(cmd_parse("exit", &c) == CMD_EXIT, "exit 解析回归失败");
    CHECK(cmd_parse("", &c) == CMD_EMPTY, "空行应解析为 CMD_EMPTY");
    CHECK(cmd_parse("nosuchcmd", &c) == CMD_UNKNOWN, "未知命令应解析为 CMD_UNKNOWN");

    if (g_fail == 0) {
        printf("test_cmd_parser: PASS\n");
        return 0;
    }
    printf("test_cmd_parser: FAIL（%d 处断言未通过）\n", g_fail);
    return 1;
}
