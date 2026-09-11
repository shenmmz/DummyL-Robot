/*
 * test_cmd_parser.c —— CLI 命令解析离线单元测试
 * ------------------------------------------------------------
 * 所属：tests（CTest）
 * 覆盖：home、movej、disable、motor、getpos、zero、exit、空行、未知命令的解析与参数校验。
 */

#include "utils/cmd_parser.h"

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

    /* home：全轴回零 */
    rc = cmd_parse("home", &c);
    CHECK(rc == CMD_HOME && c.joint == 0, "home 应解析为 CMD_HOME joint=0（得到 rc=%d, joint=%d）", rc, c.joint);

    /* home：单轴回零 */
    rc = cmd_parse("home:3", &c);
    CHECK(rc == CMD_HOME && c.joint == 3, "home:3 应解析为 CMD_HOME joint=3（得到 rc=%d, joint=%d）", rc, c.joint);

    /* home：关节号非法 */
    CHECK(cmd_parse("home:7", &c) == CMD_UNKNOWN, "home:7 关节号非法应被拒绝");

    /* movej：关节+角度+速度 */
    rc = cmd_parse("movej:1:45:200", &c);
    CHECK(rc == CMD_MOVEJ && c.joint == 1 && c.angle_deg == 45.0 && c.speed_rpm == 200.0,
          "movej:1:45:200 解析失败（rc=%d, joint=%d, angle=%.3f, speed=%.3f）", rc, c.joint, c.angle_deg, c.speed_rpm);

    /* movej：关节+角度（默认速度） */
    rc = cmd_parse("movej:2:90", &c);
    CHECK(rc == CMD_MOVEJ && c.joint == 2 && c.angle_deg == 90.0 && c.speed_rpm == 0.0,
          "movej:2:90 解析失败（rc=%d, angle=%.3f, speed=%.3f）", rc, c.angle_deg, c.speed_rpm);

    /* movej：关节号非法 */
    CHECK(cmd_parse("movej:7:45", &c) == CMD_UNKNOWN, "movej:7:45 关节号非法应被拒绝");

    /* disable */
    rc = cmd_parse("disable", &c);
    CHECK(rc == CMD_DISABLE && c.joint == 0, "disable 应解析为 CMD_DISABLE joint=0（得到 rc=%d, joint=%d）", rc, c.joint);

    /* disable:N 单轴泄力 */
    rc = cmd_parse("disable:3", &c);
    CHECK(rc == CMD_DISABLE && c.joint == 3, "disable:3 应解析为 CMD_DISABLE joint=3（得到 rc=%d, joint=%d）", rc, c.joint);

    /* disable: 关节号非法 */
    CHECK(cmd_parse("disable:7", &c) == CMD_UNKNOWN, "disable:7 关节号非法应被拒绝");

    /* motor：启动/停止全关节监控（不再支持 motor:N 子指令） */
    rc = cmd_parse("motor", &c);
    CHECK(rc == CMD_MOTOR && c.joint == 0, "motor 应解析为 CMD_MOTOR joint=0（得到 rc=%d, joint=%d）", rc, c.joint);

    /* motor:N 后缀不再解析：仍按全关节监控处理（joint 保持 0） */
    rc = cmd_parse("motor:2", &c);
    CHECK(rc == CMD_MOTOR && c.joint == 0, "motor:2 应解析为 CMD_MOTOR，N 后缀忽略（得到 rc=%d, joint=%d）", rc, c.joint);

    /* exit */
    rc = cmd_parse("exit", &c);
    CHECK(rc == CMD_EXIT, "exit 解析失败（得到 rc=%d）", rc);

    /* quit */
    rc = cmd_parse("quit", &c);
    CHECK(rc == CMD_EXIT, "quit 解析失败（得到 rc=%d）", rc);

    /* help */
    rc = cmd_parse("help", &c);
    CHECK(rc == CMD_HELP, "help 解析失败（得到 rc=%d）", rc);

    /* getpos */
    rc = cmd_parse("getpos", &c);
    CHECK(rc == CMD_GETPOS, "getpos 解析失败（得到 rc=%d）", rc);

    /* zero / zero save / zerosave */
    rc = cmd_parse("zero", &c);
    CHECK(rc == CMD_ZERO && c.joint == 0, "zero 应解析为 CMD_ZERO 非保存（得到 rc=%d, joint=%d）", rc, c.joint);
    rc = cmd_parse("zero save", &c);
    CHECK(rc == CMD_ZERO && c.joint == 1, "zero save 应解析为 CMD_ZERO 保存（得到 rc=%d, joint=%d）", rc, c.joint);
    rc = cmd_parse("zerosave", &c);
    CHECK(rc == CMD_ZERO && c.joint == 1, "zerosave 应解析为 CMD_ZERO 保存（得到 rc=%d, joint=%d）", rc, c.joint);

    /* 空行 */
    rc = cmd_parse("", &c);
    CHECK(rc == CMD_EMPTY, "空行应解析为 CMD_EMPTY");

    /* 未知命令 */
    rc = cmd_parse("nosuchcmd", &c);
    CHECK(rc == CMD_UNKNOWN, "未知命令应解析为 CMD_UNKNOWN");

    if (g_fail == 0) {
        printf("test_cmd_parser: PASS\n");
        return 0;
    }
    printf("test_cmd_parser: FAIL（%d 处断言未通过）\n", g_fail);
    return 1;
}
