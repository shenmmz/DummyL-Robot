/*
 * test_cmd_parser.c —— CLI 命令解析离线单元测试
 * ------------------------------------------------------------
 * 所属：tests（CTest）
 * 覆盖：home、movej（单/多关节及参数校验）、movel、disable、enable、motor、
 *       getpos、zero、zero_save、exit、空行、未知命令的解析与参数校验。
 */

#include "utils/cmd_parser.h"

#include <stdio.h>
#include <math.h>

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

    /* movej：关节+角度（默认速度，绝对） */
    rc = cmd_parse("movej:2:90", &c);
    CHECK(rc == CMD_MOVEJ && c.joint == 2 && c.angle_deg == 90.0 && c.speed_rpm == 0.0 && c.rel == 0,
          "movej:2:90 解析失败（rc=%d, angle=%.3f, speed=%.3f, rel=%d）", rc, c.angle_deg, c.speed_rpm, c.rel);

    /* movej：相对模式（默认速度） */
    rc = cmd_parse("movej:2:15:r", &c);
    CHECK(rc == CMD_MOVEJ && c.joint == 2 && c.angle_deg == 15.0 && c.speed_rpm == 0.0 && c.rel == 1,
          "movej:2:15:r 相对模式解析失败（rc=%d, angle=%.3f, rel=%d）", rc, c.angle_deg, c.rel);

    /* movej：相对模式 + 速度 */
    rc = cmd_parse("movej:2:15:200:r", &c);
    CHECK(rc == CMD_MOVEJ && c.joint == 2 && c.angle_deg == 15.0 && c.speed_rpm == 200.0 && c.rel == 1,
          "movej:2:15:200:r 相对+速度解析失败（rc=%d, angle=%.3f, spd=%.3f, rel=%d）", rc, c.angle_deg, c.speed_rpm, c.rel);

    /* movej：显式绝对模式 */
    rc = cmd_parse("movej:2:90:a", &c);
    CHECK(rc == CMD_MOVEJ && c.rel == 0 && c.speed_rpm == 0.0,
          "movej:2:90:a 显式绝对模式解析失败（rc=%d, rel=%d）", rc, c.rel);

    /* movej：模式在前、速度在后（顺序无关） */
    rc = cmd_parse("movej:2:15:r:200", &c);
    CHECK(rc == CMD_MOVEJ && c.rel == 1 && c.speed_rpm == 200.0,
          "movej:2:15:r:200 模式优先速度解析失败（rc=%d, rel=%d, spd=%.3f）", rc, c.rel, c.speed_rpm);

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

    /* zero / zero_save */
    rc = cmd_parse("zero", &c);
    CHECK(rc == CMD_ZERO && c.joint == 0, "zero 应解析为 CMD_ZERO 非保存（得到 rc=%d, joint=%d）", rc, c.joint);
    rc = cmd_parse("zero_save:-176.88,72.89,-175.48,7.27,118.11,444.58", &c);
    CHECK(rc == CMD_ZERO_SAVE, "zero_save 应解析为 CMD_ZERO_SAVE（得到 rc=%d）", rc);
    CHECK(fabs(c.zero_vals[0] + 176.88) < 1e-6 && fabs(c.zero_vals[5] - 444.58) < 1e-6,
          "zero_save 数值解析失败（q0=%.4f, q5=%.4f）", c.zero_vals[0], c.zero_vals[5]);

    /* multi-joint movej：movej:ANG1..ANG6,SPD,ACC,DEC（9 段，J1~J6 全轴） */
    rc = cmd_parse("movej:10,20,30,40,50,60,3000,150,200", &c);
    CHECK(rc == CMD_MOVEJ && c.num_joints == 6, "多关节 movej 应解析为 CMD_MOVEJ num_joints=6（得到 rc=%d, n=%d）", rc, c.num_joints);
    CHECK(c.joints[0] == 1 && c.angles[0] == 10.0 && c.joints[5] == 6 && c.angles[5] == 60.0,
          "多关节 movej 参数解析失败（j0=%d,a0=%.2f,j5=%d,a5=%.2f）",
          c.joints[0], c.angles[0], c.joints[5], c.angles[5]);
    CHECK(c.speeds[0] == 3000.0 && c.accel_ms[0] == 150 && c.decel_ms[0] == 200,
          "多关节 movej 加减速解析失败（spd=%.0f,acc=%d,dec=%d）",
          c.speeds[0], c.accel_ms[0], c.decel_ms[0]);

    /* movel：笛卡尔坐标（仅位姿，默认速度） */
    rc = cmd_parse("movel:100,200,300,0,0,0", &c);
    CHECK(rc == CMD_MOSEL, "movel 应解析为 CMD_MOSEL（得到 rc=%d）", rc);
    CHECK(fabs(c.cartesian[0] - 100) < 1e-6 && fabs(c.cartesian[5] - 0) < 1e-6,
          "movel 数值解析失败（X=%.4f, Rz=%.4f）", c.cartesian[0], c.cartesian[5]);
    CHECK(c.speeds[0] == 3000.0 && c.accel_ms[0] == 80 && c.decel_ms[0] == 90,
          "movel 默认速度参数错误（spd=%.0f,acc=%d,dec=%d）", c.speeds[0], c.accel_ms[0], c.decel_ms[0]);

    /* movel：位姿 + 速度/加减速度（用户示例形式） */
    rc = cmd_parse("movel:150,62,103,-180,0,-180,10,50,50", &c);
    CHECK(rc == CMD_MOSEL, "movel 9 段应解析为 CMD_MOSEL（得到 rc=%d）", rc);
    CHECK(fabs(c.cartesian[0] - 150) < 1e-6 && fabs(c.cartesian[3] + 180) < 1e-6,
          "movel 9 段位姿解析失败（X=%.4f, Rx=%.4f）", c.cartesian[0], c.cartesian[3]);
    CHECK(c.speeds[0] == 10.0 && c.accel_ms[0] == 50 && c.decel_ms[0] == 50,
          "movel 速度参数解析失败（spd=%.0f,acc=%d,dec=%d）", c.speeds[0], c.accel_ms[0], c.decel_ms[0]);

    /* movel 段数 / 参数校验 */
    CHECK(cmd_parse("movel:1,2,3,4,5", &c) == CMD_UNKNOWN, "movel 5 段应拒绝");
    CHECK(cmd_parse("movel:1,2,3,4,5,6,7", &c) == CMD_UNKNOWN, "movel 7 段应拒绝");
    CHECK(cmd_parse("movel:1,2,3,4,5,6,7,8", &c) == CMD_UNKNOWN, "movel 8 段应拒绝");
    CHECK(cmd_parse("movel:1,2,3,4,5,6,0,80,90", &c) == CMD_UNKNOWN, "movel 速度 0 应拒绝");

    /* ---- 多关节 movej 段数不足 / 脏段：必须判错（防静默错位与 atof 误读） ---- */
    CHECK(cmd_parse("movej:10,20,30,40,50,60,3000,150", &c) == CMD_UNKNOWN,
          "movej 多关节缺一段应被拒绝");
    CHECK(cmd_parse("movej:10,20,30,40,50,60,3000,150,200,9", &c) == CMD_UNKNOWN,
          "movej 多关节段数过多应被拒绝");
    CHECK(cmd_parse("movej:10,20,30,40,50,abc,3000,150,200", &c) == CMD_UNKNOWN,
          "movej 角度非纯数字应被拒绝");
    CHECK(cmd_parse("movej:10,20,30,40,50,60,3000,150,200x", &c) == CMD_UNKNOWN,
          "movej 末项含非数字尾缀应被拒绝");
    CHECK(cmd_parse("movej:10,20,30,40,50,60,0,150,200", &c) == CMD_UNKNOWN,
          "movej 多关节速度 0 应被拒绝");
    CHECK(cmd_parse("movej:10,20,30,40,50,60,3000,0,200", &c) == CMD_UNKNOWN,
          "movej 多关节加速时间 0 应被拒绝");
    CHECK(cmd_parse("movej:1:45:0", &c) == CMD_UNKNOWN,
          "movej 单关节速度 0 应被拒绝");
    CHECK(cmd_parse("movej:1:45:200:9", &c) == CMD_UNKNOWN,
          "movej 单关节多余字段应被拒绝");

    /* ---- enable[:关节号] ---- */
    rc = cmd_parse("enable", &c);
    CHECK(rc == CMD_ENABLE && c.joint == 0, "enable 应解析为 CMD_ENABLE joint=0（得到 rc=%d, joint=%d）", rc, c.joint);
    rc = cmd_parse("enable:3", &c);
    CHECK(rc == CMD_ENABLE && c.joint == 3, "enable:3 应解析为 CMD_ENABLE joint=3（得到 rc=%d, joint=%d）", rc, c.joint);
    CHECK(cmd_parse("enable:9", &c) == CMD_UNKNOWN,
          "enable:9 电机不存在应被拒绝（不得静默误报为全轴使能）");
    CHECK(cmd_parse("enable:abc", &c) == CMD_UNKNOWN,
          "enable:abc 非数字关节号应被拒绝");

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
