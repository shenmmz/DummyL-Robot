/*
 * main.c —— DummyL-Robot 命令行控制台主程序
 * ------------------------------------------------------------
 * 所属模块：应用入口层
 * 对外接口：main（入口）
 * 依赖模块：control/robot、control/monitor、utils/logger、utils/cmd_parser
 */

/*
 * DummyL-Robot CLI 主程序
 * ------------------------------------------------------------
 * 交互循环：读取 robot_config.ini 获取串口参数，
 * 支持 home / movej / enable / disable / status / scan / calib / help / exit。
 * 用法: dummyrobot [串口名，如 COM3]
 */

#include "control/robot.h"
#include "control/monitor.h"
#include "utils/logger.h"
#include "utils/cmd_parser.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define INI_PATH "config/robot_config.ini"

/* 极简 ini 读取：取 [serial] 段下 key 的 value（去除空白），找不到返回默认 */
static void ini_read_serial(const char *path, char *port, size_t port_sz, unsigned long *baud)
{
    FILE *f;
    char line[256];
    int in_serial = 0;

    snprintf(port, port_sz, "COM3");
    *baud = 115200UL;

    f = fopen(path, "r");
    if (f == NULL) {
        return;
    }
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') {
            continue;
        }
        if (*p == '[') {
            in_serial = (strncmp(p, "[serial]", 8) == 0) ? 1 : 0;
            continue;
        }
        if (!in_serial) {
            continue;
        }
        if (strncmp(p, "port", 4) == 0) {
            char *eq = strchr(p, '=');
            if (eq) {
                char *v = eq + 1;
                size_t n = strlen(v);
                while (n > 0 && (v[n - 1] == '\n' || v[n - 1] == '\r' || v[n - 1] == ' ' || v[n - 1] == '\t')) {
                    v[--n] = '\0';
                }
                while (*v == ' ' || *v == '\t') v++;
                if (*v) {
                    snprintf(port, port_sz, "%s", v);
                }
            }
        } else if (strncmp(p, "baudrate", 8) == 0) {
            char *eq = strchr(p, '=');
            if (eq) {
                *baud = strtoul(eq + 1, NULL, 10);
            }
        }
    }
    fclose(f);
}

/* cmd_status：查询并打印全部关节的状态/位置/电流 */
static int cmd_status(Robot *robot)
{
    int j;
    LOG_INFO("关节状态查询：");
    for (j = 1; j <= 6; j++) {
        int st, ok = 0;
        int32_t pos;
        int cur;
        if (robot_is_masked(robot, j)) {
            printf("  关节%d: 已屏蔽\n", j);
            continue;
        }
        st = robot_read_status(robot, j);
        pos = robot_read_position_steps(robot, j, &ok);
        cur = robot_read_current_ma(robot, j);
        if (st >= 0) {
            printf("  关节%d: 在线 状态=0x%04X 位置=%d步 电流=%dmA\n", j, st, (int)pos, cur);
        } else {
            printf("  关节%d: 离线\n", j);
        }
    }
    return 0;
}

/* main：程序入口，初始化机器人并进入交互命令循环 */
int main(int argc, char **argv)
{
    char port[64];
    unsigned long baud;
    Robot *robot;
    char line[256];
    int running = 1;

#ifdef _WIN32
    SetConsoleOutputCP(65001); /* 控制台 UTF-8，保证中文正常显示 */
#endif

    log_set_level(LOG_LEVEL_INFO);
    printf("DummyL-Robot 控制台 (C11 + MinGW)\n");
    printf("输入 help 查看命令，exit 退出。\n\n");

    ini_read_serial(INI_PATH, port, sizeof(port), &baud);
    if (argc > 1) {
        snprintf(port, sizeof(port), "%s", argv[1]);
    }
    LOG_INFO("打开串口 %s @ %lu 8N1", port, baud);

    robot = robot_init(port, (uint32_t)baud);
    if (robot == NULL) {
        LOG_ERROR("初始化失败，请检查串口连接与 robot_config.ini");
        return 1;
    }

    while (running) {
        ParsedCmd cmd;
        printf("DummyL> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL) {
            break;
        }
        cmd_parse(line, &cmd);

        switch (cmd.type) {
        case CMD_HOME:
            robot_home(robot);
            break;
        case CMD_MOVEJ:
            robot_movej(robot, cmd.joint, cmd.angle_deg, cmd.speed_rpm);
            break;
        case CMD_ENABLE:
            robot_enable(robot, cmd.joint);
            break;
        case CMD_DISABLE:
            robot_disable(robot, cmd.joint);
            break;
        case CMD_STATUS:
            cmd_status(robot);
            break;
        case CMD_MASK:
            robot_mask(robot, cmd.joint);
            break;
        case CMD_UNMASK:
            robot_unmask(robot, cmd.joint);
            break;
        case CMD_SCAN:
            LOG_INFO("总线扫描请运行 scan_motors 工具");
            break;
        case CMD_CALIB:
            LOG_INFO("单关节调试请运行 servo_calib 工具");
            break;
        case CMD_HELP:
            cmd_print_help();
            break;
        case CMD_EXIT:
            running = 0;
            break;
        case CMD_EMPTY:
            break;
        default:
            LOG_WARN("未知命令，输入 help 查看帮助");
            break;
        }
    }

    robot_close(robot);
    printf("已退出。\n");
    return 0;
}
