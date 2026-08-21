/*
 * servo_calib.c —— 单关节手动标定/调试工具（命令行独立程序）
 * ------------------------------------------------------------
 * 所属模块：工具层（tools）
 * 对外接口：main（入口）
 * 依赖模块：control/robot、config/robot_config、utils/logger
 */

/*
 * 单关节手动标定工具
 * ------------------------------------------------------------
 * 交互式单关节调试：使能/失能、绝对运动（度）、读取状态/位置/电流。
 * 用于安装后验证单关节转向、限位与回零方向。
 * 用法: servo_calib [串口名]
 */

#include "control/robot.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

static void print_help(void)
{
    printf("命令:\n");
    printf("  en:N      使能关节 N\n");
    printf("  dis:N     失能关节 N\n");
    printf("  mov:N:A   关节 N 绝对运动到 A 度\n");
    printf("  st:N      读取关节 N 状态/位置/电流\n");
    printf("  help      帮助\n");
    printf("  exit      退出\n");
}

/* main：交互式单关节使能/运动/状态调试入口 */
int main(int argc, char **argv)
{
    const char *port = "COM3";
    Robot *robot;
    char line[128];

#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif

    if (argc > 1) port = argv[1];
    log_set_level(LOG_LEVEL_INFO);
    printf("单关节标定工具，串口 %s\n", port);

    robot = robot_init(port, MODBUS_BAUDRATE);
    if (robot == NULL) {
        LOG_ERROR("初始化失败");
        return 1;
    }

    print_help();
    while (1) {
        char cmd[16] = {0};
        int joint, n;
        double angle;

        printf("calib> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL) {
            break;
        }
        n = sscanf(line, "%15s", cmd);
        if (n != 1) continue;

        if (strcmp(cmd, "help") == 0) {
            print_help();
        } else if (strcmp(cmd, "exit") == 0) {
            break;
        } else if (sscanf(cmd, "en:%d", &joint) == 1) {
            robot_enable(robot, joint);
        } else if (sscanf(cmd, "dis:%d", &joint) == 1) {
            robot_disable(robot, joint);
        } else if (sscanf(cmd, "st:%d", &joint) == 1) {
            int st = robot_read_status(robot, joint);
            int ok = 0;
            int32_t pos = robot_read_position_steps(robot, joint, &ok);
            int cur = robot_read_current_ma(robot, joint);
            printf("关节%d: 状态=0x%04X 位置=%d步 电流=%dmA\n", joint, st, (int)pos, cur);
        } else if (strncmp(cmd, "mov:", 4) == 0) {
            if (sscanf(cmd + 4, "%d:%lf", &joint, &angle) == 2) {
                robot_movej(robot, joint, angle, 0.0);
            } else {
                printf("用法: mov:N:A\n");
            }
        } else {
            printf("未知命令\n");
        }
    }

    robot_close(robot);
    return 0;
}
