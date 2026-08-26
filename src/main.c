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
#include "control/robot_internal.h"
#include "control/home.h"
#include "control/monitor.h"
#include "comm/serial_win.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/logger.h"
#include "utils/cmd_parser.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#define INI_PATH "config/robot_config.ini"

/* 极简 ini 读取：取 [serial] 段下 key 的 value（去除空白），找不到返回默认。
 * 返回 1 = ini 文件存在并已装载（生效来源：ini）；
 * 返回 0 = ini 缺失/无法打开，回退默认值宏（生效来源：默认）。 */
static int ini_read_serial(const char *path, char *port, size_t port_sz, unsigned long *baud)
{
    FILE *f;
    char line[256];
    int in_serial = 0;

    snprintf(port, port_sz, "COM3");
    *baud = MODBUS_BAUDRATE;

    f = fopen(path, "r");
    if (f == NULL) {
        return 0;
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
    return 1;
}

/* cmd_status：查询并打印全部关节的状态/位置/电流（完整 32 位状态字解析） */
static int cmd_status(Robot *robot)
{
    int j;
    LOG_INFO("关节状态查询：");
    for (j = 1; j <= 6; j++) {
        uint32_t st32 = 0;
        int ok = 0;
        int32_t pos;
        int cur;
        ErrCode rc;
        if (robot_is_masked(robot, j)) {
            printf("  关节%d: 已屏蔽\n", j);
            continue;
        }
        rc = robot_read_status32(robot, j, &st32);
        pos = robot_read_position_steps(robot, j, &ok);
        cur = robot_read_current_ma(robot, j);
        if (rc == ERR_NONE) {
            const char *run = "空闲";
            switch (st32 & LEESN_STAT_RUN_MASK) {
            case LEESN_STAT_RUN_START:  run = "即将启动"; break;
            case LEESN_STAT_RUN_STOP:   run = "即将停止"; break;
            case LEESN_STAT_RUN_ACTIVE: run = "正在运行"; break;
            default:                    run = "空闲";     break;
            }
            printf("  关节%d: 在线 状态=0x%08X 运行=%s 位置=%d步 电流=%dmA\n",
                   j, (unsigned)st32, run, (int)pos, cur);
            printf("         [%s%s%s%s%s%s]\n",
                   (st32 & LEESN_STAT_INPOS)      ? "到位" : "",
                   (st32 & LEESN_STAT_SOFT_NEG)   ? "负限位" : "",
                   (st32 & LEESN_STAT_SOFT_POS)   ? "正限位" : "",
                   (st32 & LEESN_STAT_HOMED)      ? "原点完成" : "",
                   (st32 & LEESN_STAT_ENABLE_LVL) ? "使能" : "",
                   (st32 & LEESN_STAT_ALARM)      ? "报警" : "");
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

    int ini_loaded = ini_read_serial(INI_PATH, port, sizeof(port), &baud);
    if (argc > 1) {
        snprintf(port, sizeof(port), "%s", argv[1]);
    }
    /* 方案二：配置单一来源——打印运行时参数生效来源（ini 缺失回退默认值宏） */
    if (ini_loaded) {
        LOG_INFO("生效来源：ini（%s），串口 %s @ %lu 8N1", INI_PATH, port, baud);
    } else {
        LOG_INFO("生效来源：默认（ini 缺失，回退 robot_config.h 默认值），串口 %s @ %lu 8N1", port, baud);
    }

    /* 注入串口 CommOps（方案一：control 层通过接口操作总线） */
    modbus_comm_set(&serial_comm_ops);

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
        case CMD_HOME: {
            ErrCode rc = robot_home(robot);
            if (rc != ERR_NONE) LOG_ERROR("回零失败：%s", err_str(rc));
            break;
        }
        case CMD_MOVEJ: {
            ErrCode rc = robot_movej(robot, cmd.joint, cmd.angle_deg, cmd.speed_rpm);
            if (rc != ERR_NONE) LOG_ERROR("运动指令失败：%s", err_str(rc));
            break;
        }
        case CMD_ENABLE: {
            ErrCode rc = robot_enable(robot, cmd.joint);
            if (rc != ERR_NONE) LOG_ERROR("使能失败：%s", err_str(rc));
            break;
        }
        case CMD_DISABLE: {
            ErrCode rc = robot_disable(robot, cmd.joint);
            if (rc != ERR_NONE) LOG_ERROR("失能失败：%s", err_str(rc));
            break;
        }
        case CMD_STATUS:
            cmd_status(robot);
            break;
        case CMD_MASK: {
            ErrCode rc = robot_mask(robot, cmd.joint);
            if (rc != ERR_NONE) LOG_ERROR("屏蔽失败：%s", err_str(rc));
            break;
        }
        case CMD_UNMASK: {
            ErrCode rc = robot_unmask(robot, cmd.joint);
            if (rc != ERR_NONE) LOG_ERROR("恢复失败：%s", err_str(rc));
            break;
        }
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
