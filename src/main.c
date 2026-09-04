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

/* cmd_status：持续刷新全部关节状态，按回车退出 */
static int cmd_status(Robot *robot)
{
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    DWORD old_mode = 0;

    /* 设为非阻塞输入模式 */
    GetConsoleMode(hStdin, &old_mode);
    SetConsoleMode(hStdin, old_mode & ~(ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT));

    printf("持续刷新状态中（按任意键退出）...\n\n");

    while (1) {
        int j;
        static const int reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;

        /* 光标回到行首，刷新输出 */
        printf("\033[H\033[J");
        printf("持续刷新状态中（按任意键退出）...\n\n");
        printf("  %-4s %-6s %-8s %-10s %-8s %-8s %-8s %-6s %-4s %-4s %s\n",
               "关节", "在线", "状态字", "位置(步)", "角度", "电流mA", "速度rpm", "报警", "IN0", "IN1", "标志");
        printf("  ---- ------ -------- ---------- -------- -------- -------- ---- ---- ---- ----\n");

        for (j = 1; j <= 6; j++) {
            uint32_t st32 = 0;
            int ok = 0;
            int32_t pos;
            int cur, spd, alm;
            ErrCode rc;

            if (robot_is_masked(robot, j)) {
                printf("  %-4d 已屏蔽\n", j);
                continue;
            }
            rc = robot_read_status32(robot, j, &st32);
            if (rc != ERR_NONE) {
                printf("  %-4d 离线\n", j);
                continue;
            }
            pos = robot_read_position_steps(robot, j, &ok);
            cur = robot_read_current_ma(robot, j);
            spd = robot_read_speed_rpm(robot, j);
            alm = robot_read_alarm(robot, j);

            {
                char flags[64] = "";
                int in0 = (st32 & LEESN_STAT_INPUT(0)) ? 1 : 0;
                int in1 = (st32 & LEESN_STAT_INPUT(1)) ? 1 : 0;
                double angle = ok ? STEPS2DEG(pos, reductions[j - 1]) : 0.0;

                if (st32 & LEESN_STAT_INPOS)      strcat(flags, "到位 ");
                if (st32 & LEESN_STAT_SOFT_NEG)   strcat(flags, "负限位 ");
                if (st32 & LEESN_STAT_SOFT_POS)   strcat(flags, "正限位 ");
                if (st32 & LEESN_STAT_HOMED)      strcat(flags, "原点 ");
                if (st32 & LEESN_STAT_ENABLE_LVL) strcat(flags, "使能 ");
                if (st32 & LEESN_STAT_ALARM)      strcat(flags, "报警!");

                printf("  %-4d %-6s 0x%06X %-10d %-8.2f %-8d %-8d %-6s %-4d %-4d %s\n",
                       j, "在线", (unsigned)(st32 & 0xFFFFFFu),
                       ok ? (int)pos : 0,
                       angle,
                       cur >= 0 ? cur : 0,
                       spd >= 0 ? spd : 0,
                       alm >= 0 ? leesn_alarm_text(alm) : "?",
                       in0, in1,
                       flags[0] ? flags : "—");
            }
        }

        fflush(stdout);

        /* 检查是否有按键 */
        if (WaitForSingleObject(hStdin, 300) == WAIT_OBJECT_0) {
            INPUT_RECORD ir;
            DWORD read;
            PeekConsoleInputA(hStdin, &ir, 1, &read);
            if (read > 0) {
                ReadConsoleInputA(hStdin, &ir, 1, &read);
                if (ir.EventType == KEY_EVENT && ir.Event.KeyEvent.bKeyDown) {
                    break;
                }
            }
        }
    }

    /* 恢复控制台模式 */
    SetConsoleMode(hStdin, old_mode);
    printf("\n状态刷新已停止。\n");
    return 0;
}

/* cmd_scan：扫描总线电机在线状态
 * 对关节 1..6 逐个发状态查询（读 0x0006），收到响应即在线。
 * 拨码型驱动器总线地址由拨码决定，扫描只判断“有无响应”，不读 0x0066。 */
static int cmd_scan(Robot *robot)
{
    int j;
    int online_list[6];
    int online_count = 0;
    int masked_count = 0;

    printf("总线电机扫描（关节 1..6）...\n\n");

    for (j = 1; j <= 6; j++) {
        if (robot_is_masked(robot, j)) {
            printf("  关节 %d: 已屏蔽（跳过）\n", j);
            masked_count++;
            continue;
        }
        if (robot_is_online(robot, j)) {
            printf("  关节 %d: 在线\n", j);
            online_list[online_count++] = j;
        } else {
            printf("  关节 %d: 离线（无响应）\n", j);
        }
        Sleep(30); /* 给总线留方向切换余量 */
    }

    printf("\n扫描结果: %d/6 在线", online_count);
    if (online_count > 0) {
        printf("，在线关节 ");
        for (int k = 0; k < online_count; k++) {
            printf("%s%d", k > 0 ? "," : "", online_list[k]);
        }
    }
    if (masked_count > 0) {
        printf("，%d 个关节被屏蔽", masked_count);
    }
    printf("\n");
    return 0;
}

/* main：程序入口，初始化机器人并进入交互命令循环 */
int main(int argc, char **argv)
{
    char port[64];
    unsigned long baud;
    Robot *robot;
    Monitor *mon = NULL;   /* 后台监控线程对象（退出前停止） */
    char line[256];
    int running = 1;

#ifdef _WIN32
    SetConsoleOutputCP(65001); /* 控制台 UTF-8，保证中文正常显示 */
#endif

    log_set_level(LOG_LEVEL_INFO);
    printf("DummyL-Robot 控制台 (C11 + MinGW)\n");
    printf("输入 help 查看命令，exit 退出。\n\n");

    if (argc > 1) {
        snprintf(port, sizeof(port), "%s", argv[1]);
    } else {
        int ini_loaded = ini_read_serial(INI_PATH, port, sizeof(port), &baud);
        if (ini_loaded) {
            LOG_INFO("生效来源：ini（%s），串口 %s @ %lu 8N1", INI_PATH, port, baud);
        } else {
            LOG_INFO("生效来源：默认（ini 缺失，回退 robot_config.h 默认值），串口 %s @ %lu 8N1", port, baud);
        }

        char found_ports[16][64];
        int port_count = 0;

        HKEY hKey;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE,
                         "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                         0, KEY_READ, &hKey) == ERROR_SUCCESS) {
            for (int i = 0; i < 256 && port_count < 16; i++) {
                char value_name[256];
                DWORD name_len = sizeof(value_name);
                BYTE value_data[64];
                DWORD data_len = sizeof(value_data);
                DWORD value_type;
                if (RegEnumValueA(hKey, i, value_name, &name_len, NULL,
                                  &value_type, value_data, &data_len) != ERROR_SUCCESS) {
                    break;
                }
                if (value_type == REG_SZ && data_len > 0) {
                    char *com_name = (char *)value_data;
                    if (strncmp(com_name, "COM", 3) == 0) {
                        snprintf(found_ports[port_count], 64, "%s", com_name);
                        port_count++;
                    }
                }
            }
            RegCloseKey(hKey);
        }

        for (int i = 0; i < port_count - 1; i++) {
            for (int j = i + 1; j < port_count; j++) {
                int a = atoi(found_ports[i] + 3);
                int b = atoi(found_ports[j] + 3);
                if (a > b) {
                    char tmp[64];
                    strcpy(tmp, found_ports[i]);
                    strcpy(found_ports[i], found_ports[j]);
                    strcpy(found_ports[j], tmp);
                }
            }
        }

        if (port_count == 0) {
            LOG_ERROR("未发现可用串口");
            return 1;
        }

        if (port_count == 1) {
            snprintf(port, sizeof(port), "%s", found_ports[0]);
        } else {
            printf("\n检测到多个串口：\n");
            for (int i = 0; i < port_count; i++) {
                printf("  [%d] %s\n", i + 1, found_ports[i]);
            }
            printf("请选择串口编号 (1-%d): ", port_count);
            fflush(stdout);
            char sel_line[16];
            if (fgets(sel_line, sizeof(sel_line), stdin) != NULL) {
                int sel = atoi(sel_line) - 1;
                if (sel >= 0 && sel < port_count) {
                    snprintf(port, sizeof(port), "%s", found_ports[sel]);
                } else {
                    LOG_ERROR("无效选择");
                    return 1;
                }
            } else {
                return 1;
            }
        }
        printf("\n使用串口: %s @ %lu 8N1\n", port, baud);
    }

    /* 注入串口 CommOps（方案一：control 层通过接口操作总线） */
    modbus_comm_set(&serial_comm_ops);

    robot = robot_init(port, (uint32_t)baud);
    if (robot == NULL) {
        LOG_ERROR("初始化失败，请检查串口连接与 robot_config.ini");
        return 1;
    }

    /* 后台定时监控：程序启动即创建线程，按周期巡检六轴状态/电流/报警 */
    mon = monitor_create(robot, 0); /* 0=不启用电流堵转事件，只做状态监控 */
    if (mon == NULL) {
        LOG_WARN("监控器创建失败，继续运行");
    } else if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS)) {
        LOG_WARN("监控线程启动失败，继续运行");
        monitor_destroy(mon);
        mon = NULL;
    } else {
        LOG_INFO("后台监控线程已启动：周期 %d ms 巡检六轴状态/电流/报警",
                 (int)MONITOR_DEFAULT_INTERVAL_MS);
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
            /* 回零期间暂停后台监控，避免其全轴扫描与堵转采样抢占 RS485 总线 */
            monitor_stop(mon);
            ErrCode rc = (cmd.joint >= 1) ? robot_home_single(robot, cmd.joint)
                                          : robot_home(robot);
            if (rc != ERR_NONE) LOG_ERROR("回零失败：%s", err_str(rc));
            if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
                LOG_WARN("回零后监控线程重启失败");
            break;
        }
        case CMD_HOMEJ: {
            monitor_stop(mon);
            ErrCode rc = robot_home_joint(robot, cmd.joint, cmd.angle_deg, cmd.speed_rpm);
            if (rc != ERR_NONE) LOG_ERROR("单轴回零失败：%s", err_str(rc));
            if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
                LOG_WARN("回零后监控线程重启失败");
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
        case CMD_MASK:
        case CMD_UNMASK: {
            int is_mask = (cmd.type == CMD_MASK);
            if (cmd.joint_count > 0) {
                for (int k = 0; k < cmd.joint_count; k++) {
                    ErrCode rc = is_mask ? robot_mask(robot, cmd.joints[k])
                                         : robot_unmask(robot, cmd.joints[k]);
                    if (rc != ERR_NONE) {
                        LOG_ERROR("%s关节%d失败：%s", is_mask ? "屏蔽" : "恢复",
                                  cmd.joints[k], err_str(rc));
                    }
                }
            } else {
                ErrCode rc = is_mask ? robot_mask(robot, cmd.joint)
                                     : robot_unmask(robot, cmd.joint);
                if (rc != ERR_NONE) LOG_ERROR("%s失败：%s", is_mask ? "屏蔽" : "恢复", err_str(rc));
            }
            break;
        }
        case CMD_SCAN:
            cmd_scan(robot);
            break;
        case CMD_CALIB:
            LOG_INFO("单关节调试功能未启用");
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

    /* 先停后台监控线程，再关总线 */
    if (mon != NULL) {
        monitor_stop(mon);
        monitor_destroy(mon);
        mon = NULL;
    }
    robot_close(robot);
    printf("已退出。\n");
    return 0;
}
