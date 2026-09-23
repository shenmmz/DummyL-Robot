#include "control/robot.h"
#include "control/monitor.h"
#include "comm/serial_win.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/cmd_parser.h"
#include "utils/ini_rw.h"
#include "utils/telemetry.h"
#include "kinematics/joint_zero.h"
#include "kinematics/dh.h"   /* dh_set_tool_length：运行时叠加末端工具长到 d6 */
#include "cli/commands.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <mmsystem.h>   /* timeBeginPeriod/timeEndPeriod：抬高系统定时器精度 */

/* INI_PATH 由 utils/ini_rw.h 统一提供，保持 main 与 cli 一致 */

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

/* main：程序入口，初始化机器人并进入交互命令循环 */
int main(int argc, char **argv)
{
    char port[64];
    unsigned long baud = MODBUS_BAUDRATE;   /* argc>1 分支未读 ini，须给默认值避免未初始化 */
    Robot *robot;
    Monitor *mon = NULL;   /* 后台监控线程对象（退出前停止） */
    char line[256];
    int running = 1;

#ifdef _WIN32
    SetConsoleOutputCP(65001); /* 控制台 UTF-8，保证中文正常显示 */
    /* 抬高系统定时器精度到 1ms：Windows 默认 15.6ms，Sleep(n) 中 n<16 一律睡满约 15.6ms。
     * 堵转轮询每轮含多次 Sleep(2)（Modbus 帧间隔）与 Sleep(1)（轮询节拍），
     * 默认精度下每轮白耗数十 ms，直接抬高轮询周期、拖慢堵转判定。 */
    timeBeginPeriod(1);
#endif

    printf("DummyL-Robot 控制台 (C11 + MinGW)\n");
    printf("输入 help 查看命令，exit 退出。\n\n");

    if (argc > 1) {
        snprintf(port, sizeof(port), "%s", argv[1]);
    } else {
        int ini_loaded = ini_read_serial(INI_PATH, port, sizeof(port), &baud);
        if (ini_loaded) {
            printf("生效来源：ini（%s），串口 %s @ %lu 8N1\n", INI_PATH, port, baud);
        } else {
            printf("生效来源：默认（ini 缺失，回退 robot_config.h 默认值），串口 %s @ %lu 8N1\n", port, baud);
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
            /* 换转换器时最容易撞上这一档。原来的单行提示太简略，用户无从下手，
             * 所以把"怎么查"直接印出来 —— 这条路径只会在启动时走一次，多几行不碍事。 */
            printf("[错误] 未发现可用串口\n");
            printf("       程序会枚举系统串口（注册表 HARDWARE\\DEVICEMAP\\SERIALCOMM），当前一个都没有。\n");
            printf("       排查：\n");
            printf("         1) 转换器是否插紧、指示灯是否亮（换 USB 口试一次）\n");
            printf("         2) 设备管理器 → 端口(COM 和 LPT) 里有没有新出现的 COM 口\n");
            printf("         3) 驱动是否装好（CH340 需 WCH 驱动，FTDI / CP210x 各有各的）\n");
            printf("         4) 也可以跳过枚举、直接指定：dummyrobot.exe COM5\n");
            printf("       注意：ini 的 [serial] port 只是参考值，枚举结果会覆盖它 ——\n");
            printf("             所以【换转换器不需要改 ini】，插好重启程序即可。\n");
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
                    printf("[错误] 无效选择\n");
                    return 1;
                }
            } else {
                return 1;
            }
        }
        printf("\n使用串口: %s @ %lu 8N1\n", port, baud);
    }

    /* 读取 ini 的 [joint_zero] 段（若已保存标定），运行时覆盖头文件默认零点 */
    {
        double loaded[6];
        if (ini_read_joint_zero(INI_PATH, loaded)) {
            joint_zero_save(loaded);
            printf("零点标定来源：ini [joint_zero]\n");
        } else {
            printf("零点标定来源：默认（robot_config.h 编译期宏）\n");
        }
    }

    /* 读取 ini 的 [tool] 段，将末端工具长叠加到 d6（法兰偏距 91.5 + tool_length） */
    {
        double tool_mm = 0.0;
        if (ini_read_tool_length(INI_PATH, &tool_mm)) {
            dh_set_tool_length(tool_mm);
            printf("工具长度来源：ini [tool]，tool_length=%.2fmm → d6=%.2fmm\n", tool_mm, DH_D6_FLANGE_MM + tool_mm);
        } else {
            dh_set_tool_length(0.0);
            printf("工具长度来源：默认 0mm（无工具）→ d6=%.2fmm\n", DH_D6_FLANGE_MM);
        }
    }

    /* UDP 遥测：把关节角发给本机 3D 镜像（sim/live_mirror.py）。
     * 默认【关闭】—— ini [telemetry] enabled = 1 才启用。
     * 这里只是初始化 socket，真正的发送在 movej_issue 的下发点，
     * 且全部失败路径静默，绝不影响控制。 */
    telemetry_init(INI_PATH);

    /* 注入串口 CommOps（control 层通过接口操作总线） */
    modbus_comm_set(&serial_comm_ops);

    robot = robot_init(port, (uint32_t)baud);
    if (robot == NULL) {
        printf("[错误] 初始化失败，请检查串口连接与 robot_config.ini\n");
        return 1;
    }

    /* 逐轴堵转阈值：优先 ini [stall] j1..j6，读不到用编译期默认表（全 0 = 全关）。
     * 【旧行为是 monitor_create(robot, 0)】——传 0 等于堵转检测从来没开过，
     * 机械臂撞上东西不会有任何反应。现在改成从配置读，填了值就真的生效。 */
    {
        int th[ROBOT_JOINT_COUNT];
        const int def[ROBOT_JOINT_COUNT] = ROBOT_STALL_CURRENT_MA_TABLE;
        int i, on = 0;
        int from_ini = ini_read_stall_current(INI_PATH, th);
        if (!from_ini) {
            for (i = 0; i < ROBOT_JOINT_COUNT; i++) th[i] = def[i];
        }
        for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
            if (th[i] > 0) on++;
        }
        printf("堵转阈值来源：%s（j1..j6 = %d,%d,%d,%d,%d,%d mA，%d 轴启用）\n",
               from_ini ? "ini [stall]" : "默认（robot_config.h）",
               th[0], th[1], th[2], th[3], th[4], th[5], on);
        if (on == 0) {
            printf("[提示] 堵转保护【未启用】：六轴阈值都是 0。\n"
                   "       先跑 curtest 量出各轴正常电流，再在 ini [stall] 填 j1..j6 并重启。\n");
        }

        /* 后台定时监控：程序启动即创建线程，按周期巡检六轴状态/电流/报警 */
        mon = monitor_create(robot, th);
    }
    if (mon == NULL) {
        printf("[警告] 监控器创建失败，继续运行\n");
    } else if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS)) {
        printf("[警告] 监控线程启动失败，继续运行\n");
        monitor_destroy(mon);
        mon = NULL;
    }

    /* 启动位姿体检：驱动器掉电会清空 0x00D2（RAM 无记忆零点），位置计数归零
     * 而 ini 标定还在 ⇒ 机械角全是假值（2026-09-18 实测读出 6 轴全越软限位）。
     * 越限时 cmd_dispatch 会锁住 MoveL/MoveJ/curtest，逼着先回零。 */
    cmd_pose_check(robot);

    /* 命令循环：读取一行 → 解析 → 分发（具体命令实现见 cli/commands.c） */
    while (running) {
        ParsedCmd cmd;
        /* 当电机监控线程运行时，不打印提示符，避免与线程输出交错 */
        if (cmd_motor_running()) {
            HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
            if (WaitForSingleObject(hStdin, 100) == WAIT_OBJECT_0) {
                INPUT_RECORD ir;
                DWORD read = 0;
                PeekConsoleInputA(hStdin, &ir, 1, &read);
                if (read > 0) {
                    ReadConsoleInputA(hStdin, &ir, 1, &read);
                    if (ir.Event.KeyEvent.bKeyDown &&
                        (ir.Event.KeyEvent.uChar.AsciiChar == 'q' ||
                         ir.Event.KeyEvent.uChar.AsciiChar == 'Q')) {
                        ParsedCmd stop_cmd;
                        memset(&stop_cmd, 0, sizeof(stop_cmd));
                        stop_cmd.type = CMD_MOTOR;
                        cmd_dispatch(robot, mon, &stop_cmd);
                        continue;
                    }
                }
            }
            continue;
        }
        printf("DummyL> ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL) {
            break;
        }
        cmd_parse(line, &cmd);
        if (cmd_dispatch(robot, mon, &cmd)) {
            running = 0;
        }
    }

    /* 先停后台监控线程，再关总线 */
    if (mon != NULL) {
        monitor_stop(mon);
        monitor_destroy(mon);
        mon = NULL;
    }
    robot_close(robot);
#ifdef _WIN32
    timeEndPeriod(1);   /* 与 main 开头的 timeBeginPeriod(1) 配对 */
#endif
    printf("已退出。\n");
    return 0;
}
