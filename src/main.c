#include "control/robot.h"
#include "control/monitor.h"
#include "comm/serial_win.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/cmd_parser.h"
#include "utils/ini_rw.h"
#include "utils/telemetry.h"
#include "kinematics/joint_zero.h"
#include "kinematics/dh.h"
#include "cli/commands.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <mmsystem.h>


/* 从 ini 的 [serial] 段读串口名与波特率。 */
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

/* 程序入口：读 ini → 枚举注册表串口（会【覆盖】ini 里的 port）→ robot_init
 * → 起后台监控线程 → 进入命令循环。
 * 两个坑：① robot_init 只要求串口能打开，六轴全离线也不退出（所以不接臂也能跑 looptest）；
 * ② 必须在【项目根目录】运行，INI_PATH 是相对路径。 */
int main(int argc, char **argv)
{
    char port[64];
    unsigned long baud = MODBUS_BAUDRATE;
    Robot *robot;
    Monitor *mon = NULL;
    char line[256];
    int running = 1;

#ifdef _WIN32
    SetConsoleOutputCP(65001);
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

    {
        double loaded[6];
        if (ini_read_joint_zero(INI_PATH, loaded)) {
            joint_zero_save(loaded);
            printf("零点标定来源：ini [joint_zero]\n");
        } else {
            printf("零点标定来源：默认（robot_config.h 编译期宏）\n");
        }
    }

    {
        double tool_mm = 0.0;
        if (ini_read_tool_length(INI_PATH, &tool_mm)) {
            dh_set_tool_length(tool_mm);
            printf("工具长度来源：ini [tool]，tool_length=%.2fmm → d6=%.2fmm\n",
                   tool_mm, DH_TABLE[5].d);
        } else {
            dh_set_tool_length(0.0);
            printf("工具长度来源：默认 0mm（无工具）→ d6=%.2fmm\n", DH_TABLE[5].d);
        }
    }

    telemetry_init(INI_PATH);

    modbus_comm_set(&serial_comm_ops);

    robot = robot_init(port, (uint32_t)baud);
    if (robot == NULL) {
        printf("[错误] 初始化失败，请检查串口连接与 robot_config.ini\n");
        return 1;
    }

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

        mon = monitor_create(robot, th);
    }
    if (mon == NULL) {
        printf("[警告] 监控器创建失败，继续运行\n");
    } else if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS)) {
        printf("[警告] 监控线程启动失败，继续运行\n");
        monitor_destroy(mon);
        mon = NULL;
    }

    cmd_pose_check(robot);

    while (running) {
        ParsedCmd cmd;
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

    if (mon != NULL) {
        monitor_stop(mon);
        monitor_destroy(mon);
        mon = NULL;
    }
    robot_close(robot);
#ifdef _WIN32
    timeEndPeriod(1);
#endif
    printf("已退出。\n");
    return 0;
}
