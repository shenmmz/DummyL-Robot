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
#include "api/motor_reg.h"
#include "control/home.h"
#include "control/monitor.h"
#include "comm/serial_win.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/cmd_parser.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>   /* timeBeginPeriod/timeEndPeriod：抬高系统定时器精度 */
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

        for (j = 1; j <= 6; j++) {
            uint32_t st32 = 0;
            int ok = 0;
            int32_t pos;
            int cur, spd, alm;
            ErrCode rc;

            if (robot_is_masked(robot, j)) {
                printf("关节:%d,状态:已屏蔽\n", j);
                continue;
            }
            rc = robot_read_status32(robot, j, &st32);
            if (rc != ERR_NONE) {
                printf("关节:%d,状态:离线\n", j);
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

                printf("关节:%d,状态:在线,字:0x%06X,步长:%d,角度:%.2f,电流:%d,速度:%d,报警:%s,IN0:%d,IN1:%d,标志:%s\n",
                       j, (unsigned)(st32 & 0xFFFFFFu),
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

/* cmd_diag：回零诊断读数 —— 辨识"真堵转"与"传动打滑/跳齿"
 * 背景：闭环电机的 0x0004 是编码器位置。顶死后若它仍按满速累加，说明电机轴真的
 * 还在转，那不是堵转而是同步带跳齿/传动打滑（伴随皮带响声）。此时无论怎么调电流
 * 阈值都判不到，且继续顶会磨坏皮带 —— 必须先解决机械侧。
 * 判定要点：
 *   实际速度(0x00D6) ≈ 设定速度 且 电流高            → 电机轴在转 = 打滑/跳齿
 *   实际速度 ≈ 0 且 位置偏差(0x0011) 持续累积        → 命令走、电机没转 = 真堵转
 * 顺带输出细分(0x0024) 与配置 ENCODER_STEPS_PER_REV 的对比，确认角度换算口径。
 * joint_only：1..6 只查该轴并连续采样 3 次（看增量）；0 = 全轴各采样 1 次。 */
static int cmd_diag(Robot *robot, int joint_only)
{
    int j, lo, hi, samples, s;

    if (joint_only >= 1 && joint_only <= 6) {
        lo = hi = joint_only;
        samples = 3;
    } else {
        lo = 1; hi = 6; samples = 1;
    }

    printf("\n回零诊断读数（闭环判据）\n");
    printf("  配置细分 ENCODER_STEPS_PER_REV = %d\n\n", (int)ENCODER_STEPS_PER_REV);
    printf("  %-6s %-15s %-9s %-11s %-10s %-11s %-8s %s\n",
           "关节", "细分(实际/配置)", "编码器线数", "实际速度rpm", "位置偏差",
           "位置(步)", "电流mA", "状态字");
    printf("  ------ --------------- --------- ----------- ---------- ----------- -------- --------\n");

    for (j = lo; j <= hi; j++) {
        int32_t prev_pos = 0;
        int     prev_err = 0;
        int     have_prev = 0;

        if (robot_is_masked(robot, j)) {
            printf("  %-6d 已屏蔽\n", j);
            continue;
        }
        for (s = 0; s < samples; s++) {
            uint32_t st32 = 0;
            int32_t subdiv, pos, spd_raw;
            int     enc_lines, pos_err, cur, pos_ok = 0;
            char    tag[16], subdiv_txt[32], enc_txt[16], spd_txt[16];
            char    err_txt[16], pos_txt[16], cur_txt[16];

            if (motor_read_status(robot, j, &st32) != ERR_NONE) {
                printf("  %-6d 离线\n", j);
                break;
            }
            subdiv    = motor_read_subdivision(robot, j);
            enc_lines = motor_read_enc_lines(robot, j);
            spd_raw   = motor_read_speed_raw(robot, j);
            pos_err   = motor_read_pos_err(robot, j);
            pos       = motor_read_position(robot, j, &pos_ok);
            cur       = motor_read_current(robot, j);

            if (samples > 1) snprintf(tag, sizeof(tag), "%d#%d", j, s + 1);
            else             snprintf(tag, sizeof(tag), "%d", j);

            if (subdiv < 0) {
                snprintf(subdiv_txt, sizeof(subdiv_txt), "读失败");
            } else if (subdiv == (int32_t)ENCODER_STEPS_PER_REV) {
                snprintf(subdiv_txt, sizeof(subdiv_txt), "%d 一致", (int)subdiv);
            } else {
                snprintf(subdiv_txt, sizeof(subdiv_txt), "%d/%d 不符",
                         (int)subdiv, (int)ENCODER_STEPS_PER_REV);
            }
            if (enc_lines >= 0) snprintf(enc_txt, sizeof(enc_txt), "%d", enc_lines);
            else                snprintf(enc_txt, sizeof(enc_txt), "—");
            if (spd_raw >= 0)   snprintf(spd_txt, sizeof(spd_txt), "%.2f", (double)spd_raw / 100.0);
            else                snprintf(spd_txt, sizeof(spd_txt), "—");
            if (pos_err >= 0)   snprintf(err_txt, sizeof(err_txt), "%d", pos_err);
            else                snprintf(err_txt, sizeof(err_txt), "—");
            if (pos_ok)         snprintf(pos_txt, sizeof(pos_txt), "%d", (int)pos);
            else                snprintf(pos_txt, sizeof(pos_txt), "—");
            if (cur >= 0)       snprintf(cur_txt, sizeof(cur_txt), "%d", cur);
            else                snprintf(cur_txt, sizeof(cur_txt), "—");

            printf("  %-6s %-15s %-9s %-11s %-10s %-11s %-8s 0x%06X\n",
                   tag, subdiv_txt, enc_txt, spd_txt, err_txt, pos_txt, cur_txt,
                   (unsigned)(st32 & 0xFFFFFFu));

            if (have_prev && pos_ok && pos_err >= 0 && prev_err >= 0) {
                printf("        └ Δ位置=%+d 步, Δ偏差=%+d 步（%dms 内）\n",
                       (int)(pos - prev_pos), pos_err - prev_err,
                       (samples > 1) ? 400 : 0);
            }
            if (pos_ok)   prev_pos = pos;
            if (pos_err >= 0) prev_err = pos_err;
            have_prev = 1;

            if (samples > 1 && s < samples - 1) Sleep(400);
        }
    }
    if (samples > 1) {
        int k;
        int32_t tp;
        uint32_t ts, t0, dt_pos = 0, dt_cur = 0;

        /* 总线测速：堵转轮询周期 ≈ 每轮事务数 × 单事务耗时。
         * 事务数是可控项（位置+状态合并后已由 3→2），这里量化单事务耗时下限。 */
        t0 = GetTickCount();
        for (k = 0; k < 20; k++) (void)motor_read_pos_status(robot, lo, &tp, &ts);
        dt_pos = GetTickCount() - t0;
        t0 = GetTickCount();
        for (k = 0; k < 20; k++) (void)motor_read_current(robot, lo);
        dt_cur = GetTickCount() - t0;

        printf("\n  总线测速（关节%d，各 20 次）：\n", lo);
        printf("    位置+状态合并读  %u ms / 20 次 = %.1f ms/次\n",
               (unsigned)dt_pos, (double)dt_pos / 20.0);
        printf("    电流读            %u ms / 20 次 = %.1f ms/次\n",
               (unsigned)dt_cur, (double)dt_cur / 20.0);
        printf("    堵转轮询周期 ≈ %.1f ms（2 事务/轮）\n",
               (double)dt_pos / 20.0 + (double)dt_cur / 20.0);
    }
    if (samples > 1) {
        printf("\n  判定提示：\n");
        printf("    实际速度≈设定速度 且 电流高、Δ位置≈额定步数 → 电机轴仍在转 = 打滑/跳齿（机械问题）\n");
        printf("    实际速度≈0、Δ位置≈0 且 Δ偏差持续累积        → 命令走电机不转 = 真堵转（判据侧可解）\n");
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
            printf("[错误] 未发现可用串口\n");
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

    /* 注入串口 CommOps（方案一：control 层通过接口操作总线） */
    modbus_comm_set(&serial_comm_ops);

    robot = robot_init(port, (uint32_t)baud);
    if (robot == NULL) {
        printf("[错误] 初始化失败，请检查串口连接与 robot_config.ini\n");
        return 1;
    }

    /* 后台定时监控：程序启动即创建线程，按周期巡检六轴状态/电流/报警 */
    mon = monitor_create(robot, 0); /* 0=不启用电流堵转事件，只做状态监控 */
    if (mon == NULL) {
        printf("[警告] 监控器创建失败，继续运行\n");
    } else if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS)) {
        printf("[警告] 监控线程启动失败，继续运行\n");
        monitor_destroy(mon);
        mon = NULL;
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
            if (rc != ERR_NONE) printf("[错误] 回零失败：%s\n", err_str(rc));
            if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
                printf("[警告] 回零后监控线程重启失败\n");
            break;
        }
        case CMD_HOMEJ: {
            monitor_stop(mon);
            ErrCode rc = robot_home_joint(robot, cmd.joint, cmd.angle_deg, cmd.speed_rpm);
            if (rc != ERR_NONE) printf("[错误] 单轴回零失败：%s\n", err_str(rc));
            if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
                printf("[警告] 回零后监控线程重启失败\n");
            break;
        }
        case CMD_MOVEJ: {
            ErrCode rc = robot_movej(robot, cmd.joint, cmd.angle_deg, cmd.speed_rpm);
            if (rc != ERR_NONE) printf("[错误] 运动指令失败：%s\n", err_str(rc));
            break;
        }
        case CMD_ENABLE: {
            ErrCode rc = robot_enable(robot, cmd.joint);
            if (rc != ERR_NONE) printf("[错误] 使能失败：%s\n", err_str(rc));
            break;
        }
        case CMD_DISABLE: {
            ErrCode rc = robot_disable(robot, cmd.joint);
            if (rc != ERR_NONE) printf("[错误] 失能失败：%s\n", err_str(rc));
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
                        printf("[错误] %s关节%d失败：%s\n", is_mask ? "屏蔽" : "恢复",
                                  cmd.joints[k], err_str(rc));
                    }
                }
            } else {
                ErrCode rc = is_mask ? robot_mask(robot, cmd.joint)
                                     : robot_unmask(robot, cmd.joint);
                if (rc != ERR_NONE) printf("[错误] %s失败：%s\n", is_mask ? "屏蔽" : "恢复", err_str(rc));
            }
            break;
        }
        case CMD_SCAN:
            cmd_scan(robot);
            break;
        case CMD_DIAG:
            cmd_diag(robot, cmd.joint);
            break;
        case CMD_TORQUE: {
            monitor_stop(mon);
            ErrCode rc = robot_torque_probe(robot, cmd.joint, cmd.torque_level);
            if (rc != ERR_NONE) printf("[错误] 力矩碰撞诊断失败：%s\n", err_str(rc));
            if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
                printf("[警告] 诊断后监控线程重启失败\n");
            break;
        }
        case CMD_CALIB:
            printf("单关节调试功能未启用\n");
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
            printf("[警告] 未知命令，输入 help 查看帮助\n");
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
#ifdef _WIN32
    timeEndPeriod(1);   /* 与 main 开头的 timeBeginPeriod(1) 配对 */
#endif
    printf("已退出。\n");
    return 0;
}
