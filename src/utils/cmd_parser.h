#ifndef CMD_PARSER_H
#define CMD_PARSER_H

/*
 * 命令行解析（CLI 交互循环使用）
 * 支持命令：
 *   home             回零
 *   movej:N:ANGLE    单关节绝对运动（例 movej:1:45，角度为度）
 *   movej:N:ANGLE:SPEED  指定速度（rpm）
 *   enable:N         使能关节
 *   disable:N        失能关节
 *   status           查询所有关节状态
 *   scan             扫描总线电机
 *   calib            单关节手动调试
 *   help             命令帮助
 *   exit             退出
 */

#include <stdint.h>

#define CMD_UNKNOWN  0
#define CMD_HOME     1
#define CMD_MOVEJ    2
#define CMD_ENABLE   3
#define CMD_DISABLE  4
#define CMD_STATUS   5
#define CMD_SCAN     6
#define CMD_CALIB    7
#define CMD_HELP     8
#define CMD_EXIT     9
#define CMD_EMPTY    10
#define CMD_MASK     11
#define CMD_UNMASK   12
#define CMD_HOMEJ    13

typedef struct {
    int      type;       /* CMD_* */
    int      joint;      /* 1..6（单个） */
    int      joints[6];  /* mask/unmask 批量关节列表（1..6） */
    int      joint_count;/* 批量个数；0 = 未使用批量 */
    double   angle_deg;  /* movej 目标角度（度） */
    double   speed_rpm;  /* movej 可选速度（rpm），0 表示使用默认 */
    char     raw[128];   /* 原始输入 */
} ParsedCmd;

/* 解析一行输入，成功返回 CMD_*，解析失败返回 CMD_UNKNOWN */
int cmd_parse(const char *line, ParsedCmd *out);

/* 打印 help 文本（精简中文） */
void cmd_print_help(void);

#endif /* CMD_PARSER_H */
