#ifndef CMD_PARSER_H
#define CMD_PARSER_H

/*
 * 命令行解析（CLI 交互循环使用）
 * 支持命令：
 *   home             回零（全轴）
 *   home:N           仅单独回零关节 N
 *   movej:N:ANGLE[:SPEED]  控制轴N，绝对角度ANGLE(度)，速度SPEED(rpm)
 *   disable          全部失能所有关节
 *   motor            启动/停止电机实时监控（3S/次循环显示）
 *   getpos           读取当前关节角(度)与笛卡尔坐标(X,Y,Z,RPY)
 *   zero[:save]      显示/保存零点标定数据
 *   help             帮助
 *   exit             退出
 */

#define CMD_UNKNOWN  0
#define CMD_HOME     1
#define CMD_MOVEJ    2
#define CMD_DISABLE  3
#define CMD_MOTOR    4
#define CMD_HELP     5
#define CMD_EXIT     6
#define CMD_EMPTY    7
#define CMD_ZERO     8
#define CMD_GETPOS   9

typedef struct {
    int      type;       /* CMD_* */
    int      joint;      /* 1..6（关节号），0=全部/未指定 */
    double   angle_deg;  /* 目标绝对角度（度） */
    double   speed_rpm;  /* 速度（rpm），0表示使用默认 */
    char     raw[128];   /* 原始输入 */
} ParsedCmd;

/* 解析一行输入，成功返回 CMD_*，解析失败返回 CMD_UNKNOWN */
int cmd_parse(const char *line, ParsedCmd *out);

/* 打印 CLI 命令帮助文本 */
void cmd_print_help(void);

#endif /* CMD_PARSER_H */
