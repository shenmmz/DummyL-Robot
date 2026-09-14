#ifndef CMD_PARSER_H
#define CMD_PARSER_H

/*
 * 命令行解析（CLI 交互循环使用）
 * 支持命令：
 *   home             回零（全轴）
 *   home:N           仅单独回零关节 N
 *   movej:N:ANGLE[:SPEED][:r|a]   单关节运动：角度 ANGLE(度)、速度 SPEED(rpm)，
 *                                   末段 r=相对当前位置 / a=绝对(默认)
 *   movej:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC   多关节同步绝对运动
 *   movel:X,Y,Z,Rx,Ry,Rz    绝对笛卡尔坐标运动（mm, 度）
 *   disable          全部失能所有关节
 *   disable:N        仅单独泄力(失能)关节 N
 *   enable           全部使能所有关节
 *   enable:N         仅单独使能关节 N
 *   motor            启动/停止电机实时监控（3S/次循环显示）
 *   getpos           读取当前关节角(度)与笛卡尔坐标(X,Y,Z,RPY)
 *   zero             显示当前零点与机械角
 *   zero_save:v1,v2,v3,v4,v5,v6   直接写入 6 个电机角零点（逗号分隔）并持久化到 ini
 *   help             帮助（? 同义）
 *   exit             退出（quit 同义）
 */

#define CMD_UNKNOWN  0
#define CMD_HOME     1
#define CMD_MOVEJ    2
#define CMD_DISABLE  3
#define CMD_ENABLE   4
#define CMD_MOTOR    5
#define CMD_HELP     6
#define CMD_EXIT     7
#define CMD_EMPTY    8
#define CMD_ZERO     9
#define CMD_GETPOS   10
#define CMD_ZERO_SAVE 11
#define CMD_MOSEL    12

typedef struct {
    int      type;       /* CMD_* */
    int      joint;      /* 1..6（关节号），0=全部/未指定 */
    double   angle_deg;  /* 目标角度（度）：rel=0 为绝对，rel=1 为相对增量 */
    double   speed_rpm;  /* 速度（rpm），0表示使用默认 */
    int      rel;        /* 单关节 movej 模式：1=相对当前位置，0=绝对（默认） */
    double   zero_vals[6]; /* zero_save 写入的 6 个电机角零点 */
    int      num_joints; /* 多关节 movej 的关节数 */
    int      joints[6];  /* 多关节 movej 的关节号 */
    double   angles[6];  /* 多关节 movej 的目标角度 */
    double   speeds[6];  /* 多关节 movej 的速度 */
    int      accel_ms[6]; /* 多关节 movej 的加速度 ms */
    int      decel_ms[6]; /* 多关节 movej 的减速度 ms */
    double   cartesian[6]; /* movel 目标位姿 X,Y,Z,Rx,Ry,Rz */
    char     raw[128];   /* 原始输入 */
} ParsedCmd;

/* 解析一行输入，成功返回 CMD_*，解析失败返回 CMD_UNKNOWN */
int cmd_parse(const char *line, ParsedCmd *out);

/* 打印 CLI 命令帮助文本 */
void cmd_print_help(void);

#endif /* CMD_PARSER_H */
