#ifndef CMD_PARSER_H
#define CMD_PARSER_H


#define CMD_UNKNOWN   0
#define CMD_HOME      1
#define CMD_MOVEJ     2
#define CMD_DISABLE   3
#define CMD_ENABLE    4
#define CMD_MOTOR     5
#define CMD_HELP      6
#define CMD_EXIT      7
#define CMD_EMPTY     8
#define CMD_ZERO      9
#define CMD_GETPOS    10
#define CMD_ZERO_SAVE 11
#define CMD_MOVEL     13
#define CMD_FK        14
#define CMD_DIAG      15
#define CMD_BCAST     16
#define CMD_NRTEST    17
#define CMD_CURTEST   18
#define CMD_STALL     19
#define CMD_POSEOK    20
#define CMD_BUSRATE   22
#define CMD_TABTEST   21
#define CMD_ACCEL     23
#define CMD_ALARM     24
#define CMD_LOOPTEST  25
#define CMD_DRVBAUD   26
#define CMD_PIPE      27

/* movel 模式：★ 2026-09-28 起【只保留 smooth】。
 * smooth = 对终点跑一次 IK → 一次 MoveJ → 驱动器自己做关节空间插值
 *          （等价于 second 的 plan_movel：整段一次 IK、不插补）。
 *          ⇒ 无段间停顿；代价是末端走弧（弓高 = 整段）且【不查过流】。
 * 已移除：step（逐段插补+逐段过流保护）、stream（按节拍连发）、sync（2026-09-23 移除）。
 *   三者写进命令都会被拒绝。要恢复见 git 历史（2026-09-28 的父提交）。 */
typedef enum {
    MOVL_MODE_SMOOTH = 3
} MovlMode;

/* 解析结果。⚠️ 新增命令务必把 out->type 放在【全部校验之后】再设，
 * 因为 main.c 只看 cmd->type 不看 cmd_parse 的返回值。 */
typedef struct {
    int      type;
    int      joint;
    double   angle_deg;
    double   speed_rpm;
    int      rel;
    double   zero_vals[6];
    int      num_joints;
    int      joints[6];
    double   angles[6];
    double   speeds[6];
    int      accel_ms[6];
    int      decel_ms[6];
    double   cartesian[6];
    int      keep_pose;
    int      movl_mode;
    double   param;
    char     raw[128];
} ParsedCmd;

/* 解析一行输入。返回 CMD_* 常量（同时写入 out->type）。
 * ⚠️ 非法参数只打警告，命令【照样执行】—— 见上面 out->type 的坑。 */
int cmd_parse(const char *line, ParsedCmd *out);

/* `help` / `?`：打印全部命令与用法。
 * 改动测量口径时必须同步改这里，否则会把人引向错误的优化方向
 * （曾写着已作废的 15.4ms 公式）。 */
void cmd_print_help(void);

#endif