#ifndef CMD_PARSER_H
#define CMD_PARSER_H


/* 命令类型码（ParsedCmd.type）：cmd_parser.c 解析时置位，commands.c 按此分发。
 * 编号为历史追加顺序，12 已删（stream 模式）不再回收；新命令往 32 起追加。 */
#define CMD_UNKNOWN   0     /* 未识别命令（只报警，不分发） */
#define CMD_HOME      1     /* home[:N]      回零（全轴/单关节） */
#define CMD_MOVEJ     2     /* MoveJ         单关节运动 / 多关节同步 */
#define CMD_DISABLE   3     /* disable[:N]   泄力失能 */
#define CMD_ENABLE    4     /* enable[:N]    使能 */
#define CMD_MOTOR     5     /* motor         电机实时监控开关 */
#define CMD_HELP      6     /* help[:topic]  分级帮助 */
#define CMD_EXIT      7     /* exit          退出 */
#define CMD_EMPTY     8     /* 空输入（含纯空白），主循环静默跳过 */
#define CMD_ZERO      9     /* zero          显示零点与机械角 */
#define CMD_GETPOS    10    /* getpos        读当前位姿 */
#define CMD_ZERO_SAVE 11    /* zero_save     保存零点标定值 */
#define CMD_MOVEL     13    /* movel         直线（默认 interp，,smooth 流畅） */
#define CMD_FK        14    /* fk            离线正解预览（不动臂） */
#define CMD_DIAG      15    /* diag          总线时延体检 */
#define CMD_BCAST     16    /* bcast         广播帧验证 */
#define CMD_NRTEST    17    /* nrtest        noread 帧完整性/安全间隔测试 */
#define CMD_CURTEST   18    /* curtest       电流实测（标定堵转阈值） */
#define CMD_STALL     19    /* stall         堵转阈值查看/设置 */
#define CMD_POSEOK    20    /* poseok        解除位姿不可信闸门 */
#define CMD_BUSRATE   22    /* busrate       485 极限速率实测 */
#define CMD_TABTEST   21    /* tabtest       表格执行 0x00DD 试验（21/22 为追加时乱序） */
#define CMD_ACCEL     23    /* accel         加减速时间读写（0x0098/99） */
#define CMD_ALARM     24    /* alarm         报警查看/清除 */
#define CMD_LOOPTEST  25    /* looptest      转换器极限（须脱离电机） */
#define CMD_DRVBAUD   26    /* drvbaud       波特率读/设/固化（⚠失联风险） */
#define CMD_PIPE      27    /* pipe          流水线批量读探针 */
#define CMD_QUEUETEST 28    /* queuetest     排队寄存器 0x00CE 试验 */
#define CMD_CHAINTEST 29    /* chain         补链排队试验（0x00CE 流水线） */
#define CMD_TRIGTEST  30    /* trigtest      表格重复触发试验（0x00DD 指针自增） */
#define CMD_PROGREAD  31    /* progread      编程区只读转储（反推指令格式） */

/* movel 模式：两种。
 * interp（默认）= 真正的笛卡尔逐点插补：按 [movel] step_mm 生成多个航点 → 逐点 IK
 *          → 逐航点下发并等到位（分段走）。末端贴着直线（弓高 ∝ 段长²、极小），
 *          且每段都走 movej_wait 的读回异常/超时急停保护。
 *          代价：航点间有加减速停顿（非零停顿），比 smooth 慢。
 * smooth   = 对终点跑一次 IK → 一次 MoveJ → 驱动器自己做关节空间插值（需显式加 ,smooth）。
 *          ⇒ 无段间停顿；代价是末端走弧（弓高 = 整段）且【不查过流】。
 * 已移除：stream（按节拍连发）、sync（2026-09-23 移除），写了会被拒绝。 */
typedef enum {
    MOVL_MODE_SMOOTH = 3,
    MOVL_MODE_INTERP = 4
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
    int      movl_mode;
    double   param;
    char     help_topic[16];
    char     raw[128];
} ParsedCmd;

/* 解析一行输入，返回 CMD_* 常量（同时写入 out->type）。 */
int cmd_parse(const char *line, ParsedCmd *out);

/* 分级帮助：topic 为空打印速查索引；为 motion/enable/state/diag/probe 打印
 * 该组详解；为 all 一次性全展开。 */
void cmd_print_help(const char *topic);

#endif