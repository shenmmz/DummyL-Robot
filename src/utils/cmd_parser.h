#ifndef CMD_PARSER_H
#define CMD_PARSER_H

/*
 * 命令行解析（CLI 交互循环使用）
 * 命令命名对齐 ABB RAPID（大小写不敏感）：
 *   home             回零（全轴）
 *   home:N           仅单独回零关节 N
 *   MoveJ:N:ANGLE[:SPEED][:r|a]   单关节关节空间运动
 *   MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC   多关节同步关节空间运动
 *   MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,MODE]   笛卡尔直线运动
 *   disable          全部失能所有关节
 *   disable:N        仅单独泄力(失能)关节 N
 *   enable           全部使能所有关节
 *   enable:N         仅单独使能关节 N
 *   motor            启动/停止电机实时监控（1S/次循环显示）
 *   getpos           读取当前关节角(度)与笛卡尔坐标(X,Y,Z,RPY)与法兰倾角
 *   fk:J1,J2,J3,J4,J5,J6   离线正解预览：按当前 DH 表算出末端位姿/法兰倾角/臂形，
 *                    不读硬件也不下发，用来和 getpos 的真机读数直接对照
 *   zero             显示当前零点与机械角
 *   zero_save:v1,v2,v3,v4,v5,v6   直接写入 6 个电机角零点（逗号分隔）并持久化到 ini
 *   help             帮助（? 同义）
 *   exit             退出（quit 同义）
 */

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
/* 12 号保留（原 CMD_MOSEL 已删除，对应 ABB 中无此语义） */
#define CMD_MOVEL     13   /* 笛卡尔直线，对应 ABB MoveL */
#define CMD_FK        14   /* 离线正解预览：fk:J1..J6，不下发、不动臂 */
#define CMD_DIAG      15   /* 总线时延体检：diag，测 flush/write/read 各占多少 ms */
#define CMD_BCAST     16   /* 广播帧验证：bcast，验证地址 0 是否被从站执行 */
#define CMD_NRTEST    17   /* noread 帧完整性测试：nrtest，测连发不撞车所需的帧间延迟 */
#define CMD_CURTEST   18   /* 电流实测：curtest（静止六轴）/ curtest:N[:DEG[:RPM]]（单轴摆动采样） */
#define CMD_STALL     19   /* 堵转阈值：stall 显示全部 / stall:N 单轴 / stall:N:MA 设置(mA) */
#define CMD_POSEOK    20   /* 位姿闸门解锁：poseok（仅在确认"越限位是误判"时使用） */
#define CMD_BUSRATE   22   /* 总线极限速率实测：busrate[:N] —— 回答"这根 485 最高能跑多少 Hz"
                            *   分别测【单从站】只写不等响应 / 写+等响应 / 读位置的频率，
                            *   以及六轴轮询一轮的耗时。用来判断"换低延迟转换器"和
                            *   "6 路独立总线"各能带来多少收益。所有写操作目标 = 当前位置，
                            *   电机原地不动，只测总线。 */
#define CMD_TABTEST   21   /* 驱动器【表格数据】验证：tabtest:N[:SEG_DEG[:RPM]]
                            *   往关节 N 的编程区写 3 段相对位移表并让驱动器自己连续执行，
                            *   全程高频读 0x00D6 实时速度 ⇒ 看段间速度掉不掉 0。
                            *   这是"又直又顺"能否成立的决定性实验（手册第 54 节）。 */
#define CMD_ACCEL     23   /* 驱动器加减速时间读写：accel[:ACC[,DEC]]
                            *   0x0098 加速 / 0x0099 减速（出厂 120/120 ms，值域 0~65535）。
                            *   驱动器每段末都要"减到停止速度 + 下段再加到目标速度"，
                            *   这段耗时 = ACC+DEC，是"顿"里唯一不用改硬件就能砍的部分。
                            *   只写 RAM（不发 0x00DC），断电即恢复出厂值，可反复试。 */
#define CMD_ALARM     24   /* 驱动器报警查看/清除：alarm / alarm:clear
                            *   读 0x00A3（低4位当前 + 高12位最近三次历史，每4位一个代码），
                            *   写 0x00A4=0 清除。报警置位会让状态字 bit21=1，
                            *   使"到位判据"里的"已退出 RUN"永不成立 ⇒ 运动超时/到位误判。 */
#define CMD_LOOPTEST  25   /* RS485 转换器【纯工具】极限测试：looptest[:N[:BAUD]]
                            *   A/B 接成回环、脱离电机，发出去的数据自己收回来 ⇒
                            *   测到的就是【转换器+Windows 串口栈】的地板延迟，没有从站参与。
                            *   接电机单事务 15.4ms − 回环 RTT = 驱动器侧耗时。
                            *   可选 BAUD 临时切 PC 侧波特率（0/省略=不改），测完自动改回。 */
#define CMD_DRVBAUD   26   /* 驱动器波特率寄存器 0x0009 读写 + 未知档位试探：
                            *   drvbaud               只读（六轴 0x0009 解码 + 固件版本）
                            *   drvbaud:CODE          广播写 0x0009=CODE（写完立即失联）
                            *   drvbaud:CODE:BAUD     广播写 → PC 侧切 BAUD → 回读验证（推荐）
                            *   drvbaud:save          广播写 0x00DC=1 固化（断电不丢）
                            *   手册 §6 档位 1~15（最大 921600）里没有 256000，
                            *   但手册第二章"速率-距离"表里列了 256000 ⇒ 实测为准。
                            *   ⚠️ 不 save 的话断电即回 115200，试探可恢复。 */

/* MoveL 下发模式：整段只发一次最平滑，但段内走关节插补会有弓高 */
typedef enum {
    MOVL_MODE_SYNC = 0,   /* 同步（默认）：按弓高预算自适应分段，逐航点等到位 ⇒ 直，但段间一停 */
    MOVL_MODE_STEP = 1,   /* 逐段到位：密集弦逼近真直线，段间有停顿 */
    MOVL_MODE_STREAM = 2, /* 周期刷新：段间不停顿，按节拍刷新目标 */
    /* 流畅优先：整段【一次】下发，交给驱动器自己做梯形规划 ⇒ 零段间停顿。
     * 代价是走关节空间直线、偏离笛卡尔直线（偏离量 = 整段弓高，会打出来）。
     * 这正是 Hg_Robot_Arm 的做法——它"不卡"的根本原因是从来不分段，
     * 只是它没把这个偏离量告诉用户而已。弓高 ∝ 长度²，短线段用它几乎看不出弯。 */
    MOVL_MODE_SMOOTH = 3
} MovlMode;

typedef struct {
    int      type;         /* CMD_* */
    int      joint;        /* 1..6（关节号），0=全部/未指定 */
    double   angle_deg;    /* 目标角度（度）：rel=0 为绝对，rel=1 为相对增量 */
    double   speed_rpm;    /* 速度（rpm），0 表示使用默认 */
    int      rel;          /* 单关节 MoveJ 模式：1=相对当前位置，0=绝对（默认） */
    double   zero_vals[6]; /* zero_save 写入的 6 个电机角零点 */
    int      num_joints;   /* 多关节 MoveJ 的关节数 */
    int      joints[6];    /* 多关节 MoveJ 的关节号 */
    double   angles[6];    /* 多关节 MoveJ 的目标角度 */
    double   speeds[6];    /* 多关节 MoveJ 的速度 */
    int      accel_ms[6];  /* 多关节 MoveJ 的加速度 ms */
    int      decel_ms[6];  /* 多关节 MoveJ 的减速度 ms */
    double   cartesian[6]; /* MoveL 目标位姿 X,Y,Z,Rx,Ry,Rz */
    /* keep_pose：1 = 姿态【保持当前不变】，只走位置。
     * 触发方式两种：① 只给 X,Y,Z 三个数；② 数字段后带 keep 关键字。
     * 【为什么要有它 —— 2026-09-19】Rx,Ry,Rz 抄错是"画斜线"的头号根因：
     * 抄成别的姿态的值 ⇒ SLERP 把姿态一路拧过去 ⇒ 法兰下方的笔尖绕法兰摆，
     * 实测 80mm 线笔尖偏离 7.71mm，而法兰直线度 0.0000mm（看不出来）。
     * 大多数人要的就是"平移过去、姿态别动"，那就别让他抄这三个数。 */
    int      keep_pose;
    int      movl_mode;    /* MoveL 下发模式，见 MovlMode（默认 MOVL_MODE_SYNC） */
    double   param;        /* 通用数值参数：stall 的阈值(mA)；-1 = 未指定（只显示） */
    char     raw[128];     /* 原始输入 */
} ParsedCmd;

/* 解析一行输入，成功返回 CMD_*，解析失败返回 CMD_UNKNOWN */
int cmd_parse(const char *line, ParsedCmd *out);

/* 打印 CLI 命令帮助文本 */
void cmd_print_help(void);

#endif /* CMD_PARSER_H */