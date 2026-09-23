/*
 * cmd_parser.c —— 命令行交互命令解析与帮助输出
 * ------------------------------------------------------------
 * 所属模块：工具层（utils）
 * 对外接口：cmd_parse、cmd_print_help
 * 支持命令（命名对齐 ABB RAPID，大小写不敏感）：
 *           home、MoveJ（单/多关节）、MoveL（笛卡尔直线）、
 *           disable、enable、motor、getpos、zero、zero_save、help、exit
 */

#include "utils/cmd_parser.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* ci_strcmp：大小写不敏感字符串比较，返回 0 表示相等 */
static int ci_strcmp(const char *a, const char *b)
{
    int ca, cb;
    while (1) {
        ca = (unsigned char)*a;
        cb = (unsigned char)*b;
        if (isalpha(ca)) ca = tolower(ca);
        if (isalpha(cb)) cb = tolower(cb);
        if (ca != cb || ca == '\0') return ca - cb;
        a++; b++;
    }
}

/* parse_full_number：严格解析"纯数字"段（strtod 且 endptr 必须走到串尾），
 * 成功返回 1 并写出 *out，否则返回 0。
 * 用于 CLI 参数校验：拒绝 "2:60"、"abc"、"12x" 这类脏段——atof 会把 "2:60" 静默读成 2，
 * 造成速度被错位解析成 2rpm 之类的隐性错误，必须显式判错。 */
static int parse_full_number(const char *s, double *out)
{
    char *end = NULL;
    double v;

    if (s == NULL || *s == '\0') {
        return 0;
    }
    v = strtod(s, &end);
    if (end == s || *end != '\0') {
        return 0;
    }
    *out = v;
    return 1;
}

/* cmd_parse：解析一行命令文本到 ParsedCmd，返回命令类型 */
int cmd_parse(const char *line, ParsedCmd *out)
{
    char buf[128];
    char cmd[32] = {0};
    char *save = NULL;
    char *tok;
    size_t len;

    if (line == NULL || out == NULL) {
        return CMD_UNKNOWN;   /* out 可能为空指针，此处【不能】写 out->type */
    }

    /* 先清零：CMD_UNKNOWN = 0，故解析中途失败（return CMD_UNKNOWN）时
     * out->type 天然是"未知命令"，不会被前一次解析的残留骗过去。
     * 【为什么必须有】main.c 里 ParsedCmd 是未初始化的局部变量，而
     * cmd_dispatch 只看 cmd->type、不看 cmd_parse 的返回值 ⇒
     * 解析失败若不清 type，就会带着脏值（甚至上一次的合法命令）被执行。 */
    memset(out, 0, sizeof(*out));

    len = strlen(line);
    if (len == 0) {
        out->type = CMD_EMPTY;
        out->raw[0] = '\0';
        return CMD_EMPTY;
    }
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, line, len);
    buf[len] = '\0';

    /* 去首尾空白 */
    {
        char *p = buf;
        char *e;
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        e = p + strlen(p);
        while (e > p && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) e--;
        *e = '\0';
        if (p != buf) {
            memmove(buf, p, (size_t)(e - p) + 1);
        }
    }
    if (buf[0] == '\0') {
        out->type = CMD_EMPTY;
        out->raw[0] = '\0';
        return CMD_EMPTY;
    }

    memset(out, 0, sizeof(*out));
    snprintf(out->raw, sizeof(out->raw), "%s", buf);

    tok = strtok_r(buf, ":", &save);
    if (tok == NULL) {
        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
    }
    snprintf(cmd, sizeof(cmd), "%s", tok);

    if (strcmp(cmd, "home") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        out->type = CMD_HOME;
        if (j != NULL) {
            int joint = atoi(j);
            if (joint < 1 || joint > 6) {
                printf("[警告] 用法: home[:关节号]  例 home:1 单独回零1轴\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = joint;
        } else {
            out->joint = 0;
        }
    } else if (ci_strcmp(cmd, "MoveJ") == 0) {
        char *rest = save;
        if (rest != NULL && strchr(rest, ',') != NULL) {
            /* 多关节：MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC
             * 前6项为关节1~6的绝对角度(度)，末3项为速度/加速度/减速度(ms) */
            char *token_ctx = NULL;
            char *toks[16];
            int ntok = 0;
            char *tok = strtok_r(rest, ",", &token_ctx);
            while (tok != NULL && ntok < 16) {
                toks[ntok++] = tok;
                tok = strtok_r(NULL, ",", &token_ctx);
            }
            if (tok != NULL) {
                printf("[警告] MoveJ 参数过多，用法: MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            if (ntok != 9) {
                printf("[警告] 用法: MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC（6个角度+3个参数）\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            {
                double ang[6];
                double spd = 0.0, acc = 0.0, dec = 0.0;
                int i;
                for (i = 0; i < 6; i++) {
                    if (!parse_full_number(toks[i], &ang[i])) {
                        printf("[警告] MoveJ 角度须为纯数字：%s\n", toks[i]);
                        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                    }
                }
                if (!parse_full_number(toks[6], &spd) ||
                    !parse_full_number(toks[7], &acc) ||
                    !parse_full_number(toks[8], &dec)) {
                    printf("[警告] MoveJ SPD/ACC/DEC 须为纯数字：%s,%s,%s\n",
                           toks[6], toks[7], toks[8]);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                if (spd <= 0.0) {
                    printf("[警告] MoveJ 速度须大于 0 rpm\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                if (acc <= 0.0 || dec <= 0.0) {
                    printf("[警告] MoveJ 加减速时间须大于 0 ms\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->num_joints = 6;
                for (i = 0; i < 6; i++) {
                    out->joints[i] = i + 1;   /* J1~J6 按顺序 */
                    out->angles[i] = ang[i];
                }
                out->speeds[0] = spd;
                out->accel_ms[0] = (int)acc;
                out->decel_ms[0] = (int)dec;
                out->type = CMD_MOVEJ;
            }
        } else {
            /* 单关节：MoveJ:N:ANGLE[:SPD][:MODE]
             * SPD 为数字(转速 rpm)，MODE 为字母 r(相对)/a(绝对)，缺省绝对。
             * 靠类型区分段：数字段=速度、字母段=模式，避免空段(:r 缺速度)歧义。
             * 顺序：SPD 可前可后于 MODE，但只有这两个可选段。 */
            char *j = strtok_r(rest, ":", &save);
            char *a = strtok_r(NULL, ":", &save);
            char *f1 = strtok_r(NULL, ":", &save);
            char *f2 = strtok_r(NULL, ":", &save);
            char *extra = strtok_r(NULL, ":", &save);
            double jnum;
            double ang;
            int joint;
            if (j == NULL || a == NULL || extra != NULL) {
                printf("[警告] 用法: MoveJ:关节号:角度[:速度][:r|a]\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            if (!parse_full_number(j, &jnum) || !parse_full_number(a, &ang)) {
                printf("[警告] MoveJ 关节号/角度须为纯数字\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            joint = (int)jnum;
            if (joint < 1 || joint > 6) {
                printf("[警告] 关节号须在 1..6 之间\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->type = CMD_MOVEJ;
            out->joint = joint;
            out->angle_deg = ang;
            out->rel = 0;                 /* 默认绝对 */
            out->speed_rpm = 0.0;         /* 0 = 使用默认速度 */
            out->num_joints = 1;
            /* 解析 f1/f2 两段（顺序无关）：数字段=速度、字母段=模式(r/a)，
               速度/模式各只允许出现一次，重复或非法段均拒绝 */
            {
                char *seg[2];
                int k, speed_set = 0, mode_set = 0;
                seg[0] = f1; seg[1] = f2;
                for (k = 0; k < 2; k++) {
                    double num;
                    if (seg[k] == NULL) continue;
                    if (parse_full_number(seg[k], &num)) {       /* 速度段 */
                        if (speed_set) {
                            printf("[警告] MoveJ 速度段重复，应仅一个\n");
                            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                        }
                        if (num <= 0.0) {
                            printf("[警告] MoveJ 速度须大于 0 rpm\n");
                            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                        }
                        out->speed_rpm = num;
                        speed_set = 1;
                    } else if (strcmp(seg[k], "r") == 0) {        /* 相对 */
                        if (mode_set) {
                            printf("[警告] MoveJ 模式段重复\n");
                            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                        }
                        out->rel = 1;
                        mode_set = 1;
                    } else if (strcmp(seg[k], "a") == 0) {        /* 绝对 */
                        if (mode_set) {
                            printf("[警告] MoveJ 模式段重复\n");
                            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                        }
                        out->rel = 0;
                        mode_set = 1;
                    } else {
                        printf("[警告] MoveJ 模式须为 r(相对) 或 a(绝对)：%s\n", seg[k]);
                        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                    }
                }
            }
        }
    } else if (ci_strcmp(cmd, "MoveL") == 0) {
        /* 笛卡尔直线（对齐 ABB MoveL）：
         *   MoveL:X,Y,Z                        姿态【保持当前不变】，速度用默认
         *   MoveL:X,Y,Z,SPD,ACC,DEC,keep       姿态保持 + 自定义速度
         *   MoveL:X,Y,Z,Rx,Ry,Rz               完整写法（姿态角要抄 getpos 打印的原值）
         *   MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC
         *   后两种再可加 ,sync|step|stream|smooth
         *
         * 【为什么加"省略姿态" —— 2026-09-19 真机事故】
         * Rx,Ry,Rz 抄错是"画斜线"的头号根因：抄成【别的姿态】下的值
         * （home 姿态 getpos 打印 Ry=89.99，绘图位姿实际是 0）⇒ SLERP 把姿态
         * 从起点一路拧到目标 ⇒ 法兰下方的笔尖绕法兰摆。实测 80mm 线：
         * 姿态抄对 笔尖偏离 0.0000mm；抄成 115,90,115 ⇒ 偏离 7.71mm，
         * 而法兰直线度始终是 0.0000mm（只看法兰坐标永远发现不了）。
         * 大多数人要的其实就是"平移过去、姿态别动" —— 那就别让他抄这三个数。
         *
         * 【歧义与取舍】6 个数字段有两种解释：X,Y,Z,Rx,Ry,Rz 或 X,Y,Z,SPD,ACC,DEC。
         * 靠 keep 关键字区分：带 keep ⇒ 后三个是速度参数且姿态保持；
         * 不带 keep ⇒ 后三个是姿态角（与旧版完全兼容，老命令不受影响）。
         * 3 个数字段无歧义，直接判为"姿态保持"。 */
        char *rest = save;
        char *ctx = NULL;
        char *tok = strtok_r(rest, ",", &ctx);
        double v[9];
        int n = 0, i;
        int has_keep = 0;
        while (tok != NULL && n < 9) {
            if (!parse_full_number(tok, &v[n])) {
                /* 非数字段：只能是 keep 或 MODE 关键字，交给下方判定 */
                break;
            }
            n++;
            tok = strtok_r(NULL, ",", &ctx);
        }
        if (tok != NULL && strcmp(tok, "keep") == 0) has_keep = 1;

        if (n != 3 && n != 6 && n != 9) {
            printf("[警告] 用法: MoveL:X,Y,Z[,Rx,Ry,Rz][,SPD,ACC,DEC][,keep][,sync|step|stream|smooth]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_MOVEL;
        /* 【2026-09-23 用户拍板：默认【不分段】】
         * 原默认是 sync（按弓高预算分段、逐段等到位）。实测坐实：
         * 每段末速度归零 + 每段约 0.33~0.61s 的段间停顿 ⇒ 3 段就是明显"卡三下"，
         * 用户原话"分航点就很诡异的，跑的一卡一卡的""市面上的机械臂都不会这样"。
         * ⇒ 流畅优先于弓高：默认整段一次下发，交给驱动器自己做梯形规划，
         *   零段间停顿（这也是 second/ 那一套的做法，用户对它的手感是满意的）。
         * 代价：走关节空间直线，偏离笛卡尔直线 = 整段弓高，程序会如实打印
         *   （弓高 ≈ 0.001×段长²：50mm→2.1mm，100mm→10mm）。
         * 需要直线精度时【显式加 ,sync】，此时才按 ini [movel] bow_mm 分段。
         *
         * ⚠️ 安全代价：不分段 ⇒ 运动全程只有"起点/终点"一个分段点 ⇒
         *    【过流（碰撞）保护在本次运动中不会被执行】（打印会写明
         *    "本段不检查（整段一次下发，无分段点）"）。要保护就用 ,sync。 */
        out->movl_mode = MOVL_MODE_SMOOTH;
        out->keep_pose = 0;
        for (i = 0; i < 3; i++) out->cartesian[i] = v[i];   /* X,Y,Z 恒为前三个 */

        if (n == 3) {
            /* 只给位置 ⇒ 姿态保持当前不变（不要求用户抄 Rx,Ry,Rz） */
            out->keep_pose = 1;
            out->speeds[0]   =  60;  /* 默认速度 rpm */
            out->accel_ms[0] =  80;  /* 默认加速度 ms */
            out->decel_ms[0] =  90;  /* 默认减速度 ms */
        } else if (n == 6 && has_keep) {
            out->keep_pose = 1;
            if (v[3] <= 0.0) {
                printf("[警告] MoveL 速度须大于 0 rpm\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            if (v[4] <= 0.0 || v[5] <= 0.0) {
                printf("[警告] MoveL 加减速时间须大于 0 ms\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speeds[0]   = v[3];
            out->accel_ms[0] = (int)v[4];
            out->decel_ms[0] = (int)v[5];
        } else if (n == 6) {
            /* X,Y,Z,Rx,Ry,Rz —— 旧行为，向后兼容 */
            for (i = 0; i < 3; i++) out->cartesian[3 + i] = v[3 + i];
            out->speeds[0]   =  60;
            out->accel_ms[0] =  80;
            out->decel_ms[0] =  90;
        } else {                                    /* n == 9 */
            if (has_keep)
                printf("[提示] 已给出 Rx,Ry,Rz，keep 被忽略（姿态以你给的值为准）\n");
            for (i = 0; i < 3; i++) out->cartesian[3 + i] = v[3 + i];
            if (v[6] <= 0.0) {
                printf("[警告] MoveL 速度须大于 0 rpm\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            if (v[7] <= 0.0 || v[8] <= 0.0) {
                printf("[警告] MoveL 加减速时间须大于 0 ms\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speeds[0]   = v[6];
            out->accel_ms[0] = (int)v[7];
            out->decel_ms[0] = (int)v[8];
        }
        if (has_keep) tok = strtok_r(NULL, ",", &ctx);   /* 消费 keep */
        if (tok != NULL) {                               /* 模式关键字 */
            if (strcmp(tok, "sync") == 0) {
                out->movl_mode = MOVL_MODE_SYNC;
            } else if (strcmp(tok, "step") == 0) {
                out->movl_mode = MOVL_MODE_STEP;
            } else if (strcmp(tok, "stream") == 0) {
                out->movl_mode = MOVL_MODE_STREAM;
            } else if (strcmp(tok, "smooth") == 0) {
                out->movl_mode = MOVL_MODE_SMOOTH;
            } else {
                printf("[警告] MoveL 模式须为 sync(按弓高预算分段)、step(逐段到位)、"
                       "stream(周期刷新) 或 smooth(流畅优先，不分段)：%s\n", tok);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            tok = strtok_r(NULL, ",", &ctx);
            if (tok != NULL) {
                printf("[警告] MoveL 参数过多，用法: MoveL:X,Y,Z[,Rx,Ry,Rz][,SPD,ACC,DEC][,keep][,MODE]\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
        }
    } else if (strcmp(cmd, "disable") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        out->type = CMD_DISABLE;
        if (j != NULL) {
            int joint = atoi(j);
            if (joint < 1 || joint > 6) {
                printf("[警告] 用法: disable[:关节号]  例 disable:1 单独泄力1轴\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = joint;
        } else {
            out->joint = 0;  /* 0=全部失能 */
        }
    } else if (strcmp(cmd, "enable") == 0) {
        /* enable[:关节号]：带关节号时只使能该轴。
         * 关节号非法时必须判错返回 CMD_UNKNOWN，否则 "enable:9"（对应电机不存在）
         * 会被静默误报为合法的 CMD_ENABLE 并触发全轴使能。 */
        char *j = strtok_r(NULL, ":", &save);
        out->type = CMD_ENABLE;
        if (j != NULL) {
            double jnum = 0.0;
            if (!parse_full_number(j, &jnum) || jnum < 1.0 || jnum > 6.0 ||
                jnum != (double)(int)jnum) {
                printf("[警告] 用法: enable[:关节号]  例 enable:1 单独使能1轴\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jnum;
        } else {
            out->joint = 0;  /* 0=全部使能 */
        }
    } else if (strcmp(cmd, "motor") == 0) {
        out->type = CMD_MOTOR;
    } else if (strcmp(cmd, "getpos") == 0) {
        out->type = CMD_GETPOS;
    } else if (strcmp(cmd, "fk") == 0) {
        /* fk:J1,J2,J3,J4,J5,J6 —— 离线正解预览：不动臂、不下发、不读硬件。
         * 用来把"模型说应该是什么样"和 getpos 读到的真机值直接对照。 */
        char *arg = strtok_r(NULL, ":", &save);
        if (arg == NULL) {
            printf("[警告] 用法: fk:J1,J2,J3,J4,J5,J6   例 fk:0,0,90,0,90,0\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        {
            char *ctx2 = NULL;
            char *tok2 = strtok_r(arg, ",", &ctx2);
            int n = 0;
            while (tok2 != NULL && n < 6) {
                double v;
                if (!parse_full_number(tok2, &v)) {
                    printf("[警告] fk 第 %d 个关节角须为纯数字：%s\n", n + 1, tok2);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->angles[n++] = v;
                tok2 = strtok_r(NULL, ",", &ctx2);
            }
            if (n != 6) {
                printf("[警告] 用法: fk:J1,J2,J3,J4,J5,J6（必须 6 个关节角，单位度）\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->num_joints = 6;
            out->type = CMD_FK;
        }
    } else if (strcmp(cmd, "diag") == 0) {
        /* diag：总线体检。不动臂，只是拿"读位置"这类只读事务打点，
         * 把 flush / write / read 三段耗时分开，找出单事务 26ms 的真正去向。 */
        out->type = CMD_DIAG;
    } else if (strcmp(cmd, "bcast") == 0) {
        /* bcast：广播帧验证。用地址 0 写一个无害寄存器（运行速度），
         * 看几轴跟着变，以此判断 LEESN 是否真的执行广播帧。 */
        out->type = CMD_BCAST;
    } else if (strcmp(cmd, "nrtest") == 0) {
        /* nrtest：noread（只写不等响应）帧完整性测试。
         * 反复给六轴下发【当前位置】（原地不动，零风险），
         * 测出"连发不撞车"所需的帧间延迟，以及每轮真实耗时。 */
        out->type = CMD_NRTEST;
    } else if (strcmp(cmd, "curtest") == 0) {
        /* curtest：堵转阈值标定用的电流实测工具（读 0x001A）。
         *   curtest                 静止采样六轴"使能保持电流"，一动不动
         *   curtest:N[:DEG[:RPM]]   关节 N 走 +DEG 再走回原位，全程高速采样该轴电流
         * DEG 默认 2°，上限 10°（摆太大会撞到东西）；RPM 默认 30，上限 300。 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: curtest  或  curtest:关节号[:摆幅度[:转速rpm]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_CURTEST;
        out->joint = 0;          /* 0 = 静止六轴 */
        out->angle_deg = 2.0;
        out->speed_rpm = 30.0;
        if (a == NULL) return CMD_CURTEST;    /* 无参数：静止采样 */
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] curtest 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double d;
            if (!parse_full_number(b, &d) || d <= 0.0 || d > 10.0) {
                printf("[警告] curtest 摆幅须在 (0, 10] 度之间（摆太大会撞到东西）：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->angle_deg = d;
        }
        if (c != NULL) {
            double r;
            if (!parse_full_number(c, &r) || r <= 0.0 || r > 300.0) {
                printf("[警告] curtest 转速须在 (0, 300] rpm 之间：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speed_rpm = r;
        }
    } else if (strcmp(cmd, "busrate") == 0) {
        /* busrate：总线极限速率实测。busrate 或 busrate:N（N=1..6，默认 1）。
         * 全程只写 0x00D8(运行速度) 并写回读到的原值 ⇒ 电机不动、速度不变。
         * 【为什么不用"写当前位置"】位置若在总线出错时读成 0，下发就是甩臂；
         * 速度寄存器写错也无害，是更安全的探针。 */
        char *a = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: busrate  或  busrate:关节号(1..6)\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_BUSRATE;
        out->joint = 1;
        if (a != NULL) {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] busrate 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
    } else if (strcmp(cmd, "accel") == 0) {
        /* accel：驱动器加减速时间读写（0x0098/0x0099，出厂 120/120 ms）。
         *   accel            只读显示六轴（含启动/停止速度 0x0096/0x0097）
         *   accel:ACC        只改加速时间
         *   accel:ACC,DEC    改加速与减速时间
         * ACC/DEC 单位 ms，值域 0~65535。
         * 【为什么重要】手册第 42/43 节：加速时间 = "从启动速度到目标速度需要的时间"，
         * 减速时间 = "从目标速度到停止速度需要的时间"。驱动器每段末都要走完这一对，
         * 出厂 120+120=240ms ⇒ 段间停顿里固定吃掉这么多。这是"顿"里唯一
         * 不用改硬件就能砍的部分。
         * ⚠️ 调太小会丢步/过流（步进扭矩有限），从 40 起试，别一上来就 0。 */
        char *a = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: accel  或  accel:加速ms  或  accel:加速ms,减速ms\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->angle_deg = -1.0;   /* 复用：-1 = 不设置加速时间 */
        out->speed_rpm = -1.0;   /* 复用：-1 = 不设置减速时间 */
        if (a != NULL) {
            char *comma = strchr(a, ',');
            double v;
            if (comma != NULL) {
                *comma = '\0';
                if (!parse_full_number(a, &v) || v < 0.0 || v > 65535.0) {
                    printf("[警告] accel 加速时间须为 0~65535 ms：%s\n", a);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->angle_deg = v;
                if (!parse_full_number(comma + 1, &v) || v < 0.0 || v > 65535.0) {
                    printf("[警告] accel 减速时间须为 0~65535 ms：%s\n", comma + 1);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->speed_rpm = v;
            } else {
                if (!parse_full_number(a, &v) || v < 0.0 || v > 65535.0) {
                    printf("[警告] accel 加速时间须为 0~65535 ms：%s\n", a);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->angle_deg = v;
            }
        }
        out->type = CMD_ACCEL;
    } else if (strcmp(cmd, "drvbaud") == 0) {
        /* drvbaud：驱动器波特率寄存器 0x0009 读写 + 未知档位试探。
         *   drvbaud            只读（六轴 0x0009 解码 + 固件版本）
         *   drvbaud:CODE       广播写档位码（写完立即失联，不切 PC 侧）
         *   drvbaud:CODE:BAUD  广播写后把 PC 侧切到 BAUD 再回读验证
         *   drvbaud:save       广播写 0x00DC=1 固化到 Flash
         * 【为什么需要它】手册 §6 的档位表是 1~15（12=115200 … 15=921600），
         * 没有 256000；但手册第二章"速率-距离"表里列了 256000（250m）。
         * 手册不等于实际固件 ⇒ 只能实测。未知码可以试：不 save 的话
         * 驱动器断电重启即回 115200，试探是可恢复的。 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: drvbaud  或  drvbaud:档位码[:PC波特率]  或  drvbaud:save\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->joint = 0;     /* 0 = 只读 */
        out->param = 0.0;   /* 0 = 不改 PC 侧波特率 */
        out->raw[0] = '\0';
        if (a != NULL) {
            if (strcmp(a, "save") == 0) {
                if (b != NULL) {
                    printf("[警告] drvbaud:save 不接受第二个参数\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                snprintf(out->raw, sizeof(out->raw), "save");
            } else {
                double v;
                if (!parse_full_number(a, &v) || v < 0.0 || v > 255.0 ||
                    v != (double)(int)v) {
                    printf("[警告] drvbaud 档位码须为 0~255 的整数：%s\n", a);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->joint = (int)v;
                if (b != NULL) {
                    double bv;
                    if (!parse_full_number(b, &bv) || bv < 0.0 || bv > 4000000.0) {
                        printf("[警告] drvbaud PC 侧波特率须为 0~4000000：%s\n", b);
                        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                    }
                    out->param = bv;
                }
            }
        }
        out->type = CMD_DRVBAUD;
    } else if (strcmp(cmd, "alarm") == 0) {
        /* alarm：显示六轴驱动器报警（0x00A3）/ alarm:clear 清除（0x00A4=0）。
         * 手册第 52 节：0x00A3 每 4 位一个报警代码，低 4 位=当前，高 12 位=最近三次历史。
         * 报警置位 ⇒ 状态字 bit21=1 ⇒ 到位判据里"已退出 RUN"永不成立 ⇒ 运动超时。 */
        char *a = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: alarm  或  alarm:clear\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->rel = 0;   /* 复用：0=只显示，1=清除 */
        if (a != NULL) {
            if (strcmp(a, "clear") != 0) {
                printf("[警告] alarm 只接受 clear：alarm  或  alarm:clear\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->rel = 1;
        }
        out->type = CMD_ALARM;
    } else if (strcmp(cmd, "looptest") == 0) {
        /* looptest：RS485 转换器【纯工具】极限测试（脱离电机）。
         *   looptest                 30 次，波特率不变，单模块 A/B 回环
         *   looptest:N               N 次（1..1000）
         *   looptest:N:BAUD          临时切到 BAUD 测，测完自动改回原值
         *   looptest:N:BAUD:COM5     双模块模式：主口发、辅口(COM5)收
         * 【为什么要单独一条命令】接电机时单事务 15.4ms 里 14.3ms 是"等响应"，
         * 但那 14.3ms 到底是【转换器+串口栈】慢还是【驱动器处理+回帧】慢，
         * 并线接法下两者混在一起分不开。把中间那一段换成一段导线（回环），
         * 没有从站参与 ⇒ 测到的就是工具的地板延迟。
         * 复用字段：joint=N、param=BAUD、rel=1 表示给了辅口、raw=辅口名。
         * ⚠️ BAUD 只改 PC 侧，接电机时切波特率会立刻失联 —— 只在脱离电机时用。 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: looptest[:次数[:波特率[:辅口名]]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->joint = 30;
        out->param = 0.0;
        out->rel = 0;
        if (a != NULL) {
            double n;
            if (!parse_full_number(a, &n) || n < 1.0 || n > 1000.0 || n != (double)(int)n) {
                printf("[警告] looptest 次数须为 1..1000 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)n;
        }
        if (b != NULL) {
            double bd;
            if (!parse_full_number(b, &bd) || bd < 0.0 || bd > 4000000.0) {
                printf("[警告] looptest 波特率须为 0~4000000：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->param = bd;
        }
        if (c != NULL) {
            /* 辅口名：ParsedCmd 没有字符串字段，这里占用 raw（looptest 不读原始行）。
             * 给出即启用双模块模式。只做长度/字符粗筛，真伪交给 serial_open 判定。 */
            size_t cl = strlen(c);
            if (cl == 0 || cl >= 32) {
                printf("[警告] looptest 辅口名不合法：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            snprintf(out->raw, sizeof(out->raw), "%s", c);
            out->rel = 1;
        }
        out->type = CMD_LOOPTEST;
    } else if (strcmp(cmd, "tabtest") == 0) {
        /* tabtest：驱动器【表格数据】最小验证实验（手册第 54 节）。
         *   tabtest:N[:SEG_DEG[:RPM]]
         *   往关节 N 编程区写 3 段【相对位移】表，一条 0x00DD 让驱动器自己连续执行，
         *   全程高频读 0x00D6 实时速度并打印 ⇒ 看段间速度掉不掉 0。
         *   不掉 0 ⇒ 表格模式可做到"又直又顺"；掉 0 ⇒ 表格只省 PC 开销、救不了卡顿。
         * SEG_DEG = 每段的机械角(度)，默认 5，上限 30；RPM 默认 60。 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: tabtest:关节号[:每段角度[:转速rpm]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_TABTEST;
        out->joint = 1;
        out->angle_deg = 5.0;
        out->speed_rpm = 60.0;
        if (a == NULL) {
            printf("[警告] 用法: tabtest:关节号[:每段角度[:转速rpm]]，例如 tabtest:1:5:60\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] tabtest 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double d;
            if (!parse_full_number(b, &d) || d <= 0.0 || d > 30.0) {
                printf("[警告] tabtest 每段角度须在 (0, 30] 度之间：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->angle_deg = d;
        }
        if (c != NULL) {
            double r;
            if (!parse_full_number(c, &r) || r <= 0.0 || r > 300.0) {
                printf("[警告] tabtest 转速须在 (0, 300] rpm 之间：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speed_rpm = r;
        }
    } else if (strcmp(cmd, "stall") == 0) {
        /* stall：逐轴堵转电流阈值。
         *   stall          显示六轴阈值 + 各轴最新电流快照（不下发、不动臂）
         *   stall:N        显示关节 N 的阈值与电流
         *   stall:N:MA     设置关节 N 的阈值为 MA mA（0 = 关闭该轴检测）
         * 运行时改只影响本次运行；要持久化请写 ini [stall] j1..j6。 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: stall  或  stall:关节号  或  stall:关节号:阈值mA\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_STALL;
        out->joint = 0;
        out->param = -1.0;
        if (a == NULL) return CMD_STALL;
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] stall 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double ma;
            if (!parse_full_number(b, &ma) || ma < 0.0 || ma > 20000.0) {
                printf("[警告] stall 阈值须在 [0, 20000] mA 之间（0=关闭该轴检测）：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->param = ma;
        }
    } else if (strcmp(cmd, "poseok") == 0) {
        /* poseok：人工解除"位姿不可信"闸门。
         * 只在确认"越软限位是误判"时使用（例如某轴本来就该停在限位附近）。
         * 若零点真的丢了，解锁后所有位姿与运动都是错的。 */
        out->type = CMD_POSEOK;
    } else if (strncmp(cmd, "zero", 4) == 0) {
        if (strstr(out->raw, "save") != NULL) {
            /* zero_save:v1,v2,v3,v4,v5,v6：直接保存指定的零点值。
             * 严格解析 + 角度范围校验：拒绝 "abc"/"12x" 被 atof 静默读成 0，
             * 避免污染零点标定并持久化到 ini。 */
            char *valstr = strchr(out->raw, ':');
            double vals[6];
            int n = 0;
            char *ctx = NULL;
            if (valstr != NULL) {
                valstr++; /* 跳过 ':' */
                char *tok = strtok_r(valstr, ",", &ctx);
                while (tok != NULL && n < 6) {
                    double v;
                    if (!parse_full_number(tok, &v)) {
                        printf("[警告] zero_save 第 %d 个零点须为纯数字：%s\n", n + 1, tok);
                        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                    }
                    if (v < -360.0 || v > 360.0) {
                        printf("[警告] zero_save 第 %d 个零点越界(须在 ±360°)：%.2f\n",
                               n + 1, v);
                        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                    }
                    vals[n] = v;
                    n++;
                    tok = strtok_r(NULL, ",", &ctx);
                }
            }
            if (n != 6) {
                printf("[警告] 用法: zero_save:v1,v2,v3,v4,v5,v6（6 个电机角零点，逗号分隔）\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->type = CMD_ZERO_SAVE;
            for (int i = 0; i < 6; i++) out->zero_vals[i] = vals[i];
        } else {
            out->type = CMD_ZERO;
            out->joint = 0;
        }
    } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        out->type = CMD_HELP;
    } else if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "quit") == 0) {
        out->type = CMD_EXIT;
    } else {
        out->type = CMD_UNKNOWN;
    }
    return out->type;
}

/* ------------------------------------------------------------------ */
/* 命令帮助文本                                                       */
/* ------------------------------------------------------------------ */

static const char HELP_TEXT[] =
    "可用命令（命名对齐 ABB RAPID，大小写不敏感）:\n"
    "  home                  回零（全轴）\n"
    "  home:N                仅单独回零关节 N，堵转后自动到该轴配置角\n"
    "  MoveJ:N:ANGLE[:SPD][:r|a]   单关节关节空间运动：轴N 至角度ANGLE(度)，\n"
    "                          速度SPEED(rpm)，末段 r=相对当前位置 / a=绝对(默认)\n"
    "  MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC   多关节同步关节空间运动\n"
    "  MoveL:X,Y,Z[,MODE]    笛卡尔直线运动，【姿态保持当前不变】（推荐）；\n"
    "                          Rx,Ry,Rz 抄错会让笔尖画斜线(实测偏 7.71mm 而法兰\n"
    "                          0.0000mm)，所以能不抄就不抄\n"
    "  MoveL:X,Y,Z,SPD,ACC,DEC,keep[,MODE]   姿态保持 + 自定义速度\n"
    "  MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,MODE]   完整写法：姿态角须抄 getpos\n"
    "                          打印的原值(含负号、含 -0.00)\n"
    "                          MODE 缺省=【不分段】(流畅优先，零段间停顿，2026-09-23 改)\n"
    "                          sync 按弓高预算分段(直但有停顿) / step 逐段到位 /\n"
    "                          stream 周期刷新 / smooth 同缺省\n"
    "                          ⚠️ 不分段时运动全程【不查过流(碰撞)】；要保护显式加 ,sync\n"
    "  disable               全部失能所有关节\n"
    "  disable:N             仅单独泄力(失能)关节 N\n"
    "  enable                恢复使能所有关节\n"
    "  enable:N              仅单独使能关节 N\n"
    "  motor                 启动/停止电机实时监控（1S/次循环显示）\n"
    "  getpos                读取当前关节角(度)、笛卡尔坐标(X,Y,Z,RPY)与法兰倾角\n"
    "  fk:J1,J2,J3,J4,J5,J6  离线正解预览：按 DH 表算出该组关节角对应的位姿、\n"
    "                        法兰倾角与臂形；不动臂不下发，用来和 getpos 对照\n"
    "  diag                  总线时延体检：把单事务拆成 flush/write/read 三段计时，\n"
    "                        并对照\"只写不读\"，定位刷新率瓶颈；不动臂\n"
    "  busrate[:N]           总线极限速率实测（回答\"这根485最高多少Hz\"）：分档给出\n"
    "                        单轴 读/写+等响应/只写不等响应、六轴一轮(并线) 的 ms 与 Hz，\n"
    "                        并给出\"拆成6根独立总线\"的等效更新率；不动臂（探针写回原速度）\n"
    "  accel[:ACC[,DEC]]     驱动器加减速时间读写（0x0098 加速 / 0x0099 减速，出厂120/120 ms）\n"
    "                        无参数=只读显示；accel:40 或 accel:40,40 = 六轴写入并读回验证。\n"
    "                        只写 RAM（不发 0x00DC），驱动器断电即恢复，可反复试。\n"
    "                        ⚠️ 调太小会丢步/过流，从 40 起试；这是\"顿\"里唯一免改硬件的部分\n"
    "  alarm                 查看六轴驱动器报警（0x00A3：当前 + 最近三次历史）\n"
    "  alarm:clear           清除六轴报警（0x00A4=0）；⚠️ 先排除原因，否则立刻复现\n"
    "  drvbaud               驱动器波特率寄存器 0x0009：只读六轴档位码/波特率/校验/停止位+固件版本\n"
    "  drvbaud:CODE          广播写 0x0009=CODE（写完驱动器立即换速率 ⇒ 当场失联，不切PC侧）\n"
    "  drvbaud:CODE:BAUD     广播写 → PC侧切 BAUD → 回读验证（推荐，一条命令走完试探）\n"
    "  drvbaud:save          广播写 0x00DC=1 固化到Flash（不 save 则断电回 115200）\n"
    "                        手册档位 1~15：12=115200 13=230400 14=460800 15=921600；\n"
    "                        未知码（如16）也可以试：不 save 就断电即回 115200，可恢复\n"
    "  looptest[:N[:BAUD[:AUX]]]  RS485转换器【纯工具】极限测试（脱离电机）：\n"
    "                        ⚠️ 接线：A、B【悬空】，千万别把 A 和 B 接在一起 ——\n"
    "                           A/B 是差分对，短接后差分恒为 0，一个字节都收不到\n"
    "                        ① 单模块：A、B 空着，转换器自己就能听见自己\n"
    "                        ② 双模块：再插一个转换器，A-A/B-B 对接，主口发、辅口收\n"
    "                           looptest:30:0:COM5   ← AUX 写辅口名即启用②\n"
    "                        N=重复次数(默认30)，BAUD=临时切PC侧波特率(省略=不改，测完自动改回)\n"
    "                        判读：接电机单事务 15.4ms − 回环RTT = 驱动器侧耗时\n"
    "                        ⚠️ 必须脱离电机；接电机时切BAUD会立刻失联\n"
    "  bcast                 广播帧验证：地址0写速度再逐轴读回，看几轴响应广播\n"
    "  curtest               电流实测（标定堵转阈值用）：静止采样六轴保持电流，不动臂\n"
    "  curtest:N[:DEG[:RPM]] 关节N 走 +DEG 度再走回原位，全程高速采样该轴电流，\n"
    "                        输出 保持/运动 的最小·均值·最大(mA) 与建议阈值区间\n"
    "  tabtest:N[:DEG[:RPM]] 驱动器【表格数据】验证（手册第54节）：往关节N 写 3 段\n"
    "                        相对位移表，让驱动器自己连续执行，全程高频读 0x00D6\n"
    "                        实时速度 ⇒ 看段间速度掉不掉 0（决定\"又直又顺\"能否成立）\n"
    "                        DEG=每段机械角(默认5,上限30)；轴会停在 +3×DEG 处，注意行程\n"
    "  stall                 显示六轴堵转阈值(mA)与各轴最新电流\n"
    "  stall:N:MA            运行时设置关节N的堵转阈值为 MA mA（0=关闭该轴）；\n"
    "                        只改本次运行，持久化请写 ini [stall] 的 j1..j6\n"
    "  poseok                人工解除「位姿不可信」闸门（零点丢失时运动命令会被锁住；\n"
    "                        确认是误判才用，否则所有位姿与运动都是错的）\n"
    "  zero                  显示当前零点与机械角\n"
    "  zero_save:v1,v2,v3,v4,v5,v6  保存指定的零点标定值\n"
    "  help                  帮助\n"
    "  exit                  退出\n";

void cmd_print_help(void)
{
    fputs(HELP_TEXT, stdout);
}