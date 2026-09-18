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
        return CMD_UNKNOWN;
    }

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
        return CMD_UNKNOWN;
    }
    snprintf(cmd, sizeof(cmd), "%s", tok);

    if (strcmp(cmd, "home") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        out->type = CMD_HOME;
        if (j != NULL) {
            int joint = atoi(j);
            if (joint < 1 || joint > 6) {
                printf("[警告] 用法: home[:关节号]  例 home:1 单独回零1轴\n");
                return CMD_UNKNOWN;
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
                return CMD_UNKNOWN;
            }
            if (ntok != 9) {
                printf("[警告] 用法: MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC（6个角度+3个参数）\n");
                return CMD_UNKNOWN;
            }
            {
                double ang[6];
                double spd = 0.0, acc = 0.0, dec = 0.0;
                int i;
                for (i = 0; i < 6; i++) {
                    if (!parse_full_number(toks[i], &ang[i])) {
                        printf("[警告] MoveJ 角度须为纯数字：%s\n", toks[i]);
                        return CMD_UNKNOWN;
                    }
                }
                if (!parse_full_number(toks[6], &spd) ||
                    !parse_full_number(toks[7], &acc) ||
                    !parse_full_number(toks[8], &dec)) {
                    printf("[警告] MoveJ SPD/ACC/DEC 须为纯数字：%s,%s,%s\n",
                           toks[6], toks[7], toks[8]);
                    return CMD_UNKNOWN;
                }
                if (spd <= 0.0) {
                    printf("[警告] MoveJ 速度须大于 0 rpm\n");
                    return CMD_UNKNOWN;
                }
                if (acc <= 0.0 || dec <= 0.0) {
                    printf("[警告] MoveJ 加减速时间须大于 0 ms\n");
                    return CMD_UNKNOWN;
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
                return CMD_UNKNOWN;
            }
            if (!parse_full_number(j, &jnum) || !parse_full_number(a, &ang)) {
                printf("[警告] MoveJ 关节号/角度须为纯数字\n");
                return CMD_UNKNOWN;
            }
            joint = (int)jnum;
            if (joint < 1 || joint > 6) {
                printf("[警告] 关节号须在 1..6 之间\n");
                return CMD_UNKNOWN;
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
                            return CMD_UNKNOWN;
                        }
                        if (num <= 0.0) {
                            printf("[警告] MoveJ 速度须大于 0 rpm\n");
                            return CMD_UNKNOWN;
                        }
                        out->speed_rpm = num;
                        speed_set = 1;
                    } else if (strcmp(seg[k], "r") == 0) {        /* 相对 */
                        if (mode_set) {
                            printf("[警告] MoveJ 模式段重复\n");
                            return CMD_UNKNOWN;
                        }
                        out->rel = 1;
                        mode_set = 1;
                    } else if (strcmp(seg[k], "a") == 0) {        /* 绝对 */
                        if (mode_set) {
                            printf("[警告] MoveJ 模式段重复\n");
                            return CMD_UNKNOWN;
                        }
                        out->rel = 0;
                        mode_set = 1;
                    } else {
                        printf("[警告] MoveJ 模式须为 r(相对) 或 a(绝对)：%s\n", seg[k]);
                        return CMD_UNKNOWN;
                    }
                }
            }
        }
    } else if (ci_strcmp(cmd, "MoveL") == 0) {
        /* 笛卡尔直线（对齐 ABB MoveL）：
         * MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,MODE]
         * MODE = sync(单发同步，默认) / step(逐段到位) / stream(周期刷新) */
        char *rest = save;
        char *ctx = NULL;
        char *tok = strtok_r(rest, ",", &ctx);
        double v[9];
        int n = 0, i;
        while (tok != NULL && n < 9) {
            if (!parse_full_number(tok, &v[n])) {
                /* 非数字段：只可能是第 10 段位置的 MODE 关键字（step/stream），
                 * 交给下方模式判定；若出现在前 6 段则由下方段数校验拦下 */
                break;
            }
            n++;
            tok = strtok_r(NULL, ",", &ctx);
        }
        if (n != 6 && n != 9) {
            printf("[警告] 用法: MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,sync|step|stream]\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_MOVEL;
        /* 默认 sync（单发同步）：逐段到位会在每段做一次完整的加减速并等待，
         * 段数一多就明显一卡一卡（实测 94mm@60rpm 分 95 段） */
        out->movl_mode = MOVL_MODE_SYNC;
        for (i = 0; i < 6; i++) out->cartesian[i] = v[i];
        if (n == 9) {
            if (v[6] <= 0.0) {
                printf("[警告] MoveL 速度须大于 0 rpm\n");
                return CMD_UNKNOWN;
            }
            if (v[7] <= 0.0 || v[8] <= 0.0) {
                printf("[警告] MoveL 加减速时间须大于 0 ms\n");
                return CMD_UNKNOWN;
            }
            out->speeds[0] = v[6];
            out->accel_ms[0] = (int)v[7];
            out->decel_ms[0] = (int)v[8];
        } else {
            out->speeds[0] =   60;  /* 默认速度 rpm */
            out->accel_ms[0] = 80;  /* 默认加速度 ms */
            out->decel_ms[0] = 90;  /* 默认减速度 ms */
        }
        if (tok != NULL) {                     /* 第 10 段：下发模式关键字 */
            if (strcmp(tok, "sync") == 0) {
                out->movl_mode = MOVL_MODE_SYNC;
            } else if (strcmp(tok, "step") == 0) {
                out->movl_mode = MOVL_MODE_STEP;
            } else if (strcmp(tok, "stream") == 0) {
                out->movl_mode = MOVL_MODE_STREAM;
            } else if (strcmp(tok, "smooth") == 0) {
                out->movl_mode = MOVL_MODE_SMOOTH;
            } else {
                printf("[警告] MoveL 模式须为 sync(按弓高预算分段，默认)、step(逐段到位)、"
                       "stream(周期刷新) 或 smooth(流畅优先，不分段)：%s\n", tok);
                return CMD_UNKNOWN;
            }
            tok = strtok_r(NULL, ",", &ctx);
            if (tok != NULL) {
                printf("[警告] MoveL 参数过多，用法: MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,sync|step|stream]\n");
                return CMD_UNKNOWN;
            }
        }
    } else if (strcmp(cmd, "disable") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        out->type = CMD_DISABLE;
        if (j != NULL) {
            int joint = atoi(j);
            if (joint < 1 || joint > 6) {
                printf("[警告] 用法: disable[:关节号]  例 disable:1 单独泄力1轴\n");
                return CMD_UNKNOWN;
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
                return CMD_UNKNOWN;
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
            return CMD_UNKNOWN;
        }
        {
            char *ctx2 = NULL;
            char *tok2 = strtok_r(arg, ",", &ctx2);
            int n = 0;
            while (tok2 != NULL && n < 6) {
                double v;
                if (!parse_full_number(tok2, &v)) {
                    printf("[警告] fk 第 %d 个关节角须为纯数字：%s\n", n + 1, tok2);
                    return CMD_UNKNOWN;
                }
                out->angles[n++] = v;
                tok2 = strtok_r(NULL, ",", &ctx2);
            }
            if (n != 6) {
                printf("[警告] 用法: fk:J1,J2,J3,J4,J5,J6（必须 6 个关节角，单位度）\n");
                return CMD_UNKNOWN;
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
            return CMD_UNKNOWN;
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
                return CMD_UNKNOWN;
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double d;
            if (!parse_full_number(b, &d) || d <= 0.0 || d > 10.0) {
                printf("[警告] curtest 摆幅须在 (0, 10] 度之间（摆太大会撞到东西）：%s\n", b);
                return CMD_UNKNOWN;
            }
            out->angle_deg = d;
        }
        if (c != NULL) {
            double r;
            if (!parse_full_number(c, &r) || r <= 0.0 || r > 300.0) {
                printf("[警告] curtest 转速须在 (0, 300] rpm 之间：%s\n", c);
                return CMD_UNKNOWN;
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
            return CMD_UNKNOWN;
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
                return CMD_UNKNOWN;
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double ma;
            if (!parse_full_number(b, &ma) || ma < 0.0 || ma > 20000.0) {
                printf("[警告] stall 阈值须在 [0, 20000] mA 之间（0=关闭该轴检测）：%s\n", b);
                return CMD_UNKNOWN;
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
                        return CMD_UNKNOWN;
                    }
                    if (v < -360.0 || v > 360.0) {
                        printf("[警告] zero_save 第 %d 个零点越界(须在 ±360°)：%.2f\n",
                               n + 1, v);
                        return CMD_UNKNOWN;
                    }
                    vals[n] = v;
                    n++;
                    tok = strtok_r(NULL, ",", &ctx);
                }
            }
            if (n != 6) {
                printf("[警告] 用法: zero_save:v1,v2,v3,v4,v5,v6（6 个电机角零点，逗号分隔）\n");
                return CMD_UNKNOWN;
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
    "  MoveL:X,Y,Z,Rx,Ry,Rz[,SPD,ACC,DEC][,MODE]   笛卡尔直线运动；\n"
    "                          MODE=sync 单发同步(默认) / step 逐段到位 / stream 周期刷新\n"
    "  disable               全部失能所有关节\n"
    "  disable:N             仅单独泄力(失能)关节 N\n"
    "  enable                恢复使能所有关节\n"
    "  enable:N              仅单独使能关节 N\n"
    "  motor                 启动/停止电机实时监控（1S/次循环显示）\n"
    "  getpos                读取当前关节角(度)、笛卡尔坐标(X,Y,Z,RPY)与法兰倾角\n"
    "  fk:J1,J2,J3,J4,J5,J6  离线正解预览：按 DH 表算出该组关节角对应的位姿、\n"
    "                        法兰倾角与臂形；不动臂不下发，用来和 getpos 对照\n"
    "  diag                  总线时延体检：把单事务拆成 flush/write/read 三段计时，\n"
    "  bcast                 广播帧验证：地址0写速度再逐轴读回，看几轴响应广播\n"
    "                        并对照\"只写不读\"，定位刷新率瓶颈；不动臂\n"
    "  curtest               电流实测（标定堵转阈值用）：静止采样六轴保持电流，不动臂\n"
    "  curtest:N[:DEG[:RPM]] 关节N 走 +DEG 度再走回原位，全程高速采样该轴电流，\n"
    "                        输出 保持/运动 的最小·均值·最大(mA) 与建议阈值区间\n"
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