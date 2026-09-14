/*
 * cmd_parser.c —— 命令行交互命令解析与帮助输出
 * ------------------------------------------------------------
 * 所属模块：工具层（utils）
 * 对外接口：cmd_parse、cmd_print_help
 * 支持命令：home、movej（单/多关节）、movel、disable、enable、motor、getpos、
 *           zero、zero_save、help、exit
 */

#include "utils/cmd_parser.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* movej 多关节用法提示（失败路径统一引用，避免各处字面量漂移） */
#define MOVEJ_MULTI_USAGE "movej:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC"

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
    } else if (strcmp(cmd, "movej") == 0) {
        char *rest = save;  /* 剩余字符串：30,60,90,0,0,0,3000,150,200 */
        if (rest != NULL && strchr(rest, ',') != NULL) {
            /* 多关节：movej:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC
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
                printf("[警告] movej 参数过多，用法: movej:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC\n");
                return CMD_UNKNOWN;
            }
            if (ntok != 9) {
                printf("[警告] 用法: movej:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC（6个角度+3个参数）\n");
                return CMD_UNKNOWN;
            }
            {
                double ang[6];
                double spd = 0.0, acc = 0.0, dec = 0.0;
                int i;
                for (i = 0; i < 6; i++) {
                    if (!parse_full_number(toks[i], &ang[i])) {
                        printf("[警告] movej 角度须为纯数字：%s\n", toks[i]);
                        return CMD_UNKNOWN;
                    }
                }
                if (!parse_full_number(toks[6], &spd) ||
                    !parse_full_number(toks[7], &acc) ||
                    !parse_full_number(toks[8], &dec)) {
                    printf("[警告] movej SPD/ACC/DEC 须为纯数字：%s,%s,%s\n",
                           toks[6], toks[7], toks[8]);
                    return CMD_UNKNOWN;
                }
                if (spd <= 0.0) {
                    printf("[警告] movej 速度须大于 0 rpm\n");
                    return CMD_UNKNOWN;
                }
                if (acc <= 0.0 || dec <= 0.0) {
                    printf("[警告] movej 加减速时间须大于 0 ms\n");
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
            /* 单关节：movej:N:ANGLE[:SPD] */
            char *j = strtok_r(rest, ":", &save);
            char *a = strtok_r(NULL, ":", &save);
            char *s = strtok_r(NULL, ":", &save);
            char *extra = strtok_r(NULL, ":", &save);
            double jnum;
            double ang;
            int joint;
            if (j == NULL || a == NULL || extra != NULL) {
                printf("[警告] 用法: movej:关节号:角度[:速度]\n");
                return CMD_UNKNOWN;
            }
            if (!parse_full_number(j, &jnum) || !parse_full_number(a, &ang)) {
                printf("[警告] movej 关节号/角度须为纯数字\n");
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
            if (s != NULL) {
                double spd = 0.0;
                if (!parse_full_number(s, &spd)) {
                    printf("[警告] movej 速度须为纯数字：%s\n", s);
                    return CMD_UNKNOWN;
                }
                if (spd <= 0.0) {
                    printf("[警告] movej 速度须大于 0 rpm\n");
                    return CMD_UNKNOWN;
                }
                out->speed_rpm = spd;
            } else {
                out->speed_rpm = 0.0;   /* 0 = 使用默认速度 */
            }
            out->num_joints = 1;
        }
    } else if (strcmp(cmd, "movel") == 0) {
        char *rest = save;
        char *ctx = NULL;
        char *tok = strtok_r(rest, ",", &ctx);
        double vals[6];
        int n = 0;
        while (tok != NULL && n < 6) {
            vals[n] = atof(tok);
            n++;
            tok = strtok_r(NULL, ",", &ctx);
        }
        if (n != 6) {
            printf("[警告] 用法: movel:X,Y,Z,Rx,Ry,Rz（mm, 度）\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_MOSEL;
        for (int i = 0; i < 6; i++) out->cartesian[i] = vals[i];
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
    } else if (strncmp(cmd, "zero", 4) == 0) {
        if (strstr(out->raw, "save") != NULL) {
            /* zero_save:v1,v2,v3,v4,v5,v6：直接保存指定的零点值 */
            char *valstr = strchr(out->raw, ':');
            double vals[6];
            int n = 0;
            char *ctx = NULL;
            if (valstr != NULL) {
                valstr++; /* 跳过 ':' */
                char *tok = strtok_r(valstr, ",", &ctx);
                while (tok != NULL && n < 6) {
                    vals[n] = atof(tok);
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
/* 命令帮助文本                                                      */
/* ------------------------------------------------------------------ */

static const char HELP_TEXT[] =
    "可用命令:\n"
    "  home                  回零（全轴）\n"
    "  home:N                仅单独回零关节 N，堵转后自动到该轴配置角\n"                      
    "  movej:N:ANGLE[:SPD]  单关节绝对运动：轴N 至角度ANGLE(度)，速度SPEED(rpm)\n"
    "  movej:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC 多关节同步运动（绝对角度）\n"
    "  movel:X,Y,Z,Rx,Ry,Rz  绝对笛卡尔坐标运动（mm, deg）\n"
    "  disable               全部失能所有关节\n"
    "  disable:N             仅单独泄力(失能)关节 N\n"
    "  enable                恢复使能所有关节\n"
    "  motor                 启动/停止电机实时监控（3S/次循环显示）\n"
    "  getpos                读取当前关节角(度)与笛卡尔坐标(X,Y,Z,RPY)\n"
    "  zero                  显示当前零点与机械角\n"
    "  zero_save:v1,v2,v3,v4,v5,v6  保存指定的零点标定值\n"
    "  help                  帮助\n"
    "  exit                  退出\n";

void cmd_print_help(void)
{
    fputs(HELP_TEXT, stdout);
}
