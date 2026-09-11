/*
 * cmd_parser.c —— 命令行交互命令解析与帮助输出
 * ------------------------------------------------------------
 * 所属模块：工具层（utils）
 * 对外接口：cmd_parse、cmd_print_help
 * 支持命令：home、movej、help、exit
 */

#include "utils/cmd_parser.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
        char *j = strtok_r(NULL, ":", &save);
        char *a = strtok_r(NULL, ":", &save);
        char *s = strtok_r(NULL, ":", &save);
        int joint;
        double angle;
        if (j == NULL || a == NULL) {
            printf("[警告] 用法: movej:关节号:角度[:速度]\n");
            return CMD_UNKNOWN;
        }
        joint = atoi(j);
        angle = atof(a);
        if (joint < 1 || joint > 6) {
            printf("[警告] 关节号须在 1..6 之间\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_MOVEJ;
        out->joint = joint;
        out->angle_deg = angle;
        out->speed_rpm = (s != NULL) ? atof(s) : 0.0;
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
    } else if (strcmp(cmd, "motor") == 0) {
        out->type = CMD_MOTOR;
    } else if (strcmp(cmd, "getpos") == 0) {
        out->type = CMD_GETPOS;
    } else if (strncmp(cmd, "zero", 4) == 0) {
        out->type = CMD_ZERO;
        out->joint = (strstr(out->raw, "save") != NULL) ? 1 : 0;
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
    "  movej:N:ANGLE         控制轴N，绝对角度ANGLE(度)\n"
    "  movej:N:ANGLE:SPEED   控制轴N，绝对角度ANGLE(度)，速度SPEED(rpm)\n"
    "  disable               全部失能所有关节\n"
    "  disable:N             仅单独泄力(失能)关节 N\n"
    "  motor                 启动/停止电机实时监控（3S/次循环显示）\n"
    "  getpos                读取当前关节角(度)与笛卡尔坐标(X,Y,Z,RPY)\n"
    "  zero                  显示零点标定数据（零点、当前读数、修正值）\n"
    "  zero save             保存当前零点标定值\n"
    "  help                  帮助\n"
    "  exit                  退出\n";

void cmd_print_help(void)
{
    fputs(HELP_TEXT, stdout);
}
