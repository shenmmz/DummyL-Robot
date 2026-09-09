/*
 * cmd_parser.c —— 命令行交互命令解析与帮助输出
 * ------------------------------------------------------------
 * 所属模块：工具层（utils）
 * 对外接口：cmd_parse（命令帮助文本见 utils/help_text.h / help_text.c）
 * 依赖模块：utils/logger
 */

#include "utils/cmd_parser.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* parse_joint_list：解析逗号分隔的关节列表（如 "2,3,4"）到 out->joints。
 * 任一关节号非法（<1 或 >6）则整体失败返回 -1，避免只屏蔽一半。
 * 单个关节（如 "2"）同样兼容。 */
static int parse_joint_list(const char *s, ParsedCmd *out)
{
    char buf[32];
    char *save = NULL;
    char *tok;
    int n = 0;

    snprintf(buf, sizeof(buf), "%s", s);
    tok = strtok_r(buf, ",", &save);
    while (tok != NULL) {
        int j = atoi(tok);
        if (j < 1 || j > 6) {
            printf("[警告] 关节号须在 1..6 之间：%s\n", tok);
            return -1;
        }
        out->joints[n++] = j;
        tok = strtok_r(NULL, ",", &save);
    }
    out->joint_count = n;
    out->joint = (n > 0) ? out->joints[0] : 0;
    return 0;
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
            out->joint = joint;   /* 带关节号 = 单轴独立回零 */
        } else {
            out->joint = 0;       /* 不带 = 全轴回零 */
        }
    } else if (strcmp(cmd, "homej") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        char *a = strtok_r(NULL, ":", &save);
        char *s = strtok_r(NULL, ":", &save);
        int joint;
        double angle;
        if (j == NULL || a == NULL) {
            printf("[警告] 用法: homej:关节号:角度[:速度]\n");
            return CMD_UNKNOWN;
        }
        joint = atoi(j);
        angle = atof(a);
        if (joint < 1 || joint > 6) {
            printf("[警告] 关节号须在 1..6 之间\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_HOMEJ;
        out->joint = joint;
        out->angle_deg = angle;
        out->speed_rpm = (s != NULL) ? atof(s) : 0.0;
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
    } else if (strcmp(cmd, "enable") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            printf("[警告] 用法: enable:关节号\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_ENABLE;
        out->joint = atoi(j);
    } else if (strcmp(cmd, "disable") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            printf("[警告] 用法: disable:关节号\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_DISABLE;
        out->joint = atoi(j);
    } else if (strcmp(cmd, "status") == 0) {
        out->type = CMD_STATUS;
    } else if (strcmp(cmd, "mask") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            printf("[警告] 用法: mask:关节号[,关节号...]  例 mask:2 或 mask:2,3,4\n");
            return CMD_UNKNOWN;
        }
        if (parse_joint_list(j, out) != 0) {
            return CMD_UNKNOWN;
        }
        out->type = CMD_MASK;
    } else if (strcmp(cmd, "unmask") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            printf("[警告] 用法: unmask:关节号[,关节号...]  例 unmask:2 或 unmask:2,3,4\n");
            return CMD_UNKNOWN;
        }
        if (parse_joint_list(j, out) != 0) {
            return CMD_UNKNOWN;
        }
        out->type = CMD_UNMASK;
    } else if (strcmp(cmd, "scan") == 0) {
        out->type = CMD_SCAN;
    } else if (strcmp(cmd, "calib") == 0) {
        out->type = CMD_CALIB;
    } else if (strcmp(cmd, "diag") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        out->type = CMD_DIAG;
        out->joint = (j != NULL) ? atoi(j) : 0;   /* 0 = 全部关节 */
    } else if (strcmp(cmd, "torque") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        char *l = strtok_r(NULL, ":", &save);
        int joint, level;
        if (j == NULL || l == NULL) {
            printf("[警告] 用法: torque:关节号:等级  例 torque:4:120 在4轴以等级120试撞\n");
            return CMD_UNKNOWN;
        }
        joint = atoi(j);
        level = atoi(l);
        if (joint < 1 || joint > 6) {
            printf("[警告] 关节号须在 1..6 之间\n");
            return CMD_UNKNOWN;
        }
        if (level < 0 || level > 255) {
            printf("[警告] 力矩等级须在 0..255 之间\n");
            return CMD_UNKNOWN;
        }
        out->type = CMD_TORQUE;
        out->joint = joint;
        out->torque_level = level;
    } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        out->type = CMD_HELP;
    } else if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "quit") == 0) {
        out->type = CMD_EXIT;
    } else {
        out->type = CMD_UNKNOWN;
    }
    return out->type;
}
