/*
 * cmd_parser.c —— 命令行交互命令解析与帮助输出
 * ------------------------------------------------------------
 * 所属模块：工具层（utils）
 * 对外接口：cmd_print_help、cmd_parse
 * 依赖模块：utils/logger
 */

#include "utils/cmd_parser.h"
#include "utils/logger.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* cmd_print_help：打印命令行可用命令帮助 */
void cmd_print_help(void)
{
    printf("可用命令:\n");
    printf("  home                  回零\n");
    printf("  movej:N:ANGLE         关节 N 绝对运动到 ANGLE 度 (N=1..6)\n");
    printf("  movej:N:ANGLE:SPEED   指定速度 (rpm)\n");
    printf("  enable:N              使能关节 N\n");
    printf("  disable:N             失能关节 N\n");
    printf("  status                查询所有关节状态\n");
    printf("  mask:N                屏蔽关节 N（跳过该关节，不发指令）\n");
    printf("  unmask:N              恢复关节 N\n");
    printf("  scan                  扫描总线电机\n");
    printf("  calib                 单关节手动调试\n");
    printf("  help                  帮助\n");
    printf("  exit                  退出\n");
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
        out->type = CMD_HOME;
    } else if (strcmp(cmd, "movej") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        char *a = strtok_r(NULL, ":", &save);
        char *s = strtok_r(NULL, ":", &save);
        int joint;
        double angle;
        if (j == NULL || a == NULL) {
            LOG_WARN("用法: movej:关节号:角度[:速度]");
            return CMD_UNKNOWN;
        }
        joint = atoi(j);
        angle = atof(a);
        if (joint < 1 || joint > 6) {
            LOG_WARN("关节号须在 1..6 之间");
            return CMD_UNKNOWN;
        }
        out->type = CMD_MOVEJ;
        out->joint = joint;
        out->angle_deg = angle;
        out->speed_rpm = (s != NULL) ? atof(s) : 0.0;
    } else if (strcmp(cmd, "enable") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            LOG_WARN("用法: enable:关节号");
            return CMD_UNKNOWN;
        }
        out->type = CMD_ENABLE;
        out->joint = atoi(j);
    } else if (strcmp(cmd, "disable") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            LOG_WARN("用法: disable:关节号");
            return CMD_UNKNOWN;
        }
        out->type = CMD_DISABLE;
        out->joint = atoi(j);
    } else if (strcmp(cmd, "status") == 0) {
        out->type = CMD_STATUS;
    } else if (strcmp(cmd, "mask") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            LOG_WARN("用法: mask:关节号");
            return CMD_UNKNOWN;
        }
        out->type = CMD_MASK;
        out->joint = atoi(j);
    } else if (strcmp(cmd, "unmask") == 0) {
        char *j = strtok_r(NULL, ":", &save);
        if (j == NULL) {
            LOG_WARN("用法: unmask:关节号");
            return CMD_UNKNOWN;
        }
        out->type = CMD_UNMASK;
        out->joint = atoi(j);
    } else if (strcmp(cmd, "scan") == 0) {
        out->type = CMD_SCAN;
    } else if (strcmp(cmd, "calib") == 0) {
        out->type = CMD_CALIB;
    } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        out->type = CMD_HELP;
    } else if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "quit") == 0) {
        out->type = CMD_EXIT;
    } else {
        out->type = CMD_UNKNOWN;
    }
    return out->type;
}
