/*
 * ini_rw.c —— 极简 ini 读写（仅服务于 joint_zero 段持久化）
 * ------------------------------------------------------------
 * 设计：不引入第三方 ini 库，沿用 main.c 的 [serial] 解析风格。
 * 写入时整体重写文件，但丢弃旧的 [joint_zero] 段，保留其它段与注释，
 * 因此 [serial] 与人工注释不会丢失。
 */

#include "utils/ini_rw.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 读取 [joint_zero] 段（q0..q5） */
int ini_read_joint_zero(const char *path, double zero[6])
{
    FILE *f = fopen(path, "r");
    char line[256];
    int in_jz = 0;
    int got = 0;
    int i;

    if (f == NULL) return 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '[') {
            in_jz = (strncmp(p, "[joint_zero]", sizeof("[joint_zero]") - 1) == 0) ? 1 : 0;
            continue;
        }
        if (!in_jz) continue;
        for (i = 0; i < 6; i++) {
            char key[8];
            int klen;
            snprintf(key, sizeof(key), "q%d", i);
            klen = (int)strlen(key);
            if (strncmp(p, key, (size_t)klen) == 0) {
                char *eq = strchr(p, '=');
                if (eq != NULL) {
                    zero[i] = atof(eq + 1);
                    got++;
                }
                break;
            }
        }
    }
    fclose(f);
    return (got == 6) ? 1 : 0;
}

/* 写回 [joint_zero] 段，保留其它段与注释 */
int ini_write_joint_zero(const char *path, const double zero[6])
{
    char buf[8192];
    int in_jz = 0;
    size_t total = 0;
    FILE *f;
    FILE *out;

    /* 读入全部内容，丢弃已有的 [joint_zero] 段 */
    f = fopen(path, "r");
    if (f != NULL) {
        char line[256];
        while (fgets(line, sizeof(line), f) != NULL) {
            char *p = line;
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '[') {
                in_jz = (strncmp(p, "[joint_zero]", sizeof("[joint_zero]") - 1) == 0) ? 1 : 0;
                if (in_jz) continue;   /* 丢弃 joint_zero 段头 */
                /* 其它段头保留 */
            } else if (in_jz) {
                continue;               /* 丢弃 joint_zero 段内容 */
            }
            {
                size_t len = strlen(line);
                if (total + len + 1 < sizeof(buf)) {
                    memcpy(buf + total, line, len);
                    total += len;
                }
            }
        }
        fclose(f);
    }

    out = fopen(path, "w");
    if (out == NULL) return 0;
    if (total > 0) {
        fwrite(buf, 1, total, out);
        if (total == 0 || buf[total - 1] != '\n') fputc('\n', out);
    }
    fprintf(out, "[joint_zero]\n");
    fprintf(out, "q0 = %.2f\n", zero[0]);
    fprintf(out, "q1 = %.2f\n", zero[1]);
    fprintf(out, "q2 = %.2f\n", zero[2]);
    fprintf(out, "q3 = %.2f\n", zero[3]);
    fprintf(out, "q4 = %.2f\n", zero[4]);
    fprintf(out, "q5 = %.2f\n", zero[5]);
    fclose(out);
    return 1;
}

/* 读取 [tool] 段（tool_length，单位 mm） */
int ini_read_tool_length(const char *path, double *tool_mm)
{
    FILE *f = fopen(path, "r");
    char line[256];
    int in_tool = 0;

    if (f == NULL) return 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '[') {
            in_tool = (strncmp(p, "[tool]", sizeof("[tool]") - 1) == 0) ? 1 : 0;
            continue;
        }
        if (!in_tool) continue;
        if (strncmp(p, "tool_length", sizeof("tool_length") - 1) == 0) {
            char *eq = strchr(p, '=');
            if (eq != NULL) {
                *tool_mm = atof(eq + 1);
                fclose(f);
                return 1;
            }
        }
    }
    fclose(f);
    return 0;
}
