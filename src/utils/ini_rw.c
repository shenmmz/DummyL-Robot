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
    /* 【2026-09-21 修 bug：固定 buf[8192] 导致静默截断】
     * 原来这里是 `char buf[8192]`，读入时 `if (total + len + 1 < sizeof(buf))`
     * 不满足的行被【直接丢弃，不报错】。文件一大，写回时后半截就凭空消失。
     *
     * 实测后果（这个坑藏得很深）：ini 本身已 8403 字节、超过 8192，
     * 补回 [stall] 段后更大 ⇒ 每次 `zero_save` 都会把位于文件中后部的
     * [stall]（逐轴过流阈值）连同其它尾部内容一起删掉，
     * 表现为【过流保护莫名其妙变成关闭】，而程序一句提示都没有。
     *
     * ⇒ 改为按文件实际大小动态分配，并且真发生截断时打印出来，不再静默。 */
    char  *buf = NULL;
    size_t cap = 0;
    size_t total = 0;
    int    in_jz = 0;
    int    truncated = 0;
    FILE  *f;
    FILE  *out;

    /* 读入全部内容，丢弃已有的 [joint_zero] 段 */
    f = fopen(path, "r");
    if (f != NULL) {
        char line[256];
        long fsize = 0;

        fseek(f, 0, SEEK_END);
        fsize = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (fsize < 0) fsize = 0;

        cap = (size_t)fsize + 16384;      /* 文件原大小 + 充足余量 */
        buf = (char *)malloc(cap);
        if (buf == NULL) {
            fclose(f);
            printf("[错误] ini 写入失败：内存不足（需要 %lu 字节）\n",
                   (unsigned long)cap);
            return 0;
        }

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
                if (total + len + 1 < cap) {
                    memcpy(buf + total, line, len);
                    total += len;
                } else {
                    truncated = 1;      /* 不再静默：下面会打印 */
                }
            }
        }
        fclose(f);
    }

    if (truncated) {
        printf("[警告] ini 文件过大，写入时发生截断（缓冲 %lu 字节）——"
               " 请检查 %s 内容是否完整\n", (unsigned long)cap, path);
    }

    out = fopen(path, "w");
    if (out == NULL) { free(buf); return 0; }
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
    free(buf);
    return 1;
}

/* 读取 [stall] 段（j1..j6，单位 mA）
 * 逐轴堵转电流阈值；0 表示该轴不检测。缺段或缺任一轴都返回 0，
 * 调用方回退到编译期默认表（ROBOT_STALL_CURRENT_MA_TABLE）。 */
int ini_read_stall_current(const char *path, int th[6])
{
    FILE *f = fopen(path, "r");
    char line[256];
    int in_stall = 0;
    int got = 0;
    int i;

    if (f == NULL) return 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '[') {
            in_stall = (strncmp(p, "[stall]", sizeof("[stall]") - 1) == 0) ? 1 : 0;
            continue;
        }
        /* 注释行也要跳过：段内注释若以 j1..j6 开头（例如被误写成 "j1 = 1500  # 注释"
         * 之外的形式）会被当成配置读走。行首 # 或 ; 一律整行忽略。 */
        if (*p == '#' || *p == ';' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (!in_stall) continue;
        for (i = 0; i < 6; i++) {
            char key[8];
            int klen;
            snprintf(key, sizeof(key), "j%d", i + 1);
            klen = (int)strlen(key);
            if (strncmp(p, key, (size_t)klen) == 0) {
                char *eq = strchr(p, '=');
                if (eq != NULL) {
                    th[i] = atoi(eq + 1);
                    got++;
                }
                break;
            }
        }
    }
    fclose(f);
    return (got == 6) ? 1 : 0;
}

/* ini_read_positive_double：通用读取 —— 指定段 + 键，值必须 > 0。
 * section 传不带方括号的段名（如 "safety"），key 传键名（如 "max_step_deg"）。
 * 返回 1 = 读到；0 = 文件打不开 / 段不存在 / 键不存在 / 值 <= 0。
 *
 * 【为什么值必须 > 0】这些键全是安全阈值。配成 0 或负数时若当成"读到了 0"
 * 回传，调用方会把它解释成"不限"——等于用户手滑一下就把闸门拆了。
 * 宁可回退编译期默认，也不要让一个无效配置悄悄关掉保护。
 *
 * 【为什么键匹配用 strncmp 而不是精确比较】沿用本文件既有风格，允许
 * "max_step_deg = 720" 这类写法；注释行（行首 # 或 ;）整行跳过，
 * 段外同名键不串味 —— 这三条都有单测（tests/test_safety_gate.c 第 3 节）。 */
int ini_read_positive_double(const char *path, const char *section,
                             const char *key, double *out)
{
    FILE *f;
    char line[256];
    char sec_hdr[64];
    int in_sec = 0;

    if (path == NULL || section == NULL || key == NULL || out == NULL) return 0;
    snprintf(sec_hdr, sizeof(sec_hdr), "[%s]", section);

    f = fopen(path, "r");
    if (f == NULL) return 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '[') {
            in_sec = (strncmp(p, sec_hdr, strlen(sec_hdr)) == 0) ? 1 : 0;
            continue;
        }
        if (*p == '#' || *p == ';' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (!in_sec) continue;
        if (strncmp(p, key, strlen(key)) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            double v = atof(eq + 1);
            if (v > 0.0) {
                *out = v;
                return 1;
            }
        }
        return 0;   /* 解析成 0 或负数 = 无效配置，回退默认而不是"关闭闸门" */
    }
    fclose(f);
    return 0;
}

/* 读取 [safety] max_step_deg（单次下发位移上限，机械角度） */
int ini_read_max_step_deg(const char *path, double *deg)
{
    return ini_read_positive_double(path, "safety", "max_step_deg", deg);
}

/* 读取 [safety] max_jump_deg（MoveL 规划层相邻插补点单关节跳变上限，机械角度） */
int ini_read_max_jump_deg(const char *path, double *deg)
{
    return ini_read_positive_double(path, "safety", "max_jump_deg", deg);
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

/* 读取 [tool] 段（pen_length，单位 mm）。语义见 ini_rw.h 的注释：
 * 只用于告警换算，不改坐标。 */
int ini_read_pen_length(const char *path, double *pen_mm)
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
        if (strncmp(p, "pen_length", sizeof("pen_length") - 1) == 0) {
            char *eq = strchr(p, '=');
            if (eq != NULL) {
                *pen_mm = atof(eq + 1);
                fclose(f);
                return 1;
            }
        }
    }
    fclose(f);
    return 0;
}
