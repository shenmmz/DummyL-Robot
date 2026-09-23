
#include "utils/ini_rw.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 读 ini [joint_zero] 的 q0[6]（机械角零点）。缺键时用内置默认值。 */
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

/* 写回 ini [joint_zero]。纯逐行拷贝 + fsize+16384 动态缓冲，不动其他段。
 * ⚠️ 历史事故：早期用 8192 固定缓冲，ini 一超长就被截断，[stall] 整段凭空消失。 */
int ini_write_joint_zero(const char *path, const double zero[6])
{
    char  *buf = NULL;
    size_t cap = 0;
    size_t total = 0;
    int    in_jz = 0;
    int    truncated = 0;
    FILE  *f;
    FILE  *out;

    f = fopen(path, "r");
    if (f != NULL) {
        char line[256];
        long fsize = 0;

        fseek(f, 0, SEEK_END);
        fsize = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (fsize < 0) fsize = 0;

        cap = (size_t)fsize + 16384;
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
                if (in_jz) continue;
            } else if (in_jz) {
                continue;
            }
            {
                size_t len = strlen(line);
                if (total + len + 1 < cap) {
                    memcpy(buf + total, line, len);
                    total += len;
                } else {
                    truncated = 1;
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

/* 读 ini [stall] 六轴堵转电流阈值（mA）。
 * ⚠️ 阈值疑似偏低：实测静止电流 ~500/499/495/385/371/254 vs 阈值 480/490/480/400/390
 * ⇒ J1/J2/J3 一上电就报"已堵转"。未证实，改前先测。 */
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

/* 从 ini 读一个 >0 的 double。读不到、非正数、缺键、缺段一律返回 0，
 * 由调用方兜底默认值。
 * ⚠️ 只认段头精确匹配 [section]；'#' 与 ';' 开头的行都跳过。 */
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
        return 0;
    }
    fclose(f);
    return 0;
}

/* 读 ini [safety] max_step_deg：单次下发允许的最大步长（度）。 */
int ini_read_max_step_deg(const char *path, double *deg)
{
    return ini_read_positive_double(path, "safety", "max_step_deg", deg);
}

/* 读 ini [safety] max_jump_deg：单次允许的最大跳变（度），超了就拒绝下发。 */
int ini_read_max_jump_deg(const char *path, double *deg)
{
    return ini_read_positive_double(path, "safety", "max_jump_deg", deg);
}

/* 读 ini [tool] tool_length：d6=183（装夹爪末端）之外再追加的工具长度（mm）。 */
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

/* 读 ini [tool] pen_length：笔尖长度（mm）。实测标定为 41.17。 */
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
