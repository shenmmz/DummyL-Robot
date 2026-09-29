
#include "utils/cmd_parser.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* MSVC 没有 POSIX 的 strtok_r（链接报 LNK2019，且因未声明被当返回 int ⇒ C4047 一串）。
 * 它的 strtok_s 签名 strtok_s(char*, const char*, char**) 与 strtok_r 完全一致，直接映射。 */
#ifdef _MSC_VER
#define strtok_r strtok_s
#endif

/* 忽略大小写的字符串比较（命令名不区分大小写）。 */
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

/* 解析一个"完整"数字：整串都必须合法，不接受 "12abc" 这种。 */
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

/* 把一行文本解析成 ParsedCmd。
 * ⚠️ 项目铁律（踩过坑）：新增命令时 out->type 要么放在【全部校验之后】设置，
 * 要么保证失败时复位。历史事故：分支先设 out->type 再校验，失败 return CMD_UNKNOWN，
 * 但 main.c【只看 cmd->type 不看返回值】⇒ 非法参数（如 busrate:7）打完警告命令照样执行。
 * 修法：入口 memset(out,0,...) + 46 处先复位 type 再返回。
 * ⚠️ line==NULL || out==NULL 那处【不能】写 out->type。 */
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
                    out->joints[i] = i + 1;
                    out->angles[i] = ang[i];
                }
                out->speeds[0] = spd;
                out->accel_ms[0] = (int)acc;
                out->decel_ms[0] = (int)dec;
                out->type = CMD_MOVEJ;
            }
        } else {
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
            out->rel = 0;
            out->speed_rpm = 0.0;
            out->num_joints = 1;
            {
                char *seg[2];
                int k, speed_set = 0, mode_set = 0;
                seg[0] = f1; seg[1] = f2;
                for (k = 0; k < 2; k++) {
                    double num;
                    if (seg[k] == NULL) continue;
                    if (parse_full_number(seg[k], &num)) {
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
                    } else if (strcmp(seg[k], "r") == 0) {
                        if (mode_set) {
                            printf("[警告] MoveJ 模式段重复\n");
                            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                        }
                        out->rel = 1;
                        mode_set = 1;
                    } else if (strcmp(seg[k], "a") == 0) {
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
        char *rest = save;
        char *ctx = NULL;
        char *tok = strtok_r(rest, ",", &ctx);
        double v[9];
        int n = 0, i;
        while (tok != NULL && n < 9) {
            if (!parse_full_number(tok, &v[n])) {
                break;
            }
            n++;
            tok = strtok_r(NULL, ",", &ctx);
        }

        if (n != 3 && n != 6 && n != 9) {
            printf("[警告] 用法: MoveL:X,Y,Z[,SPD,ACC,DEC]\n"
                   "       带姿态须写满 9 段: MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_MOVEL;
        out->movl_mode = MOVL_MODE_SMOOTH;
        for (i = 0; i < 3; i++) out->cartesian[i] = v[i];
        /* ★ 段数语义（只放行 9 段带姿态）：
         *   3 段 X,Y,Z                      → 姿态抄当前（keep_pose=1），速度 60/80/90
         *   6 段 X,Y,Z,SPD,ACC,DEC          → 姿态抄当前（keep_pose=1），自定义速度
         *   9 段 X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC  → 用给定姿态（keep_pose=0）
         * ⚠️ 3/6 段第 4 个数一律当【速度】而非 Rx —— 想显式指定姿态必须写满 9 段。
         *   终点姿态若与 FK 起点不等，SLERP 会把姿态一路拧过去 ⇒ 笔尖绕法兰摆
         *   pen_length×2sin(θ/2) ⇒ 画斜线（实测 80mm 线偏 7.71mm）；故非 9 段恒沿用
         *   当前姿态。9 段执行前 cmd_movel 用 movl_pose_warn 比对当前姿态，超阈
         *   打警告但仍照走（用户显式指令优先）。 */
        if (n == 9) {
            out->keep_pose = 0;
            for (i = 3; i < 6; i++) out->cartesian[i] = v[i];
            if (v[6] <= 0.0) {
                printf("[警告] MoveL 速度须大于 0 rpm（收到 %.2f）\n", v[6]);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            if (v[7] <= 0.0 || v[8] <= 0.0) {
                printf("[警告] MoveL 加减速时间须大于 0 ms\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speeds[0]   = v[6];
            out->accel_ms[0] = (int)v[7];
            out->decel_ms[0] = (int)v[8];
        } else {
            out->keep_pose = 1;
            for (i = 3; i < 6; i++) out->cartesian[i] = 0.0;  /* cmd_movel 会用 start_pose 覆盖 */
            if (n == 3) {
                out->speeds[0]   =  60;
                out->accel_ms[0] =  80;
                out->decel_ms[0] =  90;
            } else {
                if (v[3] <= 0.0) {
                    printf("[警告] MoveL 第 4 个参数是【速度 rpm】，须大于 0（收到 %.2f）。\n", v[3]);
                    printf("       若要显式指定姿态，请写满 9 段 MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                if (v[4] <= 0.0 || v[5] <= 0.0) {
                    printf("[警告] MoveL 加减速时间须大于 0 ms\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->speeds[0]   = v[3];
                out->accel_ms[0] = (int)v[4];
                out->decel_ms[0] = (int)v[5];
            }
        }

        if (tok != NULL && strcmp(tok, "keep") == 0) {
            out->keep_pose = 1;   /* 强制沿用当前姿态（覆盖 9 段里给定的 Rx,Ry,Rz） */
            for (i = 3; i < 6; i++) out->cartesian[i] = 0.0;
            tok = strtok_r(NULL, ",", &ctx);
        }
        if (tok != NULL) {
            if (strcmp(tok, "smooth") == 0) {
                out->movl_mode = MOVL_MODE_SMOOTH;
            } else if (strcmp(tok, "interp") == 0 || strcmp(tok, "step") == 0) {
                out->movl_mode = MOVL_MODE_INTERP;
            } else if (strcmp(tok, "stream") == 0 || strcmp(tok, "sync") == 0) {
                printf("[警告] MoveL 模式 %s 已移除：可用 smooth（默认，不插补）或 interp（逐点插补）。\n", tok);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            } else {
                printf("[警告] MoveL 参数过多，用法: MoveL:X,Y,Z[,SPD,ACC,DEC][,smooth|interp] 或 9 段带姿态\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            tok = strtok_r(NULL, ",", &ctx);
            if (tok != NULL) {
                printf("[警告] MoveL 参数过多，用法: MoveL:X,Y,Z[,SPD,ACC,DEC][,smooth|interp] 或 9 段带姿态\n");
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
            out->joint = 0;
        }
    } else if (strcmp(cmd, "enable") == 0) {
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
            out->joint = 0;
        }
    } else if (strcmp(cmd, "motor") == 0) {
        out->type = CMD_MOTOR;
    } else if (strcmp(cmd, "getpos") == 0) {
        out->type = CMD_GETPOS;
    } else if (strcmp(cmd, "fk") == 0) {
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
        out->type = CMD_DIAG;
    } else if (strcmp(cmd, "bcast") == 0) {
        out->type = CMD_BCAST;
    } else if (strcmp(cmd, "nrtest") == 0) {
        out->type = CMD_NRTEST;
    } else if (strcmp(cmd, "curtest") == 0) {
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: curtest  或  curtest:关节号[:摆幅度[:转速rpm]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_CURTEST;
        out->joint = 0;
        out->angle_deg = 2.0;
        out->speed_rpm = 30.0;
        if (a == NULL) return CMD_CURTEST;
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
        char *a = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: accel  或  accel:加速ms  或  accel:加速ms,减速ms\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->angle_deg = -1.0;
        out->speed_rpm = -1.0;
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
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: drvbaud  或  drvbaud:档位码[:PC波特率]  或  drvbaud:save\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->joint = 0;
        out->param = 0.0;
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
    } else if (strcmp(cmd, "pipe") == 0) {
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: pipe  或  pipe:每档轮数[:单档间隔us]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->joint = 20;
        out->param = -1.0;
        if (a != NULL) {
            double r;
            if (!parse_full_number(a, &r) || r < 1.0 || r > 200.0 ||
                r != (double)(int)r) {
                printf("[警告] pipe 每档轮数须为 1~200 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)r;
        }
        if (b != NULL) {
            double g;
            if (!parse_full_number(b, &g) || g < 0.0 || g > 5000.0) {
                printf("[警告] pipe 间隔须为 0~5000 us：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->param = g;
        }
        out->type = CMD_PIPE;
    } else if (strcmp(cmd, "alarm") == 0) {
        char *a = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: alarm  或  alarm:clear\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->rel = 0;
        if (a != NULL) {
            if (strcmp(a, "clear") != 0) {
                printf("[警告] alarm 只接受 clear：alarm  或  alarm:clear\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->rel = 1;
        }
        out->type = CMD_ALARM;
    } else if (strcmp(cmd, "looptest") == 0) {
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
        out->type = CMD_POSEOK;
    } else if (strncmp(cmd, "zero", 4) == 0) {
        if (strstr(out->raw, "save") != NULL) {
            char *valstr = strchr(out->raw, ':');
            double vals[6];
            int n = 0;
            char *ctx = NULL;
            if (valstr != NULL) {
                valstr++;
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


static const char HELP_TEXT[] =
    "可用命令（命名对齐 ABB RAPID，大小写不敏感）:\n"
    "  home                  回零（全轴）\n"
    "  home:N                仅单独回零关节 N，堵转后自动到该轴配置角\n"
    "  MoveJ:N:ANGLE[:SPD][:r|a]   单关节关节空间运动：轴N 至角度ANGLE(度)，\n"
    "                          速度SPEED(rpm)，末段 r=相对当前位置 / a=绝对(默认)\n"
    "  MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC   多关节同步关节空间运动\n"
    "  MoveL:X,Y,Z                 笛卡尔直线（姿态恒取当前 getpos，推荐）\n"
    "  MoveL:X,Y,Z,SPD,ACC,DEC    同上 + 自定义速度（SPD=rpm，ACC/DEC=ms）\n"
    "  MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC   9 段：显式指定目标姿态（SPD/ACC/DEC 必填）\n"
    "  MoveL:...,DEC,interp      末尾加 ,interp 走逐点插补（末端贴直线、逐段保护）\n"
    "                          ★ 写法约定：\n"
    "                          ① 只 3/6 段时姿态沿用当前位姿，第 4 个数是速度不是 Rx；\n"
    "                             要显式给 Rx,Ry,Rz 必须写满 9 段\n"
    "                          ② 9 段若姿态与当前差太多，会警告\"拧姿态、笔尖偏离\"\n"
    "                             （实测偏 7.71mm），但仍照走；想保直线就抄 getpos 原值\n"
    "                          ③ 模式 smooth（默认）：终点一次 IK + 一次 MoveJ ⇒ 不插补、\n"
    "                             零段间停顿；但末端走弧（弓高 = 整段）且全程不查过流\n"
    "                          ④ 模式 interp：按 [movel] step_mm 逐点插补、逐航点下发并\n"
    "                             等到位 ⇒ 末端贴着直线（弓高极小）+ 每段读过流/超时保护；\n"
    "                             代价是段间有加减速停顿，比 smooth 慢\n"
    "                          ⑤ stream / sync 已移除，写了会被拒绝\n"
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
    "                        判读：单事务耗时看 diag（921600 下 ≈1.7ms），\n"
    "                              它 = 线上 0.23 + 驱动器周转 ≈1.47 + USB栈 ≤0.08 ms\n"
    "                              ⚠️ 回环RTT只代表本工具的往返能力，【不能】从单事务里减\n"
    "                        ⚠️ 必须脱离电机；接电机时切BAUD会立刻失联\n"
    "  bcast                 广播帧验证：地址0写速度再逐轴读回，看几轴响应广播\n"
    "  pipe[:轮数[:间隔us]]   流水线批量读探针（只读位置，不动臂）：\n"
    "                        基线＝现状一问一答；然后先连发 6 个请求、再收 6 个响应，\n"
    "                        扫 0/50/100/200/300/500/1000/2000 us 八档间隔，报丢帧数。\n"
    "                        目的：把\"等响应那 1.6ms 里总线其实是空的\"这块钱榨出来。\n"
    "                        轮数默认 20；给第三个参数只测那一档（如 pipe:30:200）\n"
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

/* 打印帮助文本（help / ?）。注意：这里的 looptest 判读文案必须与 commands.c 的实际
 * 测量口径一致，否则会把人引向错误的优化方向（曾写着已作废的 15.4ms 公式）。 */
void cmd_print_help(void)
{
    fputs(HELP_TEXT, stdout);
}