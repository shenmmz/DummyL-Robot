
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

/* .rbt 命名点位辅助：
 * split_first_field：从 s 取第一个字段（截到 ':' / ',' / 行尾），去首尾空格写入 name。
 *   返回 1 = 该字段非空且不是纯数字（⇒ 当作点位名），同时把 *after 指向字段后的分隔符；
 *   返回 0 = 空或纯数字（⇒ 不是点位名，走原有内联数字解析）。 */
static int split_first_field(const char *s, char *name, size_t nsz, const char **after)
{
    size_t b = 0, e = 0, i;
    double tmp;

    if (s == NULL) return 0;
    while (*s == ' ' || *s == '\t') s++;
    for (i = 0; s[i] != '\0'; i++)
        if (s[i] == ':' || s[i] == ',') break;
    e = i;
    while (e > 0 && (s[e - 1] == ' ' || s[e - 1] == '\t')) e--;  /* 去尾部空格 */
    if (after) *after = s + i;
    if (e == 0) return 0;                       /* 空字段 */
    if ((size_t)(e - b) >= nsz) e = nsz - 1;
    memcpy(name, s, e);
    name[e] = '\0';
    if (parse_full_number(name, &tmp)) return 0; /* 是纯数字 ⇒ 不是点位名 */
    return 1;
}

/* 从 p 起按 delim 解析恰好 want 个数字到 v[]；全为数字且无多余则返回 1，否则 0。 */
static int parse_exact_nums(char *p, const char *delim, double *v, int want)
{
    char *ctx = NULL;
    char *tok = strtok_r(p, delim, &ctx);
    int n = 0;
    while (tok != NULL) {
        if (n >= want) return 0;                 /* 多余 */
        if (!parse_full_number(tok, &v[n])) return 0;
        n++;
        tok = strtok_r(NULL, delim, &ctx);
    }
    return (n == want) ? 1 : 0;
}

/* 从 after（点位名后的分隔符位置）取可选的 SPD,ACC,DEC（':' 或 ',' 分隔，≤ 3 个）。
 * 缺省或不足则对应输出置 0（下游按默认值处理）。返回 0 = 解析失败。 */
static int parse_opt_sad(char *after, double *spd, double *acc, double *dec)
{
    char *ctx = NULL;
    char *tok;
    int n = 0;
    double v[3] = { 0.0, 0.0, 0.0 };
    if (after == NULL) { *spd = *acc = *dec = 0.0; return 1; }
    while (*after == ':' || *after == ',' || *after == ' ' || *after == '\t') after++;
    if (*after == '\0') { *spd = *acc = *dec = 0.0; return 1; }
    tok = strtok_r(after, ":,", &ctx);
    while (tok != NULL) {
        if (n >= 3) return 0;
        if (!parse_full_number(tok, &v[n])) return 0;
        n++;
        tok = strtok_r(NULL, ":,", &ctx);
    }
    *spd = v[0]; *acc = v[1]; *dec = v[2];
    return 1;
}

/* 就地去掉首尾空白，返回指向首个非空白字符的指针（NULL 安全）。 */
static char *strim(char *s)
{
    char *e;
    if (s == NULL) return NULL;
    while (*s == ' ' || *s == '\t') s++;
    e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t')) e--;
    *e = '\0';
    return s;
}

/* 从原始行取“命令词之后的参数区”：跳过命令词，遇到第一个 ':' 或空格
 * 就当分隔符，返回其后（去前导空格）的整段。用于 run/cd 这类路径参数，
 * 同时支持 `run:test` 与 `run test`，且保留 Windows 盘符里的冒号与路径中的空格。 */
static const char *arg_after_first_delim(const char *raw)
{
    const char *d = raw;
    while (*d != '\0' && *d != ':' && *d != ' ' && *d != '\t') d++;
    if (*d == '\0') return "";      /* 无参数 */
    d++;
    while (*d == ' ' || *d == '\t') d++;
    return d;
}

/* 示教器风格命名点位：
 *   TARGET <名>={J1:v J2:v J3:v J4:v J5:v J6:v}   （关节，kind=0）
 *   TARGET <名>={x:v y:v z:v a:v b:v c:v}   （笛卡尔，kind=1）
 * 命中且合法返回 1（已填 def_name/def_kind/def_vals）；是 TARGET 行但不合格返回 -1（已打印错误）；
 * 首词不是 target 返回 0（交回常规解析）。 */
static int try_parse_target(const char *s, ParsedCmd *out)
{
    const char *ws, *ns, *lb, *rb, *ne;
    char  first[8];
    size_t fi, ni;
    char  body[224];
    char *ctx, *tk;
    double vals[6];
    int  nv = 0, kind = -1, i;

    ws = s;
    while (*ws != '\0' && *ws != ' ' && *ws != '\t' && *ws != '=') ws++;
    fi = (size_t)(ws - s);
    if (fi == 0 || fi >= sizeof(first)) return 0;
    memcpy(first, s, fi);
    first[fi] = '\0';
    if (ci_strcmp(first, "target") != 0) return 0;

    ns = ws;
    while (*ns == ' ' || *ns == '\t') ns++;
    lb = strchr(ns, '{');
    rb = (lb != NULL) ? strrchr(lb, '}') : NULL;
    if (lb == NULL || rb == NULL || rb <= lb) {
        printf("[警告] 用法: TARGET <名>={J1:v J2:v J3:v J4:v J5:v J6:v}  或  {x:v y:v z:v a:v b:v c:v}\n");
        return -1;
    }
    /* 名字 = [ns, lb) 去尾部空格与一个 '=' */
    ne = lb;
    while (ne > ns && (ne[-1] == ' ' || ne[-1] == '\t')) ne--;
    if (ne > ns && ne[-1] == '=') {
        ne--;
        while (ne > ns && (ne[-1] == ' ' || ne[-1] == '\t')) ne--;
    }
    ni = (size_t)(ne - ns);
    if (ni == 0 || ni >= sizeof(out->def_name)) {
        printf("[警告] TARGET 点位名不合法（1..31 字符）\n");
        return -1;
    }
    memcpy(out->def_name, ns, ni);
    out->def_name[ni] = '\0';

    {
        size_t bl = (size_t)(rb - (lb + 1));
        if (bl >= sizeof(body)) { printf("[警告] TARGET 内容过长\n"); return -1; }
        memcpy(body, lb + 1, bl);
        body[bl] = '\0';
    }

    for (tk = strtok_r(body, " \t", &ctx); tk != NULL; tk = strtok_r(NULL, " \t", &ctx)) {
        char *colon = strchr(tk, ':');
        char *rest, *endp;
        double v;
        if (colon == NULL) { printf("[警告] TARGET 内每一项须为 键:值 ：%s\n", tk); return -1; }
        *colon = '\0';
        if (nv == 0) {                       /* 首项据键名定类型 */
            if (tk[0] == 'J' || tk[0] == 'j')      kind = 0;
            else if (tk[0] == 'x' || tk[0] == 'X') kind = 1;
            else { printf("[警告] TARGET 首键须为 J1(关节) 或 x(笛卡尔)：%s\n", tk); return -1; }
        }
        rest = colon + 1;
        v = strtod(rest, &endp);
        if (endp == rest || *endp != '\0') { printf("[警告] TARGET 值非数字：%s\n", rest); return -1; }
        if (nv >= 6) { printf("[警告] TARGET 需要恰好 6 个值\n"); return -1; }
        vals[nv++] = v;
    }
    if (nv != 6) { printf("[警告] TARGET 需要恰好 6 个值（当前 %d）\n", nv); return -1; }

    for (i = 0; i < 6; i++) out->def_vals[i] = vals[i];
    out->def_kind = kind;
    return 1;
}

/* 把一行文本解析成 ParsedCmd。 */
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

    if (buf[0] == '#' || buf[0] == ';') {   /* 脚本注释行：静默跳过（方便 run:<文件> 与管道复用）*/
        out->type = CMD_EMPTY;
        out->raw[0] = '\0';
        return CMD_EMPTY;
    }

    {   /* TARGET <名>={...} 命名点位（早于 ':' 分词，因内容里自带冒号） */
        int tgt = try_parse_target(buf, out);
        if (tgt != 0) {
            if (tgt < 0) { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            out->type = CMD_DEFPOINT;
            return CMD_DEFPOINT;
        }
    }

    tok = strtok_r(buf, ":", &save);
    if (tok == NULL) {
        { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
    }
    snprintf(cmd, sizeof(cmd), "%s", tok);

    /* 兼容空格分隔：命令词与参数之间用空格（如 `cd test.rbt`、`getpos movej`）。
     * 若第一个 ':' 分出的词里含空格，就把首个空格前当命令、其后整段设为参数区（覆盖 save），
     * 使各分支的 strtok_r(NULL, ":", &save) 能取到它。含 ':' 的传统写法不受影响。 */
    {
        char *sp = strchr(cmd, ' ');
        if (sp != NULL) {
            size_t off = (size_t)(sp - cmd);
            char  *argp = tok + off + 1;
            *sp = '\0';
            while (*argp == ' ' || *argp == '\t') argp++;
            save = argp;
        }
    }

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
        /* 引用命名关节点位：MoveJ:<点位名>[:SPD,ACC,DEC]。首字段非纯数字即视为点位名。 */
        {
            char nm[32];
            const char *after = NULL;
            if (split_first_field(rest, nm, sizeof nm, &after)) {
                double spd = 0.0, acc = 0.0, dec = 0.0;
                if (strlen(nm) >= sizeof(out->point_name)) {
                    printf("[警告] MoveJ 点位名过长（≤31 字符）：%s\n", nm);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                if (!parse_opt_sad((char *)after, &spd, &acc, &dec)) {
                    printf("[警告] MoveJ:点位名 后只可跟 SPD,ACC,DEC 三个数字\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->use_point = 1;
                snprintf(out->point_name, sizeof(out->point_name), "%s", nm);
                out->speeds[0]   = spd;
                out->accel_ms[0] = (int)acc;
                out->decel_ms[0] = (int)dec;
                out->type = CMD_MOVEJ;
                return CMD_MOVEJ;
            }
        }
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
        /* 引用命名笛卡尔点位：MoveL:<点位名>[:SPD,ACC,DEC[,smooth|interp]]。 */
        {
            char nm[32];
            const char *after = NULL;
            if (split_first_field(rest, nm, sizeof nm, &after)) {
                double spd = 0.0, acc = 0.0, dec = 0.0;
                if (strlen(nm) >= sizeof(out->point_name)) {
                    printf("[警告] MoveL 点位名过长（≤31 字符）：%s\n", nm);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                if (!parse_opt_sad((char *)after, &spd, &acc, &dec)) {
                    printf("[警告] MoveL:点位名 后只可跟 SPD,ACC,DEC 三个数字\n");
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                out->use_point   = 1;
                snprintf(out->point_name, sizeof(out->point_name), "%s", nm);
                out->movl_mode   = MOVL_MODE_INTERP;
                out->speeds[0]   = spd;
                out->accel_ms[0] = (int)acc;
                out->decel_ms[0] = (int)dec;
                out->type = CMD_MOVEL;
                return CMD_MOVEL;
            }
        }
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

        if (n != 9) {
            printf("[警告] 用法: MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC[,smooth|interp]\n"
                   "       必须写满 9 段（含显式姿态 Rx,Ry,Rz），不接受缺姿态的简写。\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_MOVEL;
        out->movl_mode = MOVL_MODE_INTERP;   /* 默认逐点插补；显式 ,smooth 才走 smooth */
        for (i = 0; i < 3; i++) out->cartesian[i] = v[i];
        /* ★ 段数语义：movel 只接受 9 段 X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC（强制显式姿态）。
         *   执行前 cmd_movel 用 movl_pose_warn 比对当前姿态，超阈时告警但仍照走
         *   （用户显式指令优先）。 */
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

        if (tok != NULL) {
            if (strcmp(tok, "smooth") == 0) {
                out->movl_mode = MOVL_MODE_SMOOTH;
            } else if (strcmp(tok, "interp") == 0 || strcmp(tok, "step") == 0) {
                out->movl_mode = MOVL_MODE_INTERP;
            } else if (strcmp(tok, "stream") == 0 || strcmp(tok, "sync") == 0) {
                printf("[警告] MoveL 模式 %s 已移除：可用 interp（默认，逐点插补）或 smooth（不插补，需显式写）。\n", tok);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            } else {
                printf("[警告] MoveL 参数过多，用法: MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC[,smooth|interp]\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            tok = strtok_r(NULL, ",", &ctx);
            if (tok != NULL) {
                printf("[警告] MoveL 参数过多，用法: MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC[,smooth|interp]\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
        }
    } else if (ci_strcmp(cmd, "MoveC") == 0) {
        /* `MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC[,interp]`：三点式空间圆弧（对齐 ABB RAPID MoveC）。
         * 起点=当前位姿；toPoint=(X,Y,Z,Rx,Ry,Rz)（与 movel 同格式、姿态显式）；
         * viaPoint=(VX,VY,VZ) 决定弧的凸向与半径（参数顺序：toPoint→viaPoint→速度）。
         * 圆弧本就靠逐点插补才画得出来，故只支持 interp 模式
         * （smooth 会退化成一次 MoveJ ⇒ 画不出弧，拒绝）。整圆请连发两段 MoveC。 */
        char *rest = save;
        /* 具名三点式：MoveC:<起点名>,<via名>,<终点名>[,SPD,ACC,DEC]。
         * 首字段非数字即走此支；否则进下面 12 数字路径。 */
        if (rest != NULL) {
            char *qq = rest;
            while (*qq == ' ' || *qq == '\t') qq++;
            if (*qq != '\0' && !(*qq == '-' || *qq == '+' || (*qq >= '0' && *qq <= '9'))) {
                char  *ctx2 = NULL;
                char  *tn[6];
                int    k, bad = 0;
                double sad[3] = { 0.0, 0.0, 0.0 };
                char  *extra;
                tn[0] = strtok_r(qq, ",", &ctx2);
                for (k = 1; k < 6; k++) tn[k] = strtok_r(NULL, ",", &ctx2);
                extra = strtok_r(NULL, ",", &ctx2);
                if (extra != NULL) {
                    printf("[警告] MoveC 具名参数过多：MoveC:<起点>,<via>,<终点>[,SPD,ACC,DEC]\n");
                    bad = 1;
                }
                if (!bad && (tn[0] == NULL || tn[1] == NULL || tn[2] == NULL)) {
                    printf("[警告] 用法: MoveC:<起点名>,<via名>,<终点名>[,SPD,ACC,DEC]\n");
                    bad = 1;
                }
                if (!bad) {
                    for (k = 0; k < 3; k++) {
                        tn[k] = strim(tn[k]);
                        if (tn[k][0] == '\0') {
                            printf("[警告] MoveC 点位名为空\n"); bad = 1;
                        } else if (strlen(tn[k]) >= sizeof(out->mc_start)) {
                            printf("[警告] MoveC 点位名过长：%s\n", tn[k]); bad = 1;
                        }
                    }
                }
                if (!bad) {
                    for (k = 3; k < 6; k++) {
                        char *s;
                        if (tn[k] == NULL) continue;
                        s = strim(tn[k]);
                        if (!parse_full_number(s, &sad[k - 3])) {
                            if (strcmp(s, "interp") == 0 || strcmp(s, "step") == 0) {
                                /* 与默认一致，忽略 */
                            } else {
                                printf("[警告] MoveC 具名后只可跟 SPD,ACC,DEC 三个数字：%s\n", s);
                                bad = 1;
                            }
                        }
                    }
                }
                if (bad) { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                snprintf(out->mc_start, sizeof(out->mc_start), "%s", tn[0]);
                snprintf(out->mc_via,   sizeof(out->mc_via),   "%s", tn[1]);
                snprintf(out->mc_to,    sizeof(out->mc_to),    "%s", tn[2]);
                out->use_point   = 1;
                out->movl_mode   = MOVL_MODE_INTERP;
                out->speeds[0]   = sad[0];
                out->accel_ms[0] = (int)sad[1];
                out->decel_ms[0] = (int)sad[2];
                out->type = CMD_MOVEC;
                return CMD_MOVEC;
            }
        }
        char *ctx = NULL;
        char *tok = strtok_r(rest, ",", &ctx);
        double v[12];
        int n = 0, i;
        while (tok != NULL && n < 12) {
            if (!parse_full_number(tok, &v[n])) {
                break;
            }
            n++;
            tok = strtok_r(NULL, ",", &ctx);
        }
        if (n != 12) {
            printf("[警告] 用法: MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC\n"
                   "       必须写满 12 段（toPoint 6 + viaPoint 3 + SPD/ACC/DEC 3）。\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        if (v[9] <= 0.0) {
            printf("[警告] MoveC 速度须大于 0 rpm（收到 %.2f）\n", v[9]);
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        if (v[10] <= 0.0 || v[11] <= 0.0) {
            printf("[警告] MoveC 加减速时间须大于 0 ms\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_MOVEC;
        out->movl_mode = MOVL_MODE_INTERP;   /* 圆弧恒为逐点插补 */
        for (i = 0; i < 6; i++) out->cartesian[i] = v[i];      /* toPoint */
        for (i = 0; i < 3; i++) out->via[i] = v[6 + i];        /* viaPoint */
        out->speeds[0]   = v[9];
        out->accel_ms[0] = (int)v[10];
        out->decel_ms[0] = (int)v[11];

        if (tok != NULL) {
            if (strcmp(tok, "interp") == 0 || strcmp(tok, "step") == 0) {
                /* 与默认一致，忽略 */
            } else if (strcmp(tok, "smooth") == 0 || strcmp(tok, "stream") == 0 ||
                       strcmp(tok, "sync") == 0) {
                printf("[警告] MoveC 只支持逐点插补（画弧本身就要分段），无 smooth/stream/sync。\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            } else {
                printf("[警告] MoveC 参数过多，用法: MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            tok = strtok_r(NULL, ",", &ctx);
            if (tok != NULL) {
                printf("[警告] MoveC 参数过多，用法: MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
        }
    } else if (ci_strcmp(cmd, "Run") == 0) {
        /* run:<文件> 或 run <文件>：轨迹脚本执行器。路径从原始行取，保留盘符冒号/空格。 */
        const char *p = arg_after_first_delim(out->raw);
        if (p[0] == '\0') {
            printf("[警告] 用法: run <脚本名或路径>  或  run:<脚本>   例 run test（自动找 tasks/test.rbt）\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        snprintf(out->run_path, sizeof(out->run_path), "%s", p);
        out->type = CMD_RUN;
    } else if (ci_strcmp(cmd, "P") == 0 || ci_strcmp(cmd, "POINT") == 0) {
        /* `P:<名>:J:a1,a2,a3,a4,a5,a6`（关节） 或 `P:<名>:X:x,y,z,a,b,c`（笛卡尔）。
         * 6 个数固定用逗号分隔；kind 取 J/X（大小写不敏）。 */
        char *s2 = NULL;
        char *name  = strim(strtok_r(save, ":", &s2));
        char *kindt = strim(strtok_r(NULL, ":", &s2));
        char *nums  = strim(strtok_r(NULL, ":", &s2));
        char *extra = strim(strtok_r(NULL, ":", &s2));
        int   kind;
        if (name == NULL || kindt == NULL || nums == NULL || extra != NULL ||
            kindt[0] == '\0' || kindt[1] != '\0') {
            printf("[警告] 用法: P:<名>:J:a1,a2,a3,a4,a5,a6  或  P:<名>:X:x,y,z,A,B,C\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        if (strlen(name) >= sizeof(out->def_name)) {
            printf("[警告] 点位名过长（≤31 字符）：%s\n", name);
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        if (kindt[0] == 'J' || kindt[0] == 'j')      kind = 0;
        else if (kindt[0] == 'X' || kindt[0] == 'x') kind = 1;
        else {
            printf("[警告] 点位类型须为 J(关节) 或 X(笛卡尔)：%s\n", kindt);
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        if (!parse_exact_nums(nums, ",", out->def_vals, 6)) {
            printf("[警告] 点位需恰好 6 个逗号分隔的数字：%s\n", nums);
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        snprintf(out->def_name, sizeof(out->def_name), "%s", name);
        out->def_kind = kind;
        out->type = CMD_DEFPOINT;
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
        /* getpos（只打印）/ getpos:j|movej[ 名字]（录关节点）/ getpos:x|movel[ 名字]（录笛卡尔点）。
         * 名字可选，写在模式之后（空格或冒号分隔）；不写则自动编号 P<n>。 */
        const char *rest = arg_after_first_delim(out->raw);
        char mode[16];
        size_t mi = 0;
        out->type = CMD_GETPOS;
        out->rec_mode = 0;
        if (rest[0] != '\0') {
            while (rest[0] != '\0' && rest[0] != ' ' && rest[0] != '\t' &&
                   rest[0] != ':' && mi + 1 < sizeof(mode)) {
                mode[mi++] = *rest++;
            }
            mode[mi] = '\0';
            while (*rest == ' ' || *rest == '\t' || *rest == ':') rest++;
            if (ci_strcmp(mode, "j") == 0 || ci_strcmp(mode, "movej") == 0 ||
                ci_strcmp(mode, "joint") == 0) {
                out->rec_mode = 1;
            } else if (ci_strcmp(mode, "x") == 0 || ci_strcmp(mode, "movel") == 0 ||
                       ci_strcmp(mode, "cart") == 0) {
                out->rec_mode = 2;
            } else {
                printf("[警告] 用法: getpos | getpos:j[ 名字] | getpos:x[ 名字]\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            if (*rest != '\0') {                  /* 名字 = 模式之后整段（去首尾空白）*/
                char *np = strim((char *)rest);
                if (strlen(np) >= sizeof(out->tp_name)) {
                    printf("[警告] 点位名过长（≤31 字符）：%s\n", np);
                    { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
                }
                snprintf(out->tp_name, sizeof(out->tp_name), "%s", np);
            }
        }
    } else if (strcmp(cmd, "cd") == 0) {
        /* 进入 .rbt 录制：cd:<文件> 或 cd <文件>；裸 cd = 退出。
         * 路径从原始行取，保留 Windows 盘符的冒号与路径中的空格。 */
        out->type = CMD_CD;
        snprintf(out->run_path, sizeof(out->run_path), "%s",
                 arg_after_first_delim(out->raw));
    } else if (strcmp(cmd, "save") == 0) {
        out->type = CMD_SAVE;
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
    } else if (strcmp(cmd, "queuetest") == 0) {
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: queuetest:关节号[:每段角度[:转速rpm]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_QUEUETEST;
        out->joint = 1;
        out->angle_deg = 5.0;
        out->speed_rpm = 60.0;
        if (a == NULL) {
            printf("[警告] 用法: queuetest:关节号[:每段角度[:转速rpm]]，例如 queuetest:1:5:60\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] queuetest 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double d;
            if (!parse_full_number(b, &d) || d <= 0.0 || d > 30.0) {
                printf("[警告] queuetest 每段角度须在 (0, 30] 度之间：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->angle_deg = d;
        }
        if (c != NULL) {
            double r;
            if (!parse_full_number(c, &r) || r <= 0.0 || r > 300.0) {
                printf("[警告] queuetest 转速须在 (0, 300] rpm 之间：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speed_rpm = r;
        }
    } else if (strcmp(cmd, "chain") == 0) {
        /* chain:关节号[:每段角度[:转速[:段数[:补发点%]]]] —— 0x00CE 流水线补链探针 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *d = strtok_r(NULL, ":", &save);
        char *e = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: chain:关节号[:每段角度[:转速[:段数[:补发点%%]]]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_CHAINTEST;
        out->joint = 1;
        out->angle_deg = 5.0;
        out->speed_rpm = 60.0;
        out->num_joints = 4;   /* 段数 */
        out->param = 50.0;     /* 补发点（本段走完百分比） */
        if (a == NULL) {
            printf("[警告] 用法: chain:关节号[:每段角度[:转速[:段数[:补发点%%]]]]，例如 chain:1:5:60:4:50\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] chain 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double deg;
            if (!parse_full_number(b, &deg) || deg <= 0.0 || deg > 30.0) {
                printf("[警告] chain 每段角度须在 (0, 30] 度之间：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->angle_deg = deg;
        }
        if (c != NULL) {
            double r;
            if (!parse_full_number(c, &r) || r <= 0.0 || r > 300.0) {
                printf("[警告] chain 转速须在 (0, 300] rpm 之间：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speed_rpm = r;
        }
        if (d != NULL) {
            double n;
            if (!parse_full_number(d, &n) || n < 1.0 || n > 8.0 ||
                n != (double)(int)n) {
                printf("[警告] chain 段数须为 1..8 的整数：%s\n", d);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->num_joints = (int)n;
        }
        if (e != NULL) {
            double p;
            if (!parse_full_number(e, &p) || p < 10.0 || p > 90.0) {
                printf("[警告] chain 补发点须在 10~90%% 之间：%s\n", e);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->param = p;
        }
    } else if (strcmp(cmd, "trigtest") == 0) {
        /* trigtest:关节号[:每段角度[:转速[:段数]]] —— 0x00DD 表格重复触发探针 */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *d = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: trigtest:关节号[:每段角度[:转速[:段数]]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_TRIGTEST;
        out->joint = 1;
        out->angle_deg = 3.0;
        out->speed_rpm = 60.0;
        out->num_joints = 6;   /* 表点数 = 触发次数 */
        if (a == NULL) {
            printf("[警告] 用法: trigtest:关节号[:每段角度[:转速[:段数]]]，例如 trigtest:1:3:60:6\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] trigtest 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double deg;
            if (!parse_full_number(b, &deg) || deg <= 0.0 || deg > 30.0) {
                printf("[警告] trigtest 每段角度须在 (0, 30] 度之间：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->angle_deg = deg;
        }
        if (c != NULL) {
            double r;
            if (!parse_full_number(c, &r) || r <= 0.0 || r > 300.0) {
                printf("[警告] trigtest 转速须在 (0, 300] rpm 之间：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speed_rpm = r;
        }
        if (d != NULL) {
            double n;
            if (!parse_full_number(d, &n) || n < 2.0 || n > 32.0 ||
                n != (double)(int)n) {
                printf("[警告] trigtest 段数须为 2..32 的整数：%s\n", d);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->num_joints = (int)n;
        }
    } else if (strcmp(cmd, "progread") == 0) {
        /* progread:关节号[:起始地址[:字数]] —— 只读转储编程区（不动臂） */
        char *a = strtok_r(NULL, ":", &save);
        char *b = strtok_r(NULL, ":", &save);
        char *c = strtok_r(NULL, ":", &save);
        char *extra = strtok_r(NULL, ":", &save);
        if (extra != NULL) {
            printf("[警告] 用法: progread:关节号[:起始地址[:字数]]\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        out->type = CMD_PROGREAD;
        out->joint = 0;
        out->angle_deg = 300.0;   /* 起始地址（编程区首地址） */
        out->speed_rpm = 32.0;    /* 字数（16位寄存器个数） */
        if (a == NULL) {
            printf("[警告] 用法: progread:关节号[:起始地址[:字数]]，例如 progread:1:300:32\n");
            { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
        }
        {
            double jn;
            if (!parse_full_number(a, &jn) || jn < 1.0 || jn > 6.0 ||
                jn != (double)(int)jn) {
                printf("[警告] progread 关节号须为 1..6 的整数：%s\n", a);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->joint = (int)jn;
        }
        if (b != NULL) {
            double ad;
            if (!parse_full_number(b, &ad) || ad < 300.0 || ad > 2047.0 ||
                ad != (double)(int)ad) {
                printf("[警告] progread 起始地址须在 300..2047：%s\n", b);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->angle_deg = ad;
        }
        if (c != NULL) {
            double wn;
            if (!parse_full_number(c, &wn) || wn < 2.0 || wn > 64.0 ||
                wn != (double)(int)wn) {
                printf("[警告] progread 字数须在 2..64：%s\n", c);
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
            out->speed_rpm = wn;
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
        char *tp = strtok_r(NULL, ":", &save);
        if (tp != NULL) {
            snprintf(out->help_topic, sizeof(out->help_topic), "%s", tp);
            if (strtok_r(NULL, ":", &save) != NULL) {
                printf("[警告] help 只接受一个主题，用法: help[:motion|enable|state|diag|probe|all]\n");
                { out->type = CMD_UNKNOWN; return CMD_UNKNOWN; }
            }
        }
        out->type = CMD_HELP;
    } else if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "quit") == 0) {
        out->type = CMD_EXIT;
    } else {
        out->type = CMD_UNKNOWN;
    }
    return out->type;
}


/* ── 分级帮助：help 打印速查索引，help:主题 打印该组详解，help:all 全展开。
 *    命名对齐 ABB RAPID，大小写不敏感。──────────────────────────────── */
static const char HELP_BRIEF[] =
    "DummyL 命令速查（大小写不敏感）  详解：help:motion / enable / state / diag / probe\n"
    "\n"
    "[运动]\n"
    "  home[:N]                    回零（全轴；:N 仅单关节）\n"
    "  MoveJ:N:ANGLE[:SPD][:r|a]   单关节运动（r=相对 a=绝对）\n"
    "  MoveJ:a1,...,a6,SPD,ACC,DEC 多关节同步（6角+3参数）\n"
    "  MoveL:X,Y,Z,Rx,Ry,Rz,...    直线（9段必填姿态，默认 interp）\n"
    "  MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC 三点式圆弧（toPoint+viaPoint+速度）\n"
    "  MoveC:<起点名>,<via名>,<终点名>[,SPD,ACC,DEC]  具名三点圆弧（需三个 X 点）\n"
    "  Run:<脚本>                    执行 tasks/<名>.rbt 轨迹脚本（# ; 注释；空行忽略）\n"
    "  TARGET <名>={J1..J6 | x,y,z,a,b,c}  定义命名点位；MoveJ:<名>/MoveL:<名> 引用\n"
    "  cd:<文件>  /  save        进入/退出 .rbt 录制；getpos:j / getpos:x 追加当前位姿为点位\n"
    "\n"
    "[使能 / 状态]\n"
    "  enable[:N]                  使能（全轴 / 关节N）\n"
    "  disable[:N]                 泄力失能（全轴 / 关节N）\n"
    "  getpos                      读当前位姿\n"
    "  motor                       电机实时监控开关\n"
    "  fk:J1,...,J6                离线正解预览（不动臂）\n"
    "  zero                        显示零点与机械角\n"
    "  zero_save:v1,...,v6         保存零点标定值\n"
    "  poseok                      解除位姿不可信闸门\n"
    "  stall[:N:MA]                堵转阈值查看/设置\n"
    "  alarm[:clear]               报警查看 / :clear 清除\n"
    "\n"
    "[总线诊断（均不动臂）]\n"
    "  diag                        总线时延体检\n"
    "  busrate[:N]                 485 极限速率实测\n"
    "  pipe[:ROUNDS[:US]]          流水线批量读探针\n"
    "  progread:N[:AD[:WORDS]]    编程区只读转储（反推指令格式）\n"
    "  bcast                       广播帧验证\n"
    "  accel[:ACC[,DEC]]           加减速时间读写（断电即失）\n"
    "  drvbaud[:CODE[:BAUD]|save]  波特率读/设/固化（⚠失联风险）\n"
    "  looptest[:N[:BAUD[:AUX]]]   转换器极限（⚠须脱离电机）\n"
    "\n"
    "[驱动器探针（⚠会真动臂）]\n"
    "  curtest[:N[:DEG[:RPM]]]     电流实测（标定堵转阈值）\n"
    "  tabtest:N[:DEG[:RPM]]       表格执行 0x00DD 试验\n"
    "  trigtest:N[:DEG[:RPM[:N]]] 表格重复触发试验\n"
    "  queuetest:N[:DEG[:RPM]]     排队寄存器 0x00CE 试验\n"
    "  chain:N[:DEG[:RPM]]         补链排队试验（段数见 help:probe）\n"
    "\n"
    "[系统]\n"
    "  help[:topic]                本一览；motion/enable/state/diag/probe/all\n"
    "  exit                        退出\n";

static const char HELP_MOTION[] =
    "【motion 运动】\n"
    "  home                  回零（全轴）\n"
    "  home:N                仅单独回零关节 N，堵转后自动到该轴配置角\n"
    "  MoveJ:N:ANGLE[:SPD][:r|a]   单关节关节空间运动：轴N 至角度ANGLE(度)，\n"
    "                          速度SPEED(rpm)，末段 r=相对当前位置 / a=绝对(默认)\n"
    "  MoveJ:ANG1,ANG2,ANG3,ANG4,ANG5,ANG6,SPD,ACC,DEC   多关节同步关节空间运动\n"
    "  MoveL:X,Y,Z,Rx,Ry,Rz,SPD,ACC,DEC   笛卡尔直线（唯一格式，9 段写满）\n"
    "                          SPD=rpm、ACC/DEC=ms；Rx,Ry,Rz 抄 getpos 的当前姿态\n"
    "  MoveL:...,DEC           默认=interp 逐点插补（末端贴直线、逐段保护；段间有加减速停顿，较慢）\n"
    "  MoveL:...,DEC,smooth    显式加 ,smooth 走流畅：终点一次 MoveJ、段间零停顿，但末端走弧、不查过流\n"
    "                          ★ 写法约定：\n"
    "                          ① 只接受 9 段（含显式姿态 Rx,Ry,Rz）；原 3/6 段简写及 keep 后缀均已移除\n"
    "                          ② 9 段若姿态与当前差太多，会警告\"拧姿态、笔尖偏离\"\n"
    "                             （实测偏 7.71mm），但仍照走；想保直线就抄 getpos 原值\n"
    "                          ③ 模式 interp（默认）：按 ini [movel] max_bow_mm（偏差预算）反推步长、逐航点下发并等到位\n"
    "                             ⇒ 末端贴着直线 + 每段读过流/超时保护；代价是段间加减速停顿\n"
    "                             预算 = [movel] min_bow_mm~max_bow_mm 的【弓高中点】(折中)；想减航点/减停顿就把两值整体抬高\n"
    "                          ④ 模式 smooth（须显式加 ,smooth）：终点一次 IK + 一次 MoveJ ⇒ 零段间停顿；\n"
    "                             但末端走弧（弓高 = 整段）且全程不查过流\n"
    "                          ⑤ stream / sync 已移除，写了会被拒绝\n"
    "  MoveC:X,Y,Z,Rx,Ry,Rz,VX,VY,VZ,SPD,ACC,DEC   三点式空间圆弧（对齐 ABB MoveC）\n"
    "                          起点=当前位姿；toPoint=(X,Y,Z,Rx,Ry,Rz)（同 MoveL，姿态显式）；\n"
    "                          viaPoint=(VX,VY,VZ) 决定弧的凸向与半径（参数顺序：toPoint→viaPoint→速度）\n"
    "                          ⇒ 过三点定圆、沿真实圆弧逐点 IK+逐段下发（同 interp 直线那套保护/护栏）\n"
    "                          恒为 interp（圆弧本就靠分段才画得出）；无 smooth/stream/sync，写了会被拒绝\n"
    "                          三点近共线时自动退化为直线（会提示）；步长/弓高预算沿用 [movel] 配置\n"
    "                          画整圆：连发两段 MoveC（各翻半圈，第一段的 toPoint 作中间接缝点）\n"
    "\n"
    "【命名点位（.rbt 轨迹脚本）】\n"
    "  TARGET <名>={J1:v J2:v J3:v J4:v J5:v J6:v}   定义关节点位（6 个机械角/度）\n"
    "  TARGET <名>={x:v y:v z:v a:v b:v c:v}    定义笛卡尔点位（mm + 姿态角）\n"
    "  MoveJ:<名>[:SPD,ACC,DEC]     运动到关节点位（只能引 J 点；SPD 缺省 60rpm）\n"
    "  MoveL:<名>[:SPD,ACC,DEC]     直线到笛卡尔点位（只能引 X 点；起点=当前位姿）\n"
    "                          点位只在当前进程/脚本内有效（重名则覆盖）；配合 Run:<名> 跑整段\n"
    "\n"
    "【.rbt 示教录制（把当前位姿写进文件）】\n"
    "  cd:<文件>              进入录制（如 cd:test → tasks/test.rbt；不存在则新建）\n"
    "  getpos:j [名字]      读当前六轴机械角，追加 TARGET <名字>={J1:.. J6:..}（不写名字自动 P<n>）\n"
    "  getpos:x [名字]      读当前末端位姿，追加 TARGET <名字>={x:.. y:.. z:.. a:.. b:.. c:..}\n"
    "                          名字不能与已有点位重名（重名报冲突）；自动编号也会避开已用 P<n>\n"
    "  save                   退出录制（裸 cd 也可退出）\n"
    "                          点位号从文件已有 P<数字> 的最大值+1 继续；引用用 MoveJ:P1 / MoveL:P2\n"
    "                          分隔符 ':' 与空格都可：`cd test` / `getpos movej` / `run test`\n";

static const char HELP_ENABLE[] =
    "【enable 使能/泄力】\n"
    "  disable               全部失能所有关节\n"
    "  disable:N             仅单独泄力(失能)关节 N\n"
    "  enable                恢复使能所有关节\n"
    "  enable:N              仅单独使能关节 N\n";

static const char HELP_STATE[] =
    "【state 状态/标定】\n"
    "  getpos                读取当前关节角(度)、笛卡尔坐标(X,Y,Z,RPY)与法兰倾角\n"
    "  motor                 启动/停止电机实时监控（1S/次循环显示）\n"
    "  fk:J1,J2,J3,J4,J5,J6  离线正解预览：按 DH 表算出该组关节角对应的位姿、\n"
    "                        法兰倾角与臂形；不动臂不下发，用来和 getpos 对照\n"
    "  stall                 显示六轴堵转阈值(mA)与各轴最新电流\n"
    "  stall:N:MA            运行时设置关节N的堵转阈值为 MA mA（0=关闭该轴）；\n"
    "                        只改本次运行，持久化请写 ini [stall] 的 j1..j6\n"
    "  poseok                人工解除「位姿不可信」闸门（零点丢失时运动命令会被锁住；\n"
    "                        确认是误判才用，否则所有位姿与运动都是错的）\n"
    "  zero                  显示当前零点与机械角\n"
    "  zero_save:v1,v2,v3,v4,v5,v6  保存指定的零点标定值\n"
    "  alarm                 查看六轴驱动器报警（0x00A3：当前 + 最近三次历史）\n"
    "  alarm:clear           清除六轴报警（0x00A4=0）；⚠️ 先排除原因，否则立刻复现\n";

static const char HELP_DIAG[] =
    "【diag 总线/时延/波特率（均不动臂，除非注明）】\n"
    "  diag                  总线时延体检：把单事务拆成 flush/write/read 三段计时，\n"
    "                        并对照\"只写不读\"，定位刷新率瓶颈；不动臂\n"
    "  busrate[:N]           总线极限速率实测（回答\"这根485最高多少Hz\"）：分档给出\n"
    "                        单轴 读/写+等响应/只写不等响应、六轴一轮(并线) 的 ms 与 Hz，\n"
    "                        并给出\"拆成6根独立总线\"的等效更新率；不动臂（探针写回原速度）\n"
    "  accel[:ACC[,DEC]]     驱动器加减速时间读写（0x0098 加速 / 0x0099 减速，出厂120/120 ms）\n"
    "                        无参数=只读显示；accel:40 或 accel:40,40 = 六轴写入并读回验证。\n"
    "                        只写 RAM（不发 0x00DC），驱动器断电即恢复，可反复试。\n"
    "                        ⚠️ 调太小会丢步/过流，从 40 起试；这是\"顿\"里唯一免改硬件的部分\n"
    "  pipe[:轮数[:间隔us]]   流水线批量读探针（只读位置，不动臂）：\n"
    "                        基线＝现状一问一答；然后先连发 6 个请求、再收 6 个响应，\n"
    "                        扫 0/50/100/200/300/500/1000/2000 us 八档间隔，报丢帧数。\n"
    "                        目的：把\"等响应那 1.6ms 里总线其实是空的\"这块钱榨出来。\n"
    "                        轮数默认 20；给第三个参数只测那一档（如 pipe:30:200）\n"
    "  bcast                 广播帧验证：地址0写速度再逐轴读回，看几轴响应广播\n"
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
    "                        判读：单事务耗时看 diag（921600 下 ≈1.7ms），它 = 线上 0.23\n"
    "                              + 驱动器周转 ≈1.47 + USB栈 ≤0.08 ms\n"
    "                              ⚠️ 回环RTT只代表本工具的往返能力，【不能】从单事务里减\n"
    "                        ⚠️ 必须脱离电机；接电机时切BAUD会立刻失联\n";

static const char HELP_PROBE[] =
    "【probe 驱动器寄存器探针（⚠️ 会真动臂，做\"又直又顺\"判定/电流标定用）】\n"
    "  curtest               电流实测（标定堵转阈值用）：静止采样六轴保持电流，不动臂\n"
    "  curtest:N[:DEG[:RPM]] 关节N 走 +DEG 度再走回原位，全程高速采样该轴电流，\n"
    "                        输出 保持/运动 的最小·均值·最大(mA) 与建议阈值区间\n"
    "  tabtest:N[:DEG[:RPM]] 驱动器【表格数据】验证（手册第54节）：往关节N 写 3 段\n"
    "                        相对位移表，让驱动器自己连续执行，全程高频读 0x00D6\n"
    "                        实时速度 ⇒ 看段间速度掉不掉 0（决定\"又直又顺\"能否成立）\n"
    "                        DEG=每段机械角(默认5,上限30)；轴会停在 +3×DEG 处，注意行程\n"
    "  trigtest:N[:DEG[:RPM[:段数]]]  表格重复触发探针（tabtest 的盲区）：写一张 N 点\n"
    "                        相同相对位移的表后，逐次写 0x00DD 触发、每次等本段走完，\n"
    "                        再跟一轮不等走完的连发 ⇒ 验两件事：①表指针是否每次+1（一次\n"
    "                        触发只吃一个点但自动前进）②运行中再触发会被强制重开还是丢帧\n"
    "                        （成立 ⇒ interp 可改「预下载航点表+廉价触发帧」大幅减开销）\n"
    "                        DEG 默认3° RPM 默认60 段数默认6(2..32)；停在 +段数×DEG 处\n"
    "  progread:N[:起始[:字数]]   编程区只读转储（不动臂）：FC03 读地址 300+ 的存贮内容，\n"
    "                        出厂演示程序若还在区内，可从中反推编程指令格式（0x00DB 连走\n"
    "                        全靠它）⇒ 纯读零风险；起始默认 300，字数默认 32(上限 64)\n"
    "  queuetest:N[:DEG[:RPM]] 排队寄存器 0x00CE 探针（手册§66④）：向关节N 连发 3 段\n"
    "                        相对位移（段间不等到位），全程高频读 0x00D6 实时速度\n"
    "                        ⇒ 看排队跑多段时段间速度掉不掉 0（掉 0=仅串行、仍卡；\n"
    "                        不掉=驱动器会混合 ⇒ interp 改走 0x00CE 可消卡顿）\n"
    "                        并只读采集运行模式(0x009F)/动态定位(0x00B6)当前值\n"
    "                        DEG=每段机械角(默认5,上限30)；轴会停在 +3×DEG 处，注意行程\n"
    "  chain:N[:DEG[:RPM[:段数[:补发点%]]]]  0x00CE 流水线补链探针：本段走到【补发点%】\n"
    "                        才下发下一段（而不是起步就连发），全程高频读实时速度\n"
    "                        ⇒ 判段间速度掉不掉 0：不掉 ⇒ interp 可改补链模式消卡顿；\n"
    "                        掉 0 但段数跑齐 ⇒ 排队可行但每段仍停；段数不够 ⇒ 补发被丢弃\n"
    "                        段数默认 4(上限8)；补发点默认 50%；轴会停在 +段数×DEG 处\n";

/* 分级帮助打印（help / ?）。topic 为空 → 索引；all → 全展开；其余 → 单组。 */
void cmd_print_help(const char *topic)
{
    if (topic == NULL || topic[0] == '\0') {
        fputs(HELP_BRIEF, stdout);
        return;
    }
    if (strcmp(topic, "all") == 0) {
        fputs(HELP_MOTION, stdout);  fputs(HELP_ENABLE, stdout);
        fputs(HELP_STATE, stdout);   fputs(HELP_DIAG, stdout);
        fputs(HELP_PROBE, stdout);
        printf("  exit                  退出\n");
        return;
    }
    if (strcmp(topic, "motion") == 0) { fputs(HELP_MOTION, stdout); return; }
    if (strcmp(topic, "enable") == 0) { fputs(HELP_ENABLE, stdout); return; }
    if (strcmp(topic, "state")  == 0) { fputs(HELP_STATE,  stdout); return; }
    if (strcmp(topic, "diag")   == 0) { fputs(HELP_DIAG,   stdout); return; }
    if (strcmp(topic, "probe")  == 0) { fputs(HELP_PROBE,  stdout); return; }
    printf("未知主题「%s」。可用：motion / enable / state / diag / probe / all\n", topic);
    fputs(HELP_BRIEF, stdout);
}