/*
 * telemetry.c —— UDP 遥测：把关节角发给本机上的 3D 镜像（sim/live_mirror.py）
 * ---------------------------------------------------------------------------
 * 为什么用 UDP 回环而不是写文件：
 *   控制循环一轮六轴 92.8ms（10.8Hz），本来就被总线延迟占满（等响应占 98.6%）。
 *   在循环里碰磁盘 = 引入不可预测的 I/O 抖动，代价是实打实的运动质量。
 *   sendto 到 127.0.0.1 是微秒级、非阻塞的，而且【没人监听时内核直接丢包】，
 *   不连 Python 也不会有任何副作用。
 *
 * 为什么失败必须静默：
 *   遥测是"看着舒服"的功能，不是"必须成功"的功能。WSAStartup 失败、端口被占、
 *   网络栈异常 —— 任何一种都不许影响机械臂控制。所以所有错误路径只设
 *   g_disabled=1，然后永远闭嘴，不打印、不报错、不重试。
 *
 * 包格式（一行 CSV，UTF-8，无中文）：
 *   <tag>,<t_ms>,<q1>,<q2>,<q3>,<q4>,<q5>,<q6>\n
 *   tag = cmd（本次下发的目标角）/ act（读回的实际角）
 */
#include "utils/telemetry.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TLM_DEFAULT_PORT 9900

static int g_ready = 0;      /* 1 = 已初始化且可用 */
static int g_disabled = 0;   /* 1 = 明确不可用，不再尝试（静默） */
static int g_port = TLM_DEFAULT_PORT;

#ifdef _WIN32
static SOCKET g_sock = INVALID_SOCKET;
#else
static int g_sock = -1;
#endif
static struct sockaddr_in g_dst;

/* ini 里 [telemetry] 段的一个数值项。读不到就返回 def。 */
static double tlm_ini_double(const char *path, const char *key, double def)
{
    FILE *f;
    char line[256];
    int in_sec = 0;

    f = fopen(path, "r");
    if (f == NULL) return def;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_sec = (strncmp(p, "[telemetry]", 11) == 0) ? 1 : 0;
            continue;
        }
        if (!in_sec) continue;
        if (strncmp(p, key, strlen(key)) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        return atof(eq + 1);
    }
    fclose(f);
    return def;
}

void telemetry_init(const char *ini_path)
{
    double enabled, port;

    if (g_ready || g_disabled) return;

    enabled = tlm_ini_double(ini_path, "enabled", 0.0);
    if (enabled <= 0.0) {          /* 默认关：不显式打开就不碰网络栈 */
        g_disabled = 1;
        return;
    }
    port = tlm_ini_double(ini_path, "port", (double)TLM_DEFAULT_PORT);
    if (port < 1.0 || port > 65535.0) port = (double)TLM_DEFAULT_PORT;
    g_port = (int)port;

#ifdef _WIN32
    {
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { g_disabled = 1; return; }
    }
    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock == INVALID_SOCKET) { g_disabled = 1; WSACleanup(); return; }
    {
        /* 发送超时 1ms：宁可丢这一包，也不许卡住控制循环 */
        int timeout_ms = 1;
        setsockopt(g_sock, SOL_SOCKET, SO_SNDTIMEO,
                   (const char *)&timeout_ms, sizeof(timeout_ms));
    }
#else
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) { g_disabled = 1; return; }
#endif

    memset(&g_dst, 0, sizeof(g_dst));
    g_dst.sin_family = AF_INET;
    g_dst.sin_port = htons((unsigned short)g_port);
#ifdef _WIN32
    g_dst.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
#else
    g_dst.sin_addr.s_addr = inet_addr("127.0.0.1");
#endif

    g_ready = 1;
}

void telemetry_send(const double deg[6], const char *tag)
{
    char buf[256];
    int n;

    if (!g_ready || deg == NULL) return;

    n = snprintf(buf, sizeof(buf), "%s,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                 (tag != NULL) ? tag : "cmd",
                 (unsigned long)clock(),
                 deg[0], deg[1], deg[2], deg[3], deg[4], deg[5]);
    if (n <= 0 || n >= (int)sizeof(buf)) return;

    /* 返回值一律不看：丢包/失败都无所谓，遥测不是关键路径 */
    (void)sendto(g_sock, buf, n, 0,
                 (const struct sockaddr *)&g_dst, sizeof(g_dst));
}

void telemetry_shutdown(void)
{
    if (!g_ready) return;
#ifdef _WIN32
    closesocket(g_sock);
    WSACleanup();
#else
    close(g_sock);
#endif
    g_ready = 0;
    g_disabled = 1;
}
