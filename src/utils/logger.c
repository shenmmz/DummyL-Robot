/*
 * logger.c —— 分级日志输出工具（DEBUG/INFO/WARN/ERROR）
 * ------------------------------------------------------------
 * 所属模块：工具层（utils）
 * 对外接口：log_set_level、log_get_level、log_msg
 * 依赖模块：无
 */

#include "utils/logger.h"

#include <stdarg.h>
#include <stdio.h>

static int g_level = LOG_LEVEL_INFO;

/* log_set_level：设置全局日志输出级别 */
void log_set_level(int level)
{
    g_level = level;
}

/* log_get_level：获取当前日志级别 */
int log_get_level(void)
{
    return g_level;
}

/* log_msg：按级别输出一行日志，低于当前级别则丢弃 */
void log_msg(int level, const char *fmt, ...)
{
    static const char *tag[] = {"追踪", "调试", "信息", "警告", "错误"};
    va_list ap;

    if (level < g_level || level < LOG_LEVEL_TRACE || level > LOG_LEVEL_ERROR) {
        return;
    }
    printf("[%s] ", tag[level]);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}
