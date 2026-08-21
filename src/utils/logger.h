#ifndef LOGGER_H
#define LOGGER_H

/*
 * 精简中文日志（纯 C）
 * 级别: TRACE/DEBUG/INFO/WARN/ERROR
 * 输出格式: [级别] 消息
 * Windows 控制台中文显示需 UTF-8 代码页（chcp 65001）或终端支持。
 */

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_WARN  3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_NONE  5

/* 设置全局日志级别（默认 INFO） */
void log_set_level(int level);

int  log_get_level(void);

void log_msg(int level, const char *fmt, ...);

#define LOG_TRACE(...) log_msg(LOG_LEVEL_TRACE, __VA_ARGS__)
#define LOG_DEBUG(...) log_msg(LOG_LEVEL_DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  log_msg(LOG_LEVEL_INFO,  __VA_ARGS__)
#define LOG_WARN(...)  log_msg(LOG_LEVEL_WARN,  __VA_ARGS__)
#define LOG_ERROR(...) log_msg(LOG_LEVEL_ERROR, __VA_ARGS__)

#endif /* LOGGER_H */
