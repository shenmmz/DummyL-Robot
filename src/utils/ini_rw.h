#ifndef INI_RW_H
#define INI_RW_H

#include <stddef.h>

/* 配置 ini 默认路径（CWD 相对，须在项目根目录运行）。
 * 同时被 main.c 与 cli/commands.c 引用，集中定义避免重复。 */
#ifndef INI_PATH
#define INI_PATH "src/config/robot_config.ini"
#endif

/* 读取 ini 的 [joint_zero] 段，填充 zero[6]（key 为 q0..q5）。
 * 找到且 6 个全部解析成功返回 1，否则返回 0（不修改 zero）。 */
int ini_read_joint_zero(const char *path, double zero[6]);

/* 写回 ini 的 [joint_zero] 段（q0..q5），保留 [serial] 等其它内容与注释。
 * 成功返回 1，失败（无法打开写）返回 0。 */
int ini_write_joint_zero(const char *path, const double zero[6]);

#endif /* INI_RW_H */
