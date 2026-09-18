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

/* 读取 ini 的 [tool] 段，填充 *tool_mm（key 为 tool_length，单位 mm）。
 * 找到且解析成功返回 1，否则返回 0（调用方按默认 0 处理）。 */
int ini_read_tool_length(const char *path, double *tool_mm);

/* 读取 ini 的 [stall] 段，填充 th[6]（key 为 j1..j6，单位 mA，0=该轴不检测）。
 * 6 个全部找到且解析成功返回 1，否则返回 0（不修改 th）。
 * 允许缺段/缺项：调用方拿到 0 就按编译期默认值处理，不要当成致命错误。 */
int ini_read_stall_current(const char *path, int th[6]);

/* 读取 ini 的 [safety] 段的 max_step_deg（单次下发位移上限，机械角度）。
 * 找到且解析成功（>0）返回 1，否则返回 0（调用方按编译期默认值处理）。
 *
 * 这是"目标离当前位置太远 ⇒ 拒绝下发"闸门的阈值。允许缺段/缺项 ——
 * 拿不到就回退默认值，不要因为少配一行就让运动指令全废。 */
int ini_read_max_step_deg(const char *path, double *deg);

#endif /* INI_RW_H */
