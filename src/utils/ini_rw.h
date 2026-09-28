#ifndef INI_RW_H
#define INI_RW_H

#include <stddef.h>

#ifndef INI_PATH
#define INI_PATH "src/config/robot_config.ini"
#endif

/* 读 [joint_zero] 的六轴零点。返回 1 = 成功，0 = 失败（调用方保留默认值）。 */
int ini_read_joint_zero(const char *path, double zero[6]);

/* 写 [joint_zero] 六轴零点（`zero:save`）。返回 1/0。 */
int ini_write_joint_zero(const char *path, const double zero[6]);

/* 读 [tool] tool_length（mm）。缺键时 *tool_mm 保持 0。 */
int ini_read_tool_length(const char *path, double *tool_mm);

/* 读 [tool] pen_length（mm）。实测标定为 41.17。 */
int ini_read_pen_length(const char *path, double *pen_mm);

/* 读 [stall] 六轴堵转电流阈值（mA）。
 * ⚠️ 阈值疑似偏低：实测静止电流 ~500/499/495/385/371/254 vs 阈值 480/490/480/400/390。 */
int ini_read_stall_current(const char *path, int th[6]);

/* 读 [safety] max_step_deg（单次运动位移上限，度）。 */
int ini_read_max_step_deg(const char *path, double *deg);

/* 通用读正数 double（读不到或 <=0 都算失败）。 */
int ini_read_positive_double(const char *path, const char *section,
                             const char *key, double *out);

/* 读 [safety] max_jump_deg（单个插补段内单关节跳变上限，度）。 */
int ini_read_max_jump_deg(const char *path, double *deg);

#endif
