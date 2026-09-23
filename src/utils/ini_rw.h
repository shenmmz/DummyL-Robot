#ifndef INI_RW_H
#define INI_RW_H

#include <stddef.h>

#ifndef INI_PATH
#define INI_PATH "src/config/robot_config.ini"
#endif

int ini_read_joint_zero(const char *path, double zero[6]);

int ini_write_joint_zero(const char *path, const double zero[6]);

int ini_read_tool_length(const char *path, double *tool_mm);

int ini_read_pen_length(const char *path, double *pen_mm);

int ini_read_stall_current(const char *path, int th[6]);

int ini_read_max_step_deg(const char *path, double *deg);

int ini_read_positive_double(const char *path, const char *section,
                             const char *key, double *out);

int ini_read_max_jump_deg(const char *path, double *deg);

#endif
