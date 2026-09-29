#ifndef ZERO_H
#define ZERO_H

#define ZERO_JOINT_COUNT 6

/* 电机角 → 机械角（减 q0 零点）。CLI 对外一律用机械角。 */
void zero_motor_to_mech(const double *q_motor, double *q_mech);

/* 机械角 → 电机角（加 q0 零点）。下发前用这个。 */
void zero_mech_to_motor(const double *q_mech, double *q_motor);

/* 取当前六轴零点数组（只读）。 */
const double *zero_get(void);

/* 更新零点（仅内存；写 ini 由 cmd_zero_save 负责）。 */
void zero_save(const double *new_zero);

/* 复位成编译期默认 ROBOT_JOINT_ZERO_DEG。 */
void zero_reset(void);

#endif
