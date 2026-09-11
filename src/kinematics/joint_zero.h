#ifndef JOINT_ZERO_H
#define JOINT_ZERO_H

#define JOINT_ZERO_COUNT 6

/* 电机角 → 机械角（数组长度 JOINT_ZERO_COUNT） */
void joint_zero_motor_to_mech(const double *q_motor, double *q_mech);

/* 机械角 → 电机角（数组长度 JOINT_ZERO_COUNT） */
void joint_zero_mech_to_motor(const double *q_mech, double *q_motor);

/* 获取当前零点标定值（运行时覆盖或默认） */
const double *joint_zero_get(void);

/* 保存新的零点标定值（运行时覆盖） */
void joint_zero_save(const double *new_zero);

/* 重置为编译期默认值 */
void joint_zero_reset(void);

#endif /* JOINT_ZERO_H */
