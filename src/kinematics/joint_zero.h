#ifndef JOINT_ZERO_H
#define JOINT_ZERO_H

#define JOINT_ZERO_COUNT 6

void joint_zero_motor_to_mech(const double *q_motor, double *q_mech);

void joint_zero_mech_to_motor(const double *q_mech, double *q_motor);

const double *joint_zero_get(void);

void joint_zero_save(const double *new_zero);

void joint_zero_reset(void);

#endif
