
#include "kinematics/joint_zero.h"
#include "config/robot_config.h"

static double g_joint_zero[JOINT_ZERO_COUNT] = ROBOT_JOINT_ZERO_DEG;
static int g_zero_overridden = 0;

/* 取当前零点表 q0[6]。约定：机械角 = 电机角 − q0。 */
const double *joint_zero_get(void)
{
    return g_joint_zero;
}

/* 写入新零点（内存 + 落盘 ini [joint_zero]）。 */
void joint_zero_save(const double *new_zero)
{
    for (int i = 0; i < JOINT_ZERO_COUNT; i++) {
        g_joint_zero[i] = new_zero[i];
    }
    g_zero_overridden = 1;
}

/* 恢复内置默认零点。 */
void joint_zero_reset(void)
{
    static const double default_zero[JOINT_ZERO_COUNT] = ROBOT_JOINT_ZERO_DEG;
    for (int i = 0; i < JOINT_ZERO_COUNT; i++) {
        g_joint_zero[i] = default_zero[i];
    }
    g_zero_overridden = 0;
}

/* 电机角 → 机械角：q_mech = q_motor − q0。 */
void joint_zero_motor_to_mech(const double *q_motor, double *q_mech)
{
    int i;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        q_mech[i] = q_motor[i] - g_joint_zero[i];
    }
}

/* 机械角 → 电机角：q_motor = q_mech + q0。用户面一律机械角，下发前用这个转。 */
void joint_zero_mech_to_motor(const double *q_mech, double *q_motor)
{
    int i;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        q_motor[i] = q_mech[i] + g_joint_zero[i];
    }
}
