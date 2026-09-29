
#include "kinematics/zero.h"
#include "config/robot_config.h"

static double g_zero[ZERO_JOINT_COUNT] = ROBOT_JOINT_ZERO_DEG;
static int g_zero_overridden = 0;

/* 取当前零点表 q0[6]。约定：机械角 = 电机角 − q0。 */
const double *zero_get(void)
{
    return g_zero;
}

/* 写入新零点（仅内存；落盘 ini 由调用方 cmd_zero_save 负责）。 */
void zero_save(const double *new_zero)
{
    int i;
    for (i = 0; i < ZERO_JOINT_COUNT; i++) {
        g_zero[i] = new_zero[i];
    }
    g_zero_overridden = 1;
}

/* 恢复内置默认零点。 */
void zero_reset(void)
{
    static const double default_zero[ZERO_JOINT_COUNT] = ROBOT_JOINT_ZERO_DEG;
    int i;
    for (i = 0; i < ZERO_JOINT_COUNT; i++) {
        g_zero[i] = default_zero[i];
    }
    g_zero_overridden = 0;
}

/* 电机角 → 机械角：q_mech = q_motor − q0。 */
void zero_motor_to_mech(const double *q_motor, double *q_mech)
{
    int i;
    for (i = 0; i < ZERO_JOINT_COUNT; i++) {
        q_mech[i] = q_motor[i] - g_zero[i];
    }
}

/* 机械角 → 电机角：q_motor = q_mech + q0。用户面一律机械角，下发前用这个转。 */
void zero_mech_to_motor(const double *q_mech, double *q_motor)
{
    int i;
    for (i = 0; i < ZERO_JOINT_COUNT; i++) {
        q_motor[i] = q_mech[i] + g_zero[i];
    }
}
