/*
 * joint_zero.c —— 上位机电机角 ↔ 机械角 换算
 * ------------------------------------------------------------
 * 所属模块：运动学（kinematics）
 * 对外接口：joint_zero_motor_to_mech、joint_zero_mech_to_motor
 * 依赖模块：config/robot_config.h
 */

#include "kinematics/joint_zero.h"
#include "config/robot_config.h"

static double g_joint_zero[JOINT_ZERO_COUNT] = ROBOT_JOINT_ZERO_DEG;
static int g_zero_overridden = 0;

const double *joint_zero_get(void)
{
    return g_joint_zero;
}

void joint_zero_save(const double *new_zero)
{
    for (int i = 0; i < JOINT_ZERO_COUNT; i++) {
        g_joint_zero[i] = new_zero[i];
    }
    g_zero_overridden = 1;
}

void joint_zero_reset(void)
{
    static const double default_zero[JOINT_ZERO_COUNT] = ROBOT_JOINT_ZERO_DEG;
    for (int i = 0; i < JOINT_ZERO_COUNT; i++) {
        g_joint_zero[i] = default_zero[i];
    }
    g_zero_overridden = 0;
}

void joint_zero_motor_to_mech(const double *q_motor, double *q_mech)
{
    int i;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        q_mech[i] = q_motor[i] - g_joint_zero[i];
    }
}

void joint_zero_mech_to_motor(const double *q_mech, double *q_motor)
{
    int i;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        q_motor[i] = q_mech[i] + g_joint_zero[i];
    }
}
