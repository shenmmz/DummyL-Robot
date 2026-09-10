/*
 * joint_zero.c —— 上位机电机角 ↔ 机械角 换算
 * ------------------------------------------------------------
 * 所属模块：运动学（kinematics）
 * 对外接口：joint_zero_motor_to_mech、joint_zero_mech_to_motor
 * 依赖模块：config/robot_config.h
 */

#include "kinematics/joint_zero.h"
#include "config/robot_config.h"

static const double JOINT_ZERO[JOINT_ZERO_COUNT] = ROBOT_JOINT_ZERO_DEG;

void joint_zero_motor_to_mech(const double *q_motor, double *q_mech)
{
    int i;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        q_mech[i] = q_motor[i] - JOINT_ZERO[i];
    }
}

void joint_zero_mech_to_motor(const double *q_mech, double *q_motor)
{
    int i;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        q_motor[i] = q_mech[i] + JOINT_ZERO[i];
    }
}
