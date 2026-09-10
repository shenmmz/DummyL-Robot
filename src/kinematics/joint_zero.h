#ifndef JOINT_ZERO_H
#define JOINT_ZERO_H

/*
 * 关节零点标定：上位机电机角 ↔ 机械角 的双向换算
 * ------------------------------------------------------------
 * 所属模块：运动学（kinematics）
 * 对外接口：joint_zero_motor_to_mech、joint_zero_mech_to_motor
 * 依赖模块：config/robot_config.h（ROBOT_JOINT_ZERO_DEG）
 *
 * 两套角度的由来：
 *   上位机电机角 —— status 显示、movej 输入的角度，零点在【回零硬限位】；
 *   机械角       —— DH 表与 FK/IK 使用的角度，零点在【机械设计零位】。
 * 两者只差一个常数（见 config/robot_config.h 的标定说明）。
 *
 * 约定（2026-09-10 定）：【用户面一律机械角】——status 显示、movej/homej 输入、
 *       fk/ik 输入输出都是机械角，与通用机械臂/示教器口径一致（机械零位为 0）。
 *       电机角只出现在两个最底层：robot_movej 下发前、位置回读换算时，
 *       这两处由 control/robot.c 调用本模块完成转换，上层无需关心。
 */

#define JOINT_ZERO_COUNT 6

/* 电机角 → 机械角（数组长度 JOINT_ZERO_COUNT） */
void joint_zero_motor_to_mech(const double *q_motor, double *q_mech);

/* 机械角 → 电机角（数组长度 JOINT_ZERO_COUNT） */
void joint_zero_mech_to_motor(const double *q_mech, double *q_motor);

#endif /* JOINT_ZERO_H */
