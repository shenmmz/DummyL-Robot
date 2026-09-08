#ifndef HOME_H
#define HOME_H

/*
 * home.h —— 堵转模式回零模块接口
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：robot_home
 * 依赖模块：control/robot（内部接口）、comm/modbus_rtu、config/robot_config、utils/logger
 *
 * 回零流程（defer 分阶段编排）：
 *   阶段1 {1,2,3,5} 并行堵转归零（仅清零） → 阶段2 关节6 传感器回零
 *   → 阶段3 关节4 单独堵转归零 → 阶段4 统一 movej 到各轴 forward_deg
 * 6 轴为 IN0/IN1 传感器回零轴（非堵转），支持 3 种初始情况（见 home_joint6）。
 */

#include "control/robot.h"

/* 堵转模式回零：分组并行堵转回零，完成后统一运动到 forward_deg 位姿。
 * 返回 ErrCode：ERR_NONE 成功，失败返回对应错误码（超时 ERR_TIMEOUT） */
ErrCode robot_home(Robot *robot);

/* 单轴回零后自动运动到指定角度：只回零关节 joint（其余轴临时屏蔽），
 * 回零成功后自动 movej 到 angle_deg（度）；speed_rpm<=0 时使用回零速度。
 * 返回 ErrCode：ERR_NONE 成功 / ERR_ARG / ERR_MASKED（目标轴被屏蔽）/ ERR_TIMEOUT */
ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 单轴独立回零（home:N）：仅操作关节 joint，不触碰/不依赖其它轴。
 * 1..5：堵转流程（arm → 堵转 → 电流超阈值 → 清零 → movej 到 forward_deg）；
 * 6   ：IN0/IN1 传感器状态机（3 种初始情况，见 home.c home_joint6）。
 * 返回 ErrCode：ERR_NONE 成功 / ERR_ARG / ERR_MASKED（目标轴被屏蔽）/ ERR_TIMEOUT */
ErrCode robot_home_single(Robot *robot, int joint);

/* 力矩碰撞回原点诊断（torque:N:L）：关节 joint 以等级 level(0~255) 启动碰撞回原点，
 * 每 200ms 打印状态字/电流/位置，检测到 bit15(HOMED) 或退出 RUN_ACTIVE 即停并收尾。
 * 仅观察、不清零/转角，用于标定力矩等级与确认到位判据。返回 ErrCode。 */
ErrCode robot_torque_probe(Robot *robot, int joint, int level);

#endif /* HOME_H */
