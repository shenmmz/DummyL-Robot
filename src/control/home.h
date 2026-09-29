#ifndef HOME_H
#define HOME_H


#include "control/robot.h"

/* 六轴全量回零（堵转清零）。 */
ErrCode robot_home(Robot *robot);

/* 单轴回零到指定机械角。 */
ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 单轴堵转回零（不动其他轴）。 */
ErrCode robot_home_single(Robot *robot, int joint);

/* 力矩碰撞回零诊断：给定力矩等级跑一次，每 200ms 打一行。 */
ErrCode robot_torque_probe(Robot *robot, int joint, int level);

#endif
