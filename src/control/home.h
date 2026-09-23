#ifndef HOME_H
#define HOME_H


#include "control/robot.h"

/* 六轴全量回零（堵转清零）。⚠️ 读回来的必然是定义值、零信息量。
 * 真零点靠人工对齐关节凹槽（对齐后应显示 0,0,90,0,0,0）。 */
ErrCode robot_home(Robot *robot);

/* 单轴回零到指定机械角。
 * ⚠️ 已知缺陷：stall[6].speed_rpm 传 0 ⇒ J6 会干等 20s（待修）。 */
ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 单轴堵转回零（不动其他轴）。 */
ErrCode robot_home_single(Robot *robot, int joint);

/* 力矩碰撞回零【诊断】：给定力矩等级跑一次，每 200ms 打一行
 * `T+ms 状态字 电流mA 位置 备注`，最多 15s。 */
ErrCode robot_torque_probe(Robot *robot, int joint, int level);

#endif
