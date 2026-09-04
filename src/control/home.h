#ifndef HOME_H
#define HOME_H

/*
 * home.h —— 堵转模式回零模块接口
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：robot_home
 * 依赖模块：control/robot（内部接口）、comm/modbus_rtu、config/robot_config、utils/logger
 *
 * 回零流程：速度模式顶硬限位 → 检测堵转 → 急停 → 清零位置 → 运动到机械原点
 * 分组：组0={1,2,3,5,6} 并行堵转回零 → 组1={4} 堵转回零
 */

#include "control/robot.h"

/* 堵转模式回零：分组并行堵转回零，完成后运动到机械原点位姿。
 * 返回 ErrCode：ERR_NONE 成功，失败返回对应错误码（超时 ERR_TIMEOUT） */
ErrCode robot_home(Robot *robot);

/* 单轴回零后自动运动到指定角度：只回零关节 joint（其余轴临时屏蔽），
 * 回零成功后自动 movej 到 angle_deg（度）；speed_rpm<=0 时使用回零速度。
 * 返回 ErrCode：ERR_NONE 成功 / ERR_ARG / ERR_MASKED（目标轴被屏蔽）/ ERR_TIMEOUT */
ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm);

/* 单轴独立堵转回零（home:N）：仅操作关节 joint，不触碰/不依赖其它轴。
 * 流程：使能+关限位 → 堵转运行 → 电流超阈值 → 急停清零 →
 *       自动 movej 到该轴配置角 stall[joint].forward_deg（0 则停在清零点）。
 * 支持 1..5（堵转轴）；6 为传感器回零轴，请走 robot_home 全流程。
 * 返回 ErrCode：ERR_NONE 成功 / ERR_ARG / ERR_MASKED（目标轴被屏蔽）/ ERR_TIMEOUT */
ErrCode robot_home_single(Robot *robot, int joint);

#endif /* HOME_H */
