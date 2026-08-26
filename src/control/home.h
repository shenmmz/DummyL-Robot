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

#endif /* HOME_H */
