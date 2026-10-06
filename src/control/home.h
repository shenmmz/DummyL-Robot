#ifndef HOME_H
#define HOME_H


#include "control/robot.h"

/* 六轴全量回零（轴1-5 堵转法，轴6 传感器法）。 */
ErrCode robot_home(Robot *robot);

/* 单轴回零（不动其他轴）：轴6 传感器法，其余堵转法。 */
ErrCode robot_home_single(Robot *robot, int joint);

#endif
