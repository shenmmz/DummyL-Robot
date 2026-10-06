#ifndef HOME_H
#define HOME_H


#include "control/robot.h"

/* 六轴全量回零（堵转清零）。 */
ErrCode robot_home(Robot *robot);

/* 单轴堵转回零（不动其他轴）。 */
ErrCode robot_home_single(Robot *robot, int joint);

#endif
