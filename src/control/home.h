#ifndef HOME_H
#define HOME_H


#include "control/robot.h"

ErrCode robot_home(Robot *robot);

ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm);

ErrCode robot_home_single(Robot *robot, int joint);

ErrCode robot_torque_probe(Robot *robot, int joint, int level);

#endif
