#ifndef ROBOT_H
#define ROBOT_H


#include <stdint.h>
#include "utils/err.h"

typedef struct Robot Robot;

Robot *robot_init(const char *port_name, uint32_t baudrate);

void robot_close(Robot *robot);

ErrCode robot_enable(Robot *robot, int joint);
ErrCode robot_disable(Robot *robot, int joint);

ErrCode robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm);

int robot_angle_in_soft_limit(int joint, double deg, double *out_min, double *out_max);

int robot_readback_anomaly(const double deg[6], const int ok[6],
                           double big_margin_deg,
                           int *bad_joint, double *bad_excess);

int robot_is_online(Robot *robot, int joint);

ErrCode robot_read_status(Robot *robot, int joint, uint32_t *status);

ErrCode robot_read_status32(Robot *robot, int joint, uint32_t *status);

int32_t robot_read_position_steps(Robot *robot, int joint, int *ok);

double robot_read_position_deg(Robot *robot, int joint, int *ok);

int robot_read_current_ma(Robot *robot, int joint);

int robot_read_speed_rpm(Robot *robot, int joint);

int robot_read_alarm(Robot *robot, int joint);

void robot_apply_subdivision(Robot *robot);

int robot_subdivision_ok(const Robot *robot, int joint);

ErrCode robot_mask(Robot *robot, int joint);
ErrCode robot_unmask(Robot *robot, int joint);

int robot_is_masked(const Robot *robot, int joint);

#endif
