#ifndef MOTOR_REG_H
#define MOTOR_REG_H


#include "control/robot.h"
#include <stdint.h>


ErrCode motor_write_u16(Robot *robot, int joint, uint16_t reg, uint16_t val);

ErrCode motor_write_i32(Robot *robot, int joint, uint16_t reg, int32_t val);

ErrCode motor_read_u16(Robot *robot, int joint, uint16_t reg, uint16_t *val);

ErrCode motor_read_i32(Robot *robot, int joint, uint16_t reg, int32_t *val);


ErrCode motor_enable(Robot *robot, int joint);

ErrCode motor_disable(Robot *robot, int joint);

ErrCode motor_estop(Robot *robot, int joint);

ErrCode motor_stop_slow(Robot *robot, int joint);

ErrCode motor_set_speed(Robot *robot, int joint, double rpm);

ErrCode motor_set_speed16(Robot *robot, int joint, int rpm);

ErrCode motor_set_profile(Robot *robot, int joint, int accel_ms, int decel_ms);

ErrCode motor_clear_pos(Robot *robot, int joint);

ErrCode motor_save_params(Robot *robot, int joint);

ErrCode motor_disable_pos_err_alarm(Robot *robot, int joint);

ErrCode motor_restore_pos_err_alarm(Robot *robot, int joint);

ErrCode motor_set_pos_err_prewarn(Robot *robot, int joint, uint16_t steps);

ErrCode motor_set_limit(Robot *robot, int joint, int enable);

ErrCode motor_run(Robot *robot, int joint, int dir);

int motor_read_current(Robot *robot, int joint);

int32_t motor_read_position(Robot *robot, int joint, int *ok);

ErrCode motor_read_status(Robot *robot, int joint, uint32_t *status);

ErrCode motor_read_pos_status(Robot *robot, int joint, int32_t *pos, uint32_t *status);


ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps);
ErrCode motor_move_abs_noread(Robot *robot, int joint, int32_t steps);

int motor_move_steps_ok(int32_t steps);
ErrCode motor_write_i32_noread(Robot *robot, int joint, uint16_t reg, int32_t val);

ErrCode motor_write_i32_broadcast(Robot *robot, uint16_t reg, int32_t val);

ErrCode motor_write_u16_broadcast(Robot *robot, uint16_t reg, uint16_t val);


int motor_read_speed(Robot *robot, int joint);

int32_t motor_read_speed_raw(Robot *robot, int joint);

int32_t motor_read_subdivision(Robot *robot, int joint);

ErrCode motor_write_subdivision(Robot *robot, int joint, int32_t per_rev);

int motor_read_pos_err(Robot *robot, int joint);

int motor_read_enc_lines(Robot *robot, int joint);

int motor_read_alarm(Robot *robot, int joint);

ErrCode motor_clear_alarm(Robot *robot, int joint);

int motor_read_device_addr(Robot *robot, int joint);

ErrCode motor_set_torque_mode(Robot *robot, int joint, int mode, int level);

ErrCode motor_torque_run(Robot *robot, int joint, int dir, int offset, int run);

#endif
