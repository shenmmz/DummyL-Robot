
#include "api/motor_reg.h"
#include "control/robot_internal.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"

#include <stdio.h>


ErrCode motor_write_u16(Robot *robot, int joint, uint16_t reg, uint16_t val)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len = modbus_build_write_single(joint_slave(joint), reg, val, frame);
    return robot_request(robot, frame, len, &resp);
}

ErrCode motor_write_i32(Robot *robot, int joint, uint16_t reg, int32_t val)
{
    uint16_t vals[2];
    uint8_t frame[32];
    ModbusFrame resp;
    vals[0] = (uint16_t)((uint32_t)val & 0xFFFFu);
    vals[1] = (uint16_t)(((uint32_t)val >> 16) & 0xFFFFu);
    size_t len = modbus_build_write_multi(joint_slave(joint), reg, vals, 2, frame);
    return robot_request(robot, frame, len, &resp);
}

ErrCode motor_write_i32_noread(Robot *robot, int joint, uint16_t reg, int32_t val)
{
    uint16_t vals[2];
    uint8_t frame[32];
    vals[0] = (uint16_t)((uint32_t)val & 0xFFFFu);
    vals[1] = (uint16_t)(((uint32_t)val >> 16) & 0xFFFFu);
    size_t len = modbus_build_write_multi(joint_slave(joint), reg, vals, 2, frame);
    return robot_request_noread(robot, frame, len);
}

ErrCode motor_write_i32_broadcast(Robot *robot, uint16_t reg, int32_t val)
{
    uint16_t vals[2];
    uint8_t frame[32];
    vals[0] = (uint16_t)((uint32_t)val & 0xFFFFu);
    vals[1] = (uint16_t)(((uint32_t)val >> 16) & 0xFFFFu);
    size_t len = modbus_build_write_multi(0, reg, vals, 2, frame);
    return robot_request_noread(robot, frame, len);
}

ErrCode motor_write_u16_broadcast(Robot *robot, uint16_t reg, uint16_t val)
{
    uint8_t frame[16];
    size_t len = modbus_build_write_single(0, reg, val, frame);
    return robot_request_noread(robot, frame, len);
}

ErrCode motor_read_u16(Robot *robot, int joint, uint16_t reg, uint16_t *val)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len = modbus_build_read(joint_slave(joint), reg, 1, frame);
    ErrCode rc = robot_request(robot, frame, len, &resp);
    if (rc != ERR_NONE) return rc;
    if (resp.data_len < 2) return ERR_LEN;
    *val = ((uint16_t)resp.data[0] << 8) | (uint16_t)resp.data[1];
    return ERR_NONE;
}

ErrCode motor_read_i32(Robot *robot, int joint, uint16_t reg, int32_t *val)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len = modbus_build_read(joint_slave(joint), reg, 2, frame);
    ErrCode rc = robot_request(robot, frame, len, &resp);
    if (rc != ERR_NONE) return rc;
    if (resp.data_len < 4) return ERR_LEN;
    uint16_t lo = ((uint16_t)resp.data[0] << 8) | (uint16_t)resp.data[1];
    uint16_t hi = ((uint16_t)resp.data[2] << 8) | (uint16_t)resp.data[3];
    *val = (int32_t)(((uint32_t)hi << 16) | (uint32_t)lo);
    return ERR_NONE;
}


ErrCode motor_enable(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_ENABLE, LEESN_CMD_ENABLE);
}

ErrCode motor_disable(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_ENABLE, LEESN_CMD_RELEASE);
}

ErrCode motor_estop(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_RUN_CTRL, LEESN_CMD_ESTOP);
}

ErrCode motor_stop_slow(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_RUN_CTRL, LEESN_CMD_STOP_SLOW);
}

ErrCode motor_set_speed(Robot *robot, int joint, double rpm)
{
    return motor_write_i32(robot, joint, LEESN_REG_VEL_RUN,
                           LEESN_RPM_TO_VELREG(rpm));
}

ErrCode motor_set_speed16(Robot *robot, int joint, int rpm)
{
    if (rpm < 0 || rpm > 10000) return ERR_ARG;
    return motor_write_u16(robot, joint, LEESN_REG_RUN_SPEED16, (uint16_t)rpm);
}

ErrCode motor_set_profile(Robot *robot, int joint, int accel_ms, int decel_ms)
{
    uint16_t vals[2];
    uint8_t frame[32];
    ModbusFrame resp;
    vals[0] = (uint16_t)accel_ms;
    vals[1] = (uint16_t)decel_ms;
    size_t len = modbus_build_write_multi(joint_slave(joint), LEESN_REG_ACC_TIME, vals, 2, frame);
    return robot_request(robot, frame, len, &resp);
}

ErrCode motor_clear_pos(Robot *robot, int joint)
{
    return motor_write_i32(robot, joint, LEESN_REG_SET_POS, 0);
}

ErrCode motor_save_params(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_SAVE_CMD, 0x0001);
}

ErrCode motor_disable_pos_err_alarm(Robot *robot, int joint)
{
    ErrCode rc;
    rc = motor_write_u16(robot, joint, LEESN_REG_ERR_DYN, 0);
    if (rc != ERR_NONE) return rc;
    return motor_write_u16(robot, joint, LEESN_REG_ERR_STAT, 0);
}

ErrCode motor_restore_pos_err_alarm(Robot *robot, int joint)
{
    ErrCode rc;
    rc = motor_write_u16(robot, joint, LEESN_REG_ERR_DYN, 200);
    if (rc != ERR_NONE) return rc;
    return motor_write_u16(robot, joint, LEESN_REG_ERR_STAT, 100);
}

ErrCode motor_set_pos_err_prewarn(Robot *robot, int joint, uint16_t steps)
{
    return motor_write_u16(robot, joint, LEESN_REG_ERR_PREWARN, steps);
}

ErrCode motor_set_limit(Robot *robot, int joint, int enable)
{
    return motor_write_u16(robot, joint, LEESN_REG_LIMIT, enable ? 0x0001 : 0x0000);
}

ErrCode motor_run(Robot *robot, int joint, int dir)
{
    uint16_t cmd = (dir > 0) ? LEESN_CMD_RUN_CW : LEESN_CMD_RUN_CCW;
    return motor_write_u16(robot, joint, LEESN_REG_RUN_CTRL, cmd);
}

int motor_read_current(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_CURRENT, &val) != ERR_NONE)
        return -1;
    return (int)val;
}

int32_t motor_read_position(Robot *robot, int joint, int *ok)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_POS, &val) != ERR_NONE) {
        if (ok) *ok = 0;
        return 0;
    }
    if (ok) *ok = 1;
    return val;
}

ErrCode motor_read_status(Robot *robot, int joint, uint32_t *status)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len = modbus_build_read(joint_slave(joint), LEESN_REG_STATUS, 2, frame);
    ErrCode rc = robot_request(robot, frame, len, &resp);
    if (rc != ERR_NONE) return rc;
    if (resp.data_len < 4) return ERR_LEN;
    uint16_t lo = ((uint16_t)resp.data[0] << 8) | (uint16_t)resp.data[1];
    uint16_t hi = ((uint16_t)resp.data[2] << 8) | (uint16_t)resp.data[3];
    *status = ((uint32_t)hi << 16) | (uint32_t)lo;
    return ERR_NONE;
}

ErrCode motor_read_pos_status(Robot *robot, int joint, int32_t *pos, uint32_t *status)
{
    uint8_t frame[16];
    ModbusFrame resp;
    uint16_t p_lo, p_hi, s_lo, s_hi;
    size_t len = modbus_build_read(joint_slave(joint), LEESN_REG_POS, 4, frame);
    ErrCode rc = robot_request(robot, frame, len, &resp);

    if (rc != ERR_NONE) return rc;
    if (resp.data_len < 8) return ERR_LEN;
    p_lo = ((uint16_t)resp.data[0] << 8) | (uint16_t)resp.data[1];
    p_hi = ((uint16_t)resp.data[2] << 8) | (uint16_t)resp.data[3];
    s_lo = ((uint16_t)resp.data[4] << 8) | (uint16_t)resp.data[5];
    s_hi = ((uint16_t)resp.data[6] << 8) | (uint16_t)resp.data[7];
    *pos    = (int32_t)(((uint32_t)p_hi << 16) | (uint32_t)p_lo);
    *status = ((uint32_t)s_hi << 16) | (uint32_t)s_lo;
    return ERR_NONE;
}


static int abs_move_steps_ok(int joint, int32_t steps)
{
    if (!motor_move_steps_ok(steps)) {
        printf("[错误] 关节%d 目标位置 %d 步超出合理量级（±%d），已拒绝下发。\n"
               "       这么大的目标只可能是逻辑错误（NaN 转 int32、高低字写反、\n"
               "       单位搞错），而它的代价是电机猛冲、超时急停、把臂甩出去。\n",
               joint, (int)steps, ROBOT_ABS_MOVE_STEPS_LIMIT);
        return 0;
    }
    return 1;
}

static int abs_move_subdiv_ok(Robot *robot, int joint)
{
    if (robot_subdivision_ok(robot, joint)) {
        return 1;
    }
    printf("[错误] 关节%d 的每转脉冲数(0x0024)未确认对齐到 %d，已拒绝下发运动。\n"
           "       该寄存器出厂为 4000、断电即回 4000，只有上电写入才是 %d；\n"
           "       未对齐时同样的角度会被放大 2.5 倍（实测 90° 会转成 225°）。\n"
           "       编码器在电机侧，程序读回永远自洽，看不出来 —— 只能靠写后读回比对。\n"
           "       处理：检查该轴供电/接线，重启本程序（启动时会重新对齐）。\n",
           joint, (int)ENCODER_STEPS_PER_REV, (int)ENCODER_STEPS_PER_REV);
    return 0;
}

int motor_move_steps_ok(int32_t steps)
{
    if (steps > ROBOT_ABS_MOVE_STEPS_LIMIT) return 0;
    if (steps < -ROBOT_ABS_MOVE_STEPS_LIMIT) return 0;
    return 1;
}

ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps)
{
    if (!abs_move_steps_ok(joint, steps)) return ERR_ARG;
    if (!abs_move_subdiv_ok(robot, joint)) return ERR_SUBDIV;
    return motor_write_i32(robot, joint, LEESN_REG_ABS_MOVE, steps);
}

ErrCode motor_move_abs_noread(Robot *robot, int joint, int32_t steps)
{
    if (!abs_move_steps_ok(joint, steps)) return ERR_ARG;
    if (!abs_move_subdiv_ok(robot, joint)) return ERR_SUBDIV;
    return motor_write_i32_noread(robot, joint, LEESN_REG_ABS_MOVE, steps);
}


int motor_read_speed(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SPEED_ACT, &val) != ERR_NONE)
        return -1;
    return (int)LEESN_VELREG_TO_RPM(val);
}

int32_t motor_read_speed_raw(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SPEED_ACT, &val) != ERR_NONE)
        return -1;
    return val;
}

int32_t motor_read_subdivision(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SUBDIV, &val) != ERR_NONE)
        return -1;
    return val;
}

ErrCode motor_write_subdivision(Robot *robot, int joint, int32_t per_rev)
{
    return motor_write_i32(robot, joint, LEESN_REG_SUBDIV, per_rev);
}

int motor_read_pos_err(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_POS_ERR, &val) != ERR_NONE)
        return -1;
    return (int)val;
}

int motor_read_enc_lines(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_ENC_LINES, &val) != ERR_NONE)
        return -1;
    return (int)val;
}

int motor_read_alarm(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_ALARM_STAT, &val) != ERR_NONE)
        return -1;
    return (int)(val & 0x0F);
}

ErrCode motor_clear_alarm(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_CLEAR_ALARM, 0x0000);
}

int motor_read_device_addr(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_DEVICE_ADDR, &val) != ERR_NONE)
        return -1;
    return (int)val;
}


ErrCode motor_set_torque_mode(Robot *robot, int joint, int mode, int level)
{
    uint16_t val;
    if (level < 0 || level > 255) return ERR_ARG;
    if (mode < 0 || mode > 15)    return ERR_ARG;
    val = (uint16_t)(((mode & 0x0F) << 8) | (level & 0xFF));
    return motor_write_u16(robot, joint, LEESN_REG_TORQUE_CFG, val);
}

ErrCode motor_torque_run(Robot *robot, int joint, int dir, int offset, int run)
{
    uint16_t val;
    int d = (dir < 0) ? 1 : 0;
    if (offset < 0 || offset > 0x3FFF) return ERR_ARG;
    if (run != 0 && run != 1)         return ERR_ARG;
    val = (uint16_t)(((d & 0x1) << 15) | ((offset & 0x3FFF) << 1) | (run & 0x1));
    return motor_write_u16(robot, joint, LEESN_REG_TORQUE_EXEC, val);
}
