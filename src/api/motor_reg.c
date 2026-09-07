/*
 * motor_reg.c —— 电机寄存器 API 实现（立三 LEESN 485 驱动器）
 * ------------------------------------------------------------
 * 封装 Modbus 帧构造与请求发送，提供简洁的寄存器读写接口。
 * 所有函数 joint 参数为 1~6 关节号，内部自动查表转换为从站地址。
 */

#include "api/motor_reg.h"
#include "control/robot_internal.h"
#include "comm/modbus_rtu.h"
#include "utils/logger.h"

/* ================= 基本寄存器读写 ================= */

/* motor_write_u16：写单个 16 位寄存器（功能码 06H）
 * robot   - 机器人对象
 * joint   - 关节号 1~6
 * reg     - 寄存器地址
 * val     - 要写入的值
 * 返回：ERR_NONE 成功，其他失败 */
ErrCode motor_write_u16(Robot *robot, int joint, uint16_t reg, uint16_t val)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len = modbus_build_write_single(joint_slave(joint), reg, val, frame);
    return robot_request(robot, frame, len, &resp);
}

/* motor_write_i32：写 32 位寄存器（功能码 10H）
 * 立三 DWORD 字节序：低 16 位寄存器在前、字内高字节在前
 * robot   - 机器人对象
 * joint   - 关节号 1~6
 * reg     - 起始寄存器地址（低 16 位）
 * val     - 32 位整数值
 * 返回：ERR_NONE 成功，其他失败 */
ErrCode motor_write_i32(Robot *robot, int joint, uint16_t reg, int32_t val)
{
    uint16_t vals[2];
    uint8_t frame[32];
    ModbusFrame resp;
    vals[0] = (uint16_t)((uint32_t)val & 0xFFFFu);      /* 低 16 位 */
    vals[1] = (uint16_t)(((uint32_t)val >> 16) & 0xFFFFu); /* 高 16 位 */
    size_t len = modbus_build_write_multi(joint_slave(joint), reg, vals, 2, frame);
    return robot_request(robot, frame, len, &resp);
}

/* motor_read_u16：读单个 16 位寄存器（功能码 03H）
 * robot   - 机器人对象
 * joint   - 关节号 1~6
 * reg     - 寄存器地址
 * val     - [输出] 读取到的值
 * 返回：ERR_NONE 成功，其他失败 */
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

/* motor_read_i32：读 32 位寄存器（功能码 03H）
 * 立三 DWORD 字节序：低 16 位寄存器在前、字内高字节在前
 * robot   - 机器人对象
 * joint   - 关节号 1~6
 * reg     - 起始寄存器地址（低 16 位）
 * val     - [输出] 读取到的 32 位值
 * 返回：ERR_NONE 成功，其他失败 */
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

/* ================= 常用电机操作 ================= */

/* motor_enable：使能电机
 * 写 0x00D4 = 0，马达上电，可以接收运动指令
 * 返回：ERR_NONE 成功 */
ErrCode motor_enable(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_ENABLE, LEESN_CMD_ENABLE);
}

/* motor_disable：失能/释放电机
 * 写 0x00D4 = 1，马达断电，自由转动
 * 返回：ERR_NONE 成功 */
ErrCode motor_disable(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_ENABLE, LEESN_CMD_RELEASE);
}

/* motor_estop：急停电机
 * 写 0x00C8 = 0x0100，立即停止输出，电机保持使能状态
 * 返回：ERR_NONE 成功 */
ErrCode motor_estop(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_RUN_CTRL, LEESN_CMD_ESTOP);
}

/* motor_stop_slow：减速停止电机（退出连续运行模式）
 * 写 0x00C8 = 0x0000，按设定减速时间停止。
 * 连续运行(0x00C8=0x0001/0x0101)未退出时，后续绝对位置运动命令
 * (0x00E8) 会被驱动器忽略，因此"连续运行→清零→move_abs"序列前
 * 必须先调本函数退出连续运行模式。
 * 返回：ERR_NONE 成功 */
ErrCode motor_stop_slow(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_RUN_CTRL, LEESN_CMD_STOP_SLOW);
}

/* motor_set_speed：设置运行速度
 * 写 0x00D8~0x00D9（INT32，单位 0.01 rpm）
 * rpm - 目标速度，单位 rpm（内部自动 ×100 转换为寄存器值）
 * 注意：0x00D8 是位置/绝对运动(0x00E8/0x00DE)的速度源；
 * 速度模式连续运行(0x00C8)实际取 0x009A（见 motor_set_speed16），
 * 连续运行前必须两个都写，否则按驱动器记忆速度（默认 300rpm）运行。
 * 返回：ERR_NONE 成功 */
ErrCode motor_set_speed(Robot *robot, int joint, int rpm)
{
    return motor_write_i32(robot, joint, LEESN_REG_VEL_RUN,
                           LEESN_RPM_TO_VELREG((double)rpm));
}

/* motor_set_speed16：设置连续运行速度源（立三 Bug1 修复）
 * 写 0x009A（UINT16，单位 rpm，0~10000）。
 * 手册 66 节：0x00C8 速度模式连续运行的运行速度为 0x009A 设置值；
 * SV126 固件 0x00D8 同时服务位置模式，但连续运行模式不读它。
 * 0x009A 是 RAM 寄存器（非断电记忆），回零/连续运行每次启动前
 * 以及运行中需要调速时都必须重写本寄存器。
 * 返回：ERR_NONE 成功 */
ErrCode motor_set_speed16(Robot *robot, int joint, int rpm)
{
    if (rpm < 0 || rpm > 10000) return ERR_ARG;
    return motor_write_u16(robot, joint, LEESN_REG_RUN_SPEED16, (uint16_t)rpm);
}

/* motor_set_profile：设置加减速时间
 * 分别写 0x0098（加速时间）和 0x0099（减速时间），两寄存器独立
 * accel_ms - 加速时间 ms，从启动速度到目标速度所需时间
 * decel_ms - 减速时间 ms，从目标速度到停止速度所需时间
 * 返回：ERR_NONE 成功 */
ErrCode motor_set_profile(Robot *robot, int joint, int accel_ms, int decel_ms)
{
    ErrCode rc = motor_write_u16(robot, joint, LEESN_REG_ACC_TIME, (uint16_t)accel_ms);
    if (rc != ERR_NONE) return rc;
    return motor_write_u16(robot, joint, LEESN_REG_DEC_TIME, (uint16_t)decel_ms);
}

/* motor_clear_pos：清零当前位置
 * 写 0x00D2 = 0，把当前电机位置设为坐标原点
 * 注意：0x00D2 是 RAM 写，仅即时生效；断电保持（零点持久化）
 * 需在清零成功后追加 motor_save_params（0x00DC=1）。
 * 返回：ERR_NONE 成功 */
ErrCode motor_clear_pos(Robot *robot, int joint)
{
    return motor_write_i32(robot, joint, LEESN_REG_SET_POS, 0);
}

/* motor_save_params：断电保存 RAM 参数（立三 Bug8 修复）
 * 写 0x00DC = 1，将当前位置等 RAM 参数写入驱动器 flash，断电不丢。
 * 调用时机：位置清零(0x00D2)成功之后、连续运行速度(0x009A)改定之后。
 * 保存写 flash 需要时间，调用后应间隔数十 ms 再发后续命令。
 * 返回：ERR_NONE 成功 */
ErrCode motor_save_params(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_SAVE_CMD, 0x0001);
}

/* motor_disable_pos_err_alarm：关闭位置超差报警（回零堵转专用）
 * 写 0x000B=0（动态误差）/ 0x000C=0（静态误差）。
 * 关闭后顶死不再触发超差报警切断输出，电流保持顶出状态，
 * 配合 home_check_stall 的"电流超阈值+位置停涨"判据使用。
 * RAM 即时生效，断电/复位后恢复记忆值（默认 200/100）。
 * 返回：ERR_NONE 成功 */
ErrCode motor_disable_pos_err_alarm(Robot *robot, int joint)
{
    ErrCode rc;
    rc = motor_write_u16(robot, joint, LEESN_REG_ERR_DYN, 0);
    if (rc != ERR_NONE) return rc;
    return motor_write_u16(robot, joint, LEESN_REG_ERR_STAT, 0);
}

/* motor_restore_pos_err_alarm：恢复位置超差报警默认值
 * 写 0x000B=200 / 0x000C=100（出厂默认，动态 360°、静态 180°）。
 * 回零结束后调用，恢复正常运动时的超差保护。
 * 返回：ERR_NONE 成功 */
ErrCode motor_restore_pos_err_alarm(Robot *robot, int joint)
{
    ErrCode rc;
    rc = motor_write_u16(robot, joint, LEESN_REG_ERR_DYN, 200);
    if (rc != ERR_NONE) return rc;
    return motor_write_u16(robot, joint, LEESN_REG_ERR_STAT, 100);
}

/* motor_set_pos_err_prewarn：设置位置偏差预警
 * 写 0x0010，值域 1~65535（无法写 0 取消），默认 20，单位 Full step(1.8°)。
 * 偏差超过该值时状态字 bit10 置位并切断输出（比 0x000B/C 报警更早动作），
 * 回零堵转期间需临时放宽（如 10000）让顶死瞬间保持输出、供电流判据检测；
 * 回零结束后写回 20 恢复失步预警保护。
 * RAM 即时生效，断电/复位后恢复记忆值（默认 20）。
 * 返回：ERR_NONE 成功 */
ErrCode motor_set_pos_err_prewarn(Robot *robot, int joint, uint16_t steps)
{
    return motor_write_u16(robot, joint, LEESN_REG_ERR_PREWARN, steps);
}

/* motor_set_limit：设置限位使能
 * 写 0x006D，enable=1 限位有效，enable=0 限位失效
 * 回零时需关闭限位防止撞到硬限位报警，正常运行时开启
 * 返回：ERR_NONE 成功 */
ErrCode motor_set_limit(Robot *robot, int joint, int enable)
{
    return motor_write_u16(robot, joint, LEESN_REG_LIMIT, enable ? 0x0001 : 0x0000);
}

/* motor_run：速度模式运行
 * 写 0x00C8，dir > 0 正转(CW)，dir < 0 反转(CCW)
 * 注意：运行前需先调用 motor_set_speed 设置速度
 * 返回：ERR_NONE 成功 */
ErrCode motor_run(Robot *robot, int joint, int dir)
{
    uint16_t cmd = (dir > 0) ? LEESN_CMD_RUN_CW : LEESN_CMD_RUN_CCW;
    return motor_write_u16(robot, joint, LEESN_REG_RUN_CTRL, cmd);
}

/* motor_read_current：读取实时电流
 * 读 0x001A（UINT16，单位 mA）
 * 返回：电流值 mA，失败返回 -1 */
int motor_read_current(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_CURRENT, &val) != ERR_NONE)
        return -1;
    return (int)val;
}

/* motor_read_position：读取实时位置
 * 读 0x0004~0x0005（INT32，单位脉冲）
 * ok - [输出] 1=读取成功，0=失败，可传 NULL
 * 返回：位置脉冲数，失败时返回 0 */
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

/* motor_read_status：读取状态字完整 32 位
 * 读 0x0006（低字）+ 0x0007（高字），拼成 UINT32
 * 包含：运行状态(bit8-9)、到位(bit12)、软限位(bit13-14)、原点(bit15)、
 *       使能电平(bit16)、报警(bit21) 等（位定义见 robot_internal.h LEESN_STAT_*）
 * status - [输出] 32 位状态字
 * 返回：ERR_NONE 成功 */
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

/* ================= 运动控制 ================= */

/* motor_move_abs：绝对位置运动
 * 写 0x00E8~0x00E9（INT32 脉冲），电机运行到目标绝对位置
 * 运行中也可执行，步数由 DEG2STEPS 计算 */
ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps)
{
    return motor_write_i32(robot, joint, LEESN_REG_ABS_MOVE, steps);
}

/* ================= 状态读取（扩展） ================= */

/* motor_read_speed：读取实时速度
 * 读 0x0019（INT32，单位 0.01rpm），转换为 rpm 返回
 * 失败返回 -1 */
int motor_read_speed(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SPEED_RT, &val) != ERR_NONE)
        return -1;
    return (int)LEESN_VELREG_TO_RPM(val);
}

/* motor_read_alarm：读取报警代码
 * 读 0x00A3（UINT16），低 4 位为当前报警代码
 * 0=正常，>0=报警，失败返回 -1 */
int motor_read_alarm(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_ALARM_STAT, &val) != ERR_NONE)
        return -1;
    return (int)(val & 0x0F);
}

/* motor_clear_alarm：清除报警
 * 写 0x00A4 = 0，清除当前报警
 * 返回：ERR_NONE 成功 */
ErrCode motor_clear_alarm(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_CLEAR_ALARM, 0x0000);
}

/* motor_read_device_addr：读取驱动器地址
 * 读 0x0066（UINT16），用于检测电机是否在线
 * 返回：地址值（1~64），失败返回 -1 */
int motor_read_device_addr(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_DEVICE_ADDR, &val) != ERR_NONE)
        return -1;
    return (int)val;
}
