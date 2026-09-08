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
 * 写 0x00D2 = 0，把当前电机位置设为坐标原点。
 * 注意：0x00D2 是【无记忆】RAM 寄存器（手册 §67 标注"WriteDWORD,无记忆"，
 * 总表标 RO），仅即时生效、断电即丢，且不在 0x00DC 断电保存范围内
 * （手册 §70：0x00DC 只保存"所有带记忆寄存器"）。
 * 因此：零点无法持久化，本系统上电必须重新回零；且禁止在清零后追加
 * motor_save_params(0x00DC=1)——既存不住零点，又会把回零期临时关闭的
 * 报警/限位等危险值一并固化进 flash（详见 motor_save_params 注释）。
 * 返回：ERR_NONE 成功 */
ErrCode motor_clear_pos(Robot *robot, int joint)
{
    return motor_write_i32(robot, joint, LEESN_REG_SET_POS, 0);
}

/* motor_save_params：断电保存【记忆】寄存器（0x00DC = 1）
 * 手册 §70：1=保存。仅对"带记忆寄存器"生效，0x00D2（当前位置）不在其中。
 *
 * 【禁止在回零清零路径调用】三条理由：
 *   1) 存不住零点——0x00D2 无记忆，保存对零点无效；
 *   2) 有副作用——回零期间 0x000B/0x000C 超差报警被关、0x0010 偏差预警
 *      被放宽到 10000、0x006D 限位被关，此刻保存会把该危险状态固化进
 *      flash，导致下次上电带"报警关闭+限位失效"启动；
 *   3) 有代价——保存约耗时 0.1s 且期间关断电机输出（手册 §70 注 1），
 *      flash 擦写寿命约 10 万次。
 *
 * 本函数仅用于显式参数整定落盘（如改定 0x009A 连续运行速度、细分、限位
 * 等后持久化），调用前须确认所有【记忆】寄存器已处于期望值。
 * 返回：ERR_NONE 成功 */
ErrCode motor_save_params(Robot *robot, int joint)
{
    return motor_write_u16(robot, joint, LEESN_REG_SAVE_CMD, 0x0001);
}

/* motor_disable_pos_err_alarm：关闭位置超差报警（回零堵转专用）
 * 写 0x000B=0（动态误差）/ 0x000C=0（静态误差）。
 * 关闭后顶死不再触发超差报警切断输出，电流保持顶出状态，
 * 配合 home_check_stall 的纯电流判据使用。
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

/* motor_read_pos_status：一次事务同时取回位置与状态
 * 读 0x0004 起 4 个寄存器：0x0004~0x0005=位置(INT32)、0x0006~0x0007=状态(UINT32)。
 * 【提速】二者地址连续，原为两次独立事务，合并后每轮采样事务数 3→2，
 * 轮询周期按比例下降（堵转判定快慢直接取决于每轮事务数）。
 * 【正确性】位置与状态同源同帧，消除顶死瞬间"位置已停/状态仍在运行"的错位，
 * 原两次读取的时刻差会让 A/B 型判据自相矛盾。
 * 字节序：低 16 位寄存器在前、字内高字节在前。 */
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

/* ================= 运动控制 ================= */

/* motor_move_abs：绝对位置运动
 * 写 0x00E8~0x00E9（INT32 脉冲），电机运行到目标绝对位置
 * 运行中也可执行，步数由 DEG2STEPS 计算 */
ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps)
{
    return motor_write_i32(robot, joint, LEESN_REG_ABS_MOVE, steps);
}

/* ================= 状态读取（扩展） ================= */

/* motor_read_speed：读取实时速度（rpm，取整）
 * 读 0x00D6~0x00D7（INT32，单位 0.01rpm），失败返回 -1。
 * 【修正】原实现读 0x0019 有两处错：①0x0019 是 INT16，按 INT32 读会把相邻的
 * 0x001A(实时电流) 拼进高 16 位，得到 325714 之类的垃圾值；②手册注明 0x0019
 * 在 SV118 及以上固件语义变为"实际给定电流"，不再是速度。改读 0x00D6。 */
int motor_read_speed(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SPEED_ACT, &val) != ERR_NONE)
        return -1;
    return (int)LEESN_VELREG_TO_RPM(val);
}

/* motor_read_speed_raw：读取实时速度原始值（0.01rpm），保留小数精度
 * 低速回零时 60rpm 以内用 rpm 取整足够，但顶死瞬间的残余转速需要 0.01rpm 精度，
 * 故单独提供原始值接口。失败返回 -1。 */
int32_t motor_read_speed_raw(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SPEED_ACT, &val) != ERR_NONE)
        return -1;
    return val;
}

/* motor_read_subdivision：读取细分（每转脉冲数）
 * 读 0x0024~0x0025（UINT32 pulses/rev，出厂默认 4000），失败返回 -1。
 * 用途：确认 ENCODER_STEPS_PER_REV 配置与驱动器实际值是否一致。
 * 不一致会让全部 DEG2STEPS/STEPS2DEG 角度换算按比例失真。 */
int32_t motor_read_subdivision(Robot *robot, int joint)
{
    int32_t val;
    if (motor_read_i32(robot, joint, LEESN_REG_SUBDIV, &val) != ERR_NONE)
        return -1;
    return val;
}

/* motor_write_subdivision：写细分（每转脉冲数）
 * 写 0x0024~0x0025（UINT32 pulses/rev，出厂默认 4000）。
 * 用途：程序启动时按从站 ID 把驱动器细分对齐到 ENCODER_STEPS_PER_REV，
 * 保证 DEG2STEPS/STEPS2DEG 角度换算与实际机械一致。
 * 注：RAM 生效、断电丢失（0x0024 记忆与否见手册），需固化另调 motor_save_params。 */
ErrCode motor_write_subdivision(Robot *robot, int joint, int32_t per_rev)
{
    return motor_write_i32(robot, joint, LEESN_REG_SUBDIV, per_rev);
}

/* motor_read_pos_err：读取实际位置偏差值（命令位置 − 编码器位置）
 * 读 0x0011（UINT16 pulses），失败返回 -1。
 * 诊断意义：真堵转时命令在走而电机轴不转，偏差持续累积；
 * 打滑/跳齿时电机轴跟着转，偏差维持在低位。 */
int motor_read_pos_err(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_POS_ERR, &val) != ERR_NONE)
        return -1;
    return (int)val;
}

/* motor_read_enc_lines：读取编码器线数（CPR）
 * 读 0x000F（UINT16，出厂 1000），失败返回 -1。
 * 每转脉冲数 = 线数 × 4（4 倍频），可交叉验证 0x0024 细分值。 */
int motor_read_enc_lines(Robot *robot, int joint)
{
    uint16_t val;
    if (motor_read_u16(robot, joint, LEESN_REG_ENC_LINES, &val) != ERR_NONE)
        return -1;
    return (int)val;
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

/* ================= 力矩模式（手册第 49 节，仅闭环） ================= */

/* motor_set_torque_mode：设定力矩模式 + 等级，不启动运动
 * 写 0x009E（UINT16，记忆）。
 * mode   - 模式位：1=碰撞回原点 / 2=抓取物体 / 3=恒力矩运行 / 4=恒力矩保持
 * level  - 力矩等级 0~255（0 最小，255 最大）。手册：值过小会导致电机不动或
 *         达不到目标速度，需根据结构阻力整定。
 * 返回：ERR_NONE 成功，level 越界返回 ERR_ARG。
 * 注意：仅设定，须再调 motor_torque_run 才执行；清力矩模式写 motor_set_torque_mode(..,0)。 */
ErrCode motor_set_torque_mode(Robot *robot, int joint, int mode, int level)
{
    uint16_t val;
    if (level < 0 || level > 255) return ERR_ARG;
    if (mode < 0 || mode > 15)    return ERR_ARG;
    val = (uint16_t)(((mode & 0x0F) << 8) | (level & 0xFF));
    return motor_write_u16(robot, joint, LEESN_REG_TORQUE_CFG, val);
}

/* motor_torque_run：执行力矩模式（含方向/偏移脉冲/启停）
 * 写 0x00CB（UINT16，记忆）。
 * dir    - >0 正向 / <0 反向（恒力矩保持模式忽略）
 * offset - 偏移脉冲数（碰撞回原点=碰撞后偏移量作原点；抓取=松开夹子脉冲；恒力矩保持=最大纠偏脉冲）
 * run    - 0 停止 / 1 运行
 * 返回：ERR_NONE 成功。
 * 时序：先 motor_set_torque_mode 设模式与等级，再本函数启停。碰撞回原点用
 * run=1 后电机以系统速度顶向限位，力矩到顶自动停（完成信号机制见 home.c 判定）。 */
ErrCode motor_torque_run(Robot *robot, int joint, int dir, int offset, int run)
{
    uint16_t val;
    int d = (dir < 0) ? 1 : 0;
    if (offset < 0 || offset > 0x3FFF) return ERR_ARG;
    if (run != 0 && run != 1)         return ERR_ARG;
    val = (uint16_t)(((d & 0x1) << 15) | ((offset & 0x3FFF) << 1) | (run & 0x1));
    return motor_write_u16(robot, joint, LEESN_REG_TORQUE_EXEC, val);
}
