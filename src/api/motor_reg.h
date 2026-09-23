#ifndef MOTOR_REG_H
#define MOTOR_REG_H


#include "control/robot.h"
#include <stdint.h>


/* 写 16 位寄存器（功能码 06H），写完【等响应】确认。
 * 一笔事务 1.70ms @921600。joint=1..6，内部查表转从站地址；reg 一律用
 * robot_internal.h 的 LEESN_REG_* 宏，禁止裸地址。全程走总线锁。 */
ErrCode motor_write_u16(Robot *robot, int joint, uint16_t reg, uint16_t val);

/* 写 32 位寄存器（功能码 10H，2 个连续寄存器）。
 * ⚠️ 立三字节序：values[0]=低 16 位、values[1]=高 16 位（字内仍大端）。 */
ErrCode motor_write_i32(Robot *robot, int joint, uint16_t reg, int32_t val);

/* 读 16 位寄存器（功能码 03H）。 */
ErrCode motor_read_u16(Robot *robot, int joint, uint16_t reg, uint16_t *val);

/* 读 32 位寄存器（功能码 03H，2 个连续寄存器，响应低字在前）。 */
ErrCode motor_read_i32(Robot *robot, int joint, uint16_t reg, int32_t *val);


/* 马达使能（0x00D4=0）。⚠️ 不 enable 时 getpos 读回来的是假数。 */
ErrCode motor_enable(Robot *robot, int joint);

/* 释放马达 / 脱机（0x00D4=1）。 */
ErrCode motor_disable(Robot *robot, int joint);

/* 急停（0x00C8=0x0100）。 */
ErrCode motor_estop(Robot *robot, int joint);

/* 减速停止（0x00C8=0x0000）。 */
ErrCode motor_stop_slow(Robot *robot, int joint);

/* 写运行速度 0x00D8（寄存器单位 0.01rpm，入参是 rpm）。 */
ErrCode motor_set_speed(Robot *robot, int joint, double rpm);

/* 写【连续运行】速度 0x009A（整数 rpm）。
 * ⚠️ 0x00D8 只服务位置/绝对运动，连续运行(0x00C8)读的是 0x009A；漏写就按
 * 记忆值 300rpm 跑（Bug1 修复点）。 */
ErrCode motor_set_speed16(Robot *robot, int joint, int rpm);

/* 一次写加速/减速时间（0x0098 起连续 2 个寄存器，单位 ms）。
 * 实测六轴：启停速度 50rpm、加速 **80ms**、减速 **90ms** ⇒ 段末速度归零要 170ms。 */
ErrCode motor_set_profile(Robot *robot, int joint, int accel_ms, int decel_ms);

/* 设置当前电机位置 = 0（0x00D2）。 */
ErrCode motor_clear_pos(Robot *robot, int joint);

/* 断电保存（0x00DC=1）。⚠️ 会把【所有】带记忆的 RAM 值一起写进 Flash；写 0 是恢复出厂。 */
ErrCode motor_save_params(Robot *robot, int joint);

/* 关掉位置超差报警（0x000B/0x000C 写 0）。回零堵转期间必须关，否则半路报 ERR_ALARM。 */
ErrCode motor_disable_pos_err_alarm(Robot *robot, int joint);

/* 恢复位置超差报警的出厂阈值（动态 200 / 静态 100，单位 1.8°）。 */
ErrCode motor_restore_pos_err_alarm(Robot *robot, int joint);

/* 写位置偏差预警阈值 0x0010（单位 full step = 1.8°，值域 1~65535，默认 20）。 */
ErrCode motor_set_pos_err_prewarn(Robot *robot, int joint, uint16_t steps);

/* 软件限位失效/有效（0x006D）。 */
ErrCode motor_set_limit(Robot *robot, int joint, int enable);

/* 点动（0x00CA，bit15 方向 / bit14~6 速度 / bit5 停止方式 / bit0 启停）。 */
ErrCode motor_run(Robot *robot, int joint, int dir);

/* 读实时电流 0x001A（mA）；失败返回 -1。
 * ⚠️ 实测静止电流 ~500/499/495/385/371/254，而 [stall] 阈值是 480/490/480/400/390
 * ⇒ 阈值疑似偏低（J1/J2/J3 的静止电流已超过阈值）。 */
int motor_read_current(Robot *robot, int joint);

/* 读实时位置 0x0004（脉冲）。ok 出参：0 = 读失败，此时返回值无意义（别当 0 用）。 */
int32_t motor_read_position(Robot *robot, int joint, int *ok);

/* 读状态字 0x0006（UINT32，位定义见 robot_internal.h 的 LEESN_STAT_*）。
 * ⚠️ STAT_INPOS(bit12) 是【粘滞位】，只看它必然假阳性（实测差 28.5° 也报"到位"）。 */
ErrCode motor_read_status(Robot *robot, int joint, uint32_t *status);

/* 一次读回「位置 + 状态字」（0x0004 起连续 4 个寄存器）——
 * 比分开读省一半总线往返；单笔事务 1.70ms，能合并就合并。 */
ErrCode motor_read_pos_status(Robot *robot, int joint, int32_t *pos, uint32_t *status);


/* 运行到绝对位置 0x00E8（等响应确认）。
 * ⚠️ 运行中写入 = 立即执行 + 强行结束当前指令 ⇒ 段末速度归零（ACC+DEC = 170ms）。 */
ErrCode motor_move_abs(Robot *robot, int joint, int32_t steps);
/* 同上但【不等响应】。⚠️ 实测零收益（从站仍要 1.47ms 周转 + 占线回帧），
 * 且会留下没人读走的残帧污染下一笔读 ⇒ 默认不用。 */
ErrCode motor_move_abs_noread(Robot *robot, int joint, int32_t steps);

/* 校验绝对位移步数是否超 ROBOT_ABS_MOVE_STEPS_LIMIT（防天文数字猛冲）。 */
int motor_move_steps_ok(int32_t steps);
/* 通用「写 32 位但不等响应」。⚠️ 帧间必须自行留 gap，实测安全帧间隔 2ms（0ms 撞车）。 */
ErrCode motor_write_i32_noread(Robot *robot, int joint, uint16_t reg, int32_t val);

/* 广播写 32 位（从站地址 0）。实测 10H 广播写 0x00D8 六轴全执行 ⇒ 广播有效。 */
ErrCode motor_write_i32_broadcast(Robot *robot, uint16_t reg, int32_t val);

/* 广播写 16 位（从站地址 0）。6 台驱动器改波特率必须用广播一起写，否则当场失联。 */
ErrCode motor_write_u16_broadcast(Robot *robot, uint16_t reg, uint16_t val);


/* 读实时速度 0x00D6（0.01rpm），转成整数 rpm 返回；失败返回 -1。
 * 【勿用】0x0019：手册注明高版本固件语义已变为"实际给定电流"。 */
int motor_read_speed(Robot *robot, int joint);

/* 读实时速度 0x00D6 原始值（0.01rpm）；失败返回 0。 */
int32_t motor_read_speed_raw(Robot *robot, int joint);

/* 读细分（每转脉冲数，0x0024）；失败返回 -1。出厂默认 4000，本机上电写 10000。 */
int32_t motor_read_subdivision(Robot *robot, int joint);

/* 写细分 0x0024。⚠️ 断电即回 4000；未对齐时同样的角度会被放大 2.5 倍
 * （实测 90° 会转成 225°）⇒ 必须每次上电重写。 */
ErrCode motor_write_subdivision(Robot *robot, int joint, int32_t per_rev);

/* 读实际位置偏差 0x0011（脉冲）。区分"真堵转"与"传动打滑"的关键量：
 * 偏差持续累积 = 命令在走、电机轴没转；偏差≈0 而位置在涨 = 打滑/跳齿。 */
int motor_read_pos_err(Robot *robot, int joint);

/* 读编码器线数 0x000F（CPR）；失败返回 -1。出厂 1000。 */
int motor_read_enc_lines(Robot *robot, int joint);

/* 读报警码 0x00A3 低 4 位（0=正常）；读不到返回 -1。
 * 代码含义见 robot_internal.h 的 LEESN_ALARM_*。 */
int motor_read_alarm(Robot *robot, int joint);

/* 清报警（0x00A4=0）。⚠️ 不先排除原因，清完会立刻复现。 */
ErrCode motor_clear_alarm(Robot *robot, int joint);

/* 读驱动器基地址 0x0066；失败返回 -1。实测本机是 2 与 3 两个值。 */
int motor_read_device_addr(Robot *robot, int joint);

/* 力矩模式设定 0x009E（BIT15~8 模式 1碰撞回原点/2抓取/3恒力矩运行/4恒力矩保持，
 * BIT7~0 力矩等级 0~255）。 */
ErrCode motor_set_torque_mode(Robot *robot, int joint, int mode, int level);

/* 力矩模式执行 0x00CB（BIT15 方向 / BIT14~1 偏移脉冲数 / BIT0 启停）。 */
ErrCode motor_torque_run(Robot *robot, int joint, int dir, int offset, int run);

#endif
