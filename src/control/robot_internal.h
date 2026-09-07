#ifndef ROBOT_INTERNAL_H
#define ROBOT_INTERNAL_H

/*
 * robot_internal.h —— 控制层内部接口
 * ------------------------------------------------------------
 * 共享立三（LEESN）485 寄存器地址、电机状态/报警常量，以及 robot.c
 * 暴露给同层模块（如 home.c）使用的内部函数声明。不对外公开。
 *
 * 【寄存器映射依据】external/485通讯手册_sv126.1.pdf（立三机电，
 * 智能型总线控制步进驱动器，V126，2025-03）。
 * 本文件为项目协议基线，全项目一律使用本文件宏，
 * 业务代码中禁止出现裸寄存器地址。
 */

#include "control/robot.h"
#include "comm/comm_if.h"
#include "utils/err.h"
#include <stddef.h>

/* 前向声明：ModbusFrame 完整定义在 comm/modbus_rtu.h（control 层不直接依赖该头） */
typedef struct ModbusFrame ModbusFrame;

/*
 * ⚠️ DWORD（INT32/UINT32）字节序契约（立三手册全部示例帧自洽验证）：
 * 立三 32 位寄存器在 Modbus 帧中为「低 16 位寄存器在前、字内高字节在前」。
 * - 写（10H）：values[0]=低 16 位、values[1]=高 16 位（构造器按数组序逐字大端发送）。
 *   例：写 -8000 脉冲 -> values={0xE0C0, 0xFFFF} -> 帧数据 E0 C0 FF FF（手册 p21 CRC=0x0DCA）。
 * - 读（03H）：响应数据 data[0..1]=低 16 位、data[2..3]=高 16 位。
 *   例：状态 0x00000302 -> 帧数据 03 02 00 00（手册 p8，X1 输入+运行中）。
 * 既有 get_u16_be/put_u16_be 逐字序不变（字内大端），仅寄存器排列顺序按上述约定。
 */

/* ================= LEESN 立三 485 寄存器地址 ================= */
#define LEESN_REG_POS         0x0004  /* 电机实时位置 (INT32 pulses, RO) */
#define LEESN_REG_STATUS      0x0006  /* 运行及输入口状态 (UINT32, RO，位定义见下) */
#define LEESN_REG_ERR_DYN     0x000B  /* 动态误差报警阈值 (UINT16, RW，单位1.8°，0=取消，默认200) */
#define LEESN_REG_ERR_STAT    0x000C  /* 静态误差报警阈值 (UINT16, RW，单位1.8°，0=取消，默认100) */
#define LEESN_REG_ERR_PREWARN 0x0010  /* 位置偏差预警 (UINT16, RW，单位 Full step(1.8°)，值域1~65535，默认20) */
#define LEESN_REG_SERIAL_TIMEOUT 0x0008 /* 串口超时设置 (UINT16, RW，单位 10ms，0=取消) */
#define LEESN_REG_BAUD_CODE   0x0009  /* 通讯参数 (UINT16, RW，低 8 位波特率码，出厂 12=115200) */
#define LEESN_REG_SPEED_RT    0x0019  /* 实时速度 (INT32, 0.01 rpm, RO) */
#define LEESN_REG_CURRENT     0x001A  /* 实时电流 (UINT16 mA, RO) */
#define LEESN_REG_SUBDIV      0x0024  /* 细分（每转脉冲数）(UINT32, RW，出厂默认 4000) */
#define LEESN_REG_DEVICE_ADDR 0x0066  /* 驱动器基地址 (UINT16, RW，默认 1，多台逐台设置并 0x00DC 保存) */
#define LEESN_REG_ADDR_SOURCE 0x0067  /* 驱动器地址源 (UINT16, RW，默认 0) */
#define LEESN_REG_LIMIT       0x006D  /* 限位失效/有效 (UINT16, RW，出厂 0) */
#define LEESN_REG_SOFT_NEG    0x006E  /* 软件负限位 (INT32 pulses, RW) */
#define LEESN_REG_SOFT_POS    0x0070  /* 软件正限位 (INT32 pulses, RW) */
#define LEESN_REG_ACC_TIME    0x0098  /* 加速时间 (UINT16 ms, RW，默认 120) */
#define LEESN_REG_DEC_TIME    0x0099  /* 减速时间 (UINT16 ms, RW，默认 120) */
#define LEESN_REG_RUN_SPEED16 0x009A  /* 连续运行速度源 (UINT16 rpm, RW)
                                       * 手册：速度模式连续运行(0x00C8)的运行速度为 0x009A 设置值。
                                       * 注意：0x00D8 仅服务位置/绝对运动(0x00E8/0x00DE)，
                                       * 连续运行(0x00C8)不读 0x00D8；回零连续运行前必须写 0x009A
                                       * （motor_set_speed16），漏写则按记忆值 300rpm 运行（Bug1 修复）。 */
#define LEESN_REG_ALARM_STAT  0x00A3  /* 报警状态 (UINT16, RO) */
#define LEESN_REG_CLEAR_ALARM 0x00A4  /* 清除报警 (UINT16, WO) */
#define LEESN_REG_RUN_CTRL    0x00C8  /* 运行/停止 (UINT16, WO，命令值见下) */
#define LEESN_REG_HOME_CMD    0x00C9  /* 回原点执行 (UINT16, WO) */
#define LEESN_REG_JOG         0x00CA  /* 电机点动 (UINT16, WO: bit15方向/14~6速度/5停止方式/0启停) */
#define LEESN_REG_SET_POS     0x00D2  /* 设置当前电机位置 (INT32 pulses, WO) */
#define LEESN_REG_ENABLE      0x00D4  /* 脱机/使能/驱动重启 (UINT16, WO，命令值见下) */
#define LEESN_REG_VEL_RUN     0x00D8  /* 运行速度 (INT32, RW，0.01 rpm，默认 30000) */
#define LEESN_REG_SAVE_CMD    0x00DC  /* 断电保存命令 (UINT16, WO：1=保存 0=恢复出厂) */
#define LEESN_REG_REL_MOVE    0x00DE  /* 运行脉冲数 (INT32 pulses, WO，相对当前位置) */
#define LEESN_REG_ABS_MOVE    0x00E8  /* 运行到绝对位置 (INT32 pulses, WO，运行/停止都可执行) */

/* 状态寄存器 0x0006 位定义（UINT32） */
#define LEESN_STAT_INPUT(n)   (1u << (n))   /* bit0~7：X0~X7 输入口状态，1=有输入 */
#define LEESN_STAT_RUN_MASK   0x0300u       /* bit8~9：运行状态 */
#define LEESN_STAT_RUN_IDLE   0x0000u       /*  00 空闲 */
#define LEESN_STAT_RUN_START  0x0100u       /*  01 即将启动 */
#define LEESN_STAT_RUN_STOP   0x0200u       /*  10 即将停止 */
#define LEESN_STAT_RUN_ACTIVE 0x0300u       /*  11 正在运行 */
#define LEESN_STAT_OVERRUN    0x0400u       /* bit10：位置超差警告 */
#define LEESN_STAT_REMIND11   0x0800u       /* bit11：位置提醒 X11 */
#define LEESN_STAT_INPOS      0x1000u       /* bit12：到位输出标识，1=到位 */
#define LEESN_STAT_SOFT_NEG   0x2000u       /* bit13：软件负限位标识 */
#define LEESN_STAT_SOFT_POS   0x4000u       /* bit14：软件正限位标识 */
#define LEESN_STAT_HOMED      0x8000u       /* bit15：原点完成标志，1=在原点 */
#define LEESN_STAT_ENABLE_LVL 0x00010000u   /* bit16：使能电平标志 */
#define LEESN_STAT_ALARM      0x00200000u   /* bit21：驱动器报警状态，1=报警 */

/* 运行/停止 0x00C8 命令值 */
#define LEESN_CMD_STOP_SLOW   0x0000u  /* 减速停止 */
#define LEESN_CMD_RUN_CW      0x0001u  /* 正向运行 */
#define LEESN_CMD_ESTOP       0x0100u  /* 急停 */
#define LEESN_CMD_RUN_CCW     0x0101u  /* 反向运行 */

/* 脱机/使能 0x00D4 命令值 */
#define LEESN_CMD_ENABLE      0x0000u  /* 马达使能 */
#define LEESN_CMD_RELEASE     0x0001u  /* 释放马达（脱机） */
#define LEESN_CMD_DRV_RESET   0x0100u  /* 驱动重启 */

/* 运行速度 0x00D8~0x00D9 单位 0.01 rpm（出厂默认 30000 = 300 rpm） */
#define LEESN_RPM_TO_VELREG(rpm)   ((int32_t)((double)(rpm) * 100.0))
#define LEESN_VELREG_TO_RPM(v)     ((double)(v) / 100.0)

/* 通讯参数 0x0009 波特率码（低 8 位）：出厂 12 = 115200 */
#define LEESN_BAUD_CODE_115200 12u

/* 报警状态 0x00A3：低 4 位为当前报警代码（0=正常） */
#define LEESN_ALARM_NONE            0   /* 正常 */
#define LEESN_ALARM_PHASE_OVERCUR   1   /* 电机相位过流 */
#define LEESN_ALARM_VBUS_HIGH       2   /* 供电电压过高 */
#define LEESN_ALARM_VBUS_LOW        3   /* 供电电压过低 */
#define LEESN_ALARM_PHASE_A_OPEN    4   /* 电机 A 相开路 */
#define LEESN_ALARM_PHASE_B_OPEN    5   /* 电机 B 相开路 */
#define LEESN_ALARM_POS_OVERDIFF    6   /* 其他报警或位置超差 */
#define LEESN_ALARM_24V_OFFSET      7   /* 内部 24V 电压偏移 */
#define LEESN_ALARM_AI_VOLT         8   /* AI 电压错误 */
#define LEESN_ALARM_BI_VOLT         9   /* BI 电压错误 */
#define LEESN_ALARM_ENCODER         10  /* 编码器错误 */

/* 报警代码 -> 中文描述（未知代码返回"未知报警"） */
const char *leesn_alarm_text(int code);

/* ================= 内部函数（供 home.c 等同层模块使用） ================= */

/* 发送 Modbus 请求并等待响应：返回解析结果（ERR_NONE 成功 / 对应错误码） */
ErrCode robot_request(Robot *r, const uint8_t *frame, size_t len, ModbusFrame *out);

/* 关节号 -> Modbus 从站地址查表 */
uint8_t joint_slave(int joint);

#endif /* ROBOT_INTERNAL_H */
