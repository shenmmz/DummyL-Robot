#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/*
 * DummyL-Robot 静态配置（编译期宏）
 * ------------------------------------
 * 硬件相关固定参数已按实机写入；
 * 标注 【TODO-待用户确认】 的数值尚未实测确认，禁止臆造，确认后替换。
 * 运行时可调参数（串口端口/波特率/默认速度）见 robot_config.ini。
 */

#include <stdint.h>

/* ================= 关节数量 ================= */
#define ROBOT_JOINT_COUNT 6

/* ================= 关节屏蔽（故障电机跳过） =================
 * 启动时默认屏蔽的关节（1=屏蔽，所有操作自动跳过该关节，不发指令、不轮询）。
 * 当前 2 号电机故障，默认屏蔽关节 2；修好后改回 0 或运行时 unmask:2。
 */
#define JOINT_MASK_DEFAULT {0, 1, 0, 0, 0, 0}

/* ================= Zeta 电机减速比（固定参数） =================
 * 型号 x 减速比：1号 42/50:1、2号 42/100:1、3号 42/50:1、
 *                4号 35/50:1、5号 35/50:1、6号 28/50:1
 * 减速比含义：电机转 n 圈，输出轴转 1 圈。
 */
#define MOTOR_TYPE_1 42
#define MOTOR_TYPE_2 42
#define MOTOR_TYPE_3 42
#define MOTOR_TYPE_4 35
#define MOTOR_TYPE_5 35
#define MOTOR_TYPE_6 28

#define MOTOR_REDUCTION_1 50   /* 1号：50:1  */
#define MOTOR_REDUCTION_2 100  /* 2号：100:1 */
#define MOTOR_REDUCTION_3 50   /* 3号：50:1  */
#define MOTOR_REDUCTION_4 50   /* 4号：50:1  */
#define MOTOR_REDUCTION_5 50   /* 5号：50:1  */
#define MOTOR_REDUCTION_6 50   /* 6号：50:1  */

/* 关节 -> 减速比查表（下标 0..5 对应关节 1..6） */
#define ROBOT_REDUCTION_TABLE {50, 100, 50, 50, 50, 50}

/* ================= 编码器/步数（固定参数） ================= */
/* 单圈 16384 步（Zeta 磁编码固定值，485 无细分） */
#define ENCODER_STEPS_PER_REV 16384

/* 步数换算公式：步数 = 角度° × 减速比 ÷ 360 × 16384
 * 示例：关节2 转角 90°，步数 = 90 × 100 / 360 × 16384 = 409600
 */
#define DEG2STEPS(deg, reduction) \
    ((int32_t)(((double)(deg) * (double)(reduction)) / 360.0 * (double)ENCODER_STEPS_PER_REV))

#define STEPS2DEG(steps, reduction) \
    (((double)(steps) * 360.0) / ((double)(reduction) * (double)ENCODER_STEPS_PER_REV))

/* ================= Modbus / 串口（固定参数） ================= */
#define MODBUS_BAUDRATE 115200u   /* 115200 8N1，手册 EAH/EBH 出厂 00 01 C2 00 */
#define MODBUS_DATA_BITS 8
#define MODBUS_PARITY    'N'
#define MODBUS_STOP_BITS 1

/* CRC16 多项式 0xA001（Modbus 标准） */

/* ================= Modbus 从站地址（手册依据） =================
 * 手册 P14 寄存器 0x00E0（设备地址）：可 03H/06H/10H 操作，出厂默认 00 01 = 1；
 * P16 读设备地址示例使用广播地址 00：发 00 03 00 E0 00 01 ...，回 00 03 02 00 01 ...
 * （电机设备地址为 1）。多台电机需分别修改 0x00E0 后填入下表。
 */
#define MODBUS_SLAVE_ADDR_DEFAULT 1u
/* 六轴从站地址表（关节1..6）。各电机 0x00E0 已设为 1~6 */
#define ROBOT_SLAVE_ADDR_TABLE {1, 2, 3, 4, 5, 6}

/* ================= 堵转保护（手册依据） =================
 * 手册 P3-6 硬件参数：驱动电流范围 0~3000mA，堵转由电流、速度、时间共同决定；
 * P14 寄存器 0x00E1（堵转电流）出厂默认 0x0BB8 = 3000mA，达到该电流一段时间判定堵转；
 * P15 寄存器 0x00E5（堵转逻辑）出厂 0x0000 = 泄力（堵转断电），0x0001 = 人机对抗（保持力矩）；
 * P15 寄存器 0x00E6（堵转时间 ms）出厂 0x0000（手册未给有效默认，需实机整定）。
 * 以下为全局出厂默认值；六轴差异化阈值需实机标定后填写。
 */
#define MODBUS_STALL_CURRENT_MA_DEFAULT 3000u   /* 手册 0x00E1 出厂默认 3000mA */
/* TODO: 六轴堵转电流表（关节1..6，单位 mA），实机按负载标定后填写：
 * #define ROBOT_STALL_CURRENT_MA_TABLE {3000, 3000, 3000, 3000, 3000, 3000}
 */
/* TODO: 堵转时间（ms，寄存器 0x00E6），手册出厂 0，实机整定后填写 */

/* ================= 限位（手册依据） =================
 * 手册 P13 寄存器 0x0008（限位开关）：0 不起作用 / 1 起作用，出厂 00 01；
 * 状态字 0x0000 中 3=正光电停、4=反光电停；
 * P7 限位输入为 NPN 开漏常开（微动/光电开关）。
 * 注意：手册仅提供限位开关使能与状态标志，未提供角度软限位参数，
 * 机械限位角度需实机确认后填写。
 */
#define MODBUS_LIMIT_SWITCH_DEFAULT 1u   /* 0x0008 出厂默认：限位开关起作用 */
/* TODO: 各关节软限位角度（单位 deg，min/max 对），实机确认后填写：
 * #define ROBOT_SOFT_LIMIT_TABLE {{-170,170}, {-170,170}, ...}
 */

/* ================= 串口超时（ms） ================= */
#define SERIAL_READ_TIMEOUT_MS  100u
#define SERIAL_WRITE_TIMEOUT_MS 100u

/* ================= 回零参数 =================
 * 找零：组0={1,2,3,5,6}并行 -> 组1={4}（写 0x001F 带符号归零速度）
 * 就位：全部找零后运动到机械原点位姿 HOME_POSE_DEG
 * HOME_DIR 决定各关节归零方向（+1 正转 / -1 反转）。
 */
#define HOME_DIR             {+1, -1, +1, -1, -1, +1}
#define HOME_SPEED_RPM       100     /* 归零速度 rpm（绝对值，方向由 HOME_DIR 定） */
#define HOME_POLL_TIMEOUT_MS 15000u  /* 单组找零轮询超时 */

/* 机械原点位姿（6 关节目标角度，度）。占位 0，待用户经指令配置后掉电保存 */
#define HOME_POSE_DEG        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

#endif /* ROBOT_CONFIG_H */
