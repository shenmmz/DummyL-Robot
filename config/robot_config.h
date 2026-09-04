#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/*
 * DummyL-Robot 配置（方案二：配置单一来源）
 * --------------------------------------------
 * 参数归属表：
 *   【编译期固定参数】只留宏，禁止走 ini（改需重新编译，物理不可变）：
 *     - CRC16 多项式 0xA001（低字节序）
 *     - Modbus 帧常量（功能码 03H/04H/06H/10H、异常位 0x80、地址域 0/1~64）
 *     - 步数换算公式系数（DEG2STEPS：角度×减速比÷360×16384）
 *     - 编码器分辨率 ENCODER_STEPS_PER_REV = 16384（4096 线）
 *     - 关节数量 ROBOT_JOINT_COUNT、串口帧格式 8N1（data/parity/stop bits）
 *
 *   【运行时可变参数】只走 robot_config.ini，宏仅作"默认值"兜底（ini 缺失时生效）：
 *     - 串口端口 port / 波特率 baudrate        -> [serial]
 *     - 从站地址 1~6 slave_addr_1..6           -> [modbus]
 *     - 波特率码 baudrate_code（0x0009 低8位） -> [modbus]
 *     - 各轴软限位 soft_limit_*_deg            -> [limits]
 *     - 各轴电流阈值 stall_current_ma_1..6     -> [safety]
 *
 *   【宏默认值 + ini 可覆盖】：
 *     - 减速比表 ROBOT_REDUCTION_TABLE（ini [motion] reduction_*）
 *     - 屏蔽表 JOINT_MASK_DEFAULT（ini [motion] joint_mask_*）
 *
 * 约定：本头文件所有"运行时可变参数"的宏名保留（编译期默认值），
 * 实际生效值由 main 启动时从 robot_config.ini 装载并打印"生效来源"。
 * 标注 【TODO-待用户确认】 的数值尚未实测确认，禁止臆造，确认后替换。
 */

#include <stdint.h>

/* ================= 关节数量 ================= */
#define ROBOT_JOINT_COUNT 6

/* ================= 关节屏蔽（未安装/故障电机跳过） =================
 * 启动时默认屏蔽的关节（1=屏蔽，所有操作自动跳过该关节，不发指令、不轮询）。
 * 当前 6 轴电机全部在线/实装，默认不屏蔽任何关节（{0,0,0,0,0,0}）。
 * 若个别关节故障或未接线，可运行时用 mask:N / unmask:N 调整。
 */
#define JOINT_MASK_DEFAULT {0, 0, 0, 0, 0, 0}

/* ================= 立三闭环步进电机减速比（固定参数） =================
 * 型号 x 减速比：1号 42/50:1、2号 42/100:1、3号 42/50:1、
 *                4号 35/50:1、5号 35/50:1、6号 28/50:1
 * 减速比含义：电机转 n 圈，输出轴转 1 圈。
 * 注：立三手册未提供减速比参数，减速比表 {50,100,50,50,50,50} 为开发提示词确定值
 *     （1/3/4/5/6=50:1，2 号=100:1）。
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

/* 关节 -> 减速比查表（下标 0..5 对应关节 1..6）。
 * 运行时来源：robot_config.ini [motion] reduction_*（宏为默认值，可覆盖） */
#define ROBOT_REDUCTION_TABLE {50, 100, 50, 50, 50, 50}

/* ================= 编码器/步数（固定参数） ================= */
/* 立三闭环步进电机：位置/运动命令的脉冲口径 = 0x0024 细分寄存器（每转脉冲数，
 * 出厂默认 4000）。注意：不是编码器物理分辨率（4096 线×4=16384），
 * 16384 是闭环编码器计数，不能用于角度换算，否则 move_abs 会超发约 4 倍。 */
#define ENCODER_STEPS_PER_REV 4000

/* 步数换算公式：脉冲 = 角度° × 减速比 ÷ 360 × 4000（0x0024 每转脉冲数）
 * 示例：关节2 转角 90°，脉冲 = 90 × 100 / 360 × 4000 = 100000
 */
#define DEG2STEPS(deg, reduction) \
    ((int32_t)(((double)(deg) * (double)(reduction)) / 360.0 * (double)ENCODER_STEPS_PER_REV))

#define STEPS2DEG(steps, reduction) \
    (((double)(steps) * 360.0) / ((double)(reduction) * (double)ENCODER_STEPS_PER_REV))

/* ================= Modbus / 串口（固定参数） ================= */
#define MODBUS_BAUDRATE 115200u   /* 115200 8N1，立三手册通讯参数寄存器 0x0009 = 12 即 115200 */
#define MODBUS_DATA_BITS 8
#define MODBUS_PARITY    'N'
#define MODBUS_STOP_BITS 1

/* CRC16 多项式 0xA001（Modbus 标准，CRC 低字节在前，其余字段大端） */

/* ================= Modbus 从站地址（手册依据） =================
 * 立三手册寄存器 0x0066（驱动器基地址）：默认 1，可读写；
 * 0x0067 为驱动器地址源（RW，默认 0）。地址域 0 为广播地址（电机不回应），
 * 1~64 为子节点。多台电机需分别写 0x0066 修改基地址并写 0x00DC=1 断电保存。
 */
#define MODBUS_SLAVE_ADDR_DEFAULT 1u
/* 六轴从站地址表（关节1..6），运行时可变参数（默认值宏）：
 * 各电机驱动器基地址 0x0066 已设为 1~6；
 * 运行时来源：robot_config.ini [modbus] slave_addr_1..6（当前版本默认值生效） */
#define ROBOT_SLAVE_ADDR_TABLE {1, 2, 3, 4, 5, 6}

/* ================= 过流/堵转保护（PC 侧判定） =================
 * 立三无堵转电流/堵转逻辑/堵转时间专用寄存器，
 * 碰撞/堵转判定由 PC 侧轮询 0x001A 实时电流（mA）超阈值完成，
 * 驱动器报警仅读 0x00A3（过流/过压/开路等，见 robot_internal.h）、清除 0x00A4。
 * 以下为全局默认阈值（mA），保守上限，各轴按实机负载标定。
 */
#define MODBUS_STALL_CURRENT_MA_DEFAULT 3000u
/* 六轴差异化电流阈值表（关节1..6，单位 mA），运行时可变参数（默认值宏）。
 * 实机按负载标定后填写；运行时来源：robot_config.ini [safety] stall_current_ma_1..6 */
#define ROBOT_STALL_CURRENT_MA_TABLE_DEFAULT {3000, 3000, 3000, 3000, 3000, 3000}

/* ================= 限位（手册依据） =================
 * 立三手册：0x006D（限位失效/有效）、0x009B（硬件正负限位端口设置）、
 * 0x006E~0x006F / 0x0070~0x0071（软件负/正限位，INT32 脉冲）；
 * 状态字 0x0006 bit13/bit14 为软件负/正限位标识。
 * 注意：角度软限位（度）换算为脉冲限位值需实机确认后填写。
 */
/* 各关节软限位角度（度，min/max 对），运行时可变参数（默认值宏，占位 0=未启用）。
 * 角度软限位换算为脉冲限位值需实机确认后填写；
 * 运行时来源：robot_config.ini [limits] soft_limit_*_deg */
#define ROBOT_SOFT_LIMIT_TABLE_DEFAULT \
    {{-170, 170}, {-170, 170}, {-170, 170}, {-170, 170}, {-170, 170}, {-170, 170}}

/* ================= 串口超时（ms） ================= */
#define SERIAL_READ_TIMEOUT_MS  100u
#define SERIAL_WRITE_TIMEOUT_MS 100u

#endif /* ROBOT_CONFIG_H */
