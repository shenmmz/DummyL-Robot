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
 * 1~6 号电机全部在线，默认不屏蔽；故障/未安装电机可改 1 或运行时 mask:N。
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
/* 立三闭环步进电机：4096 线编码器，每转 16384 脉冲（细分 0x0024 可设，
 * 485 模式下位置/运动命令均以脉冲为单位，默认细分不影响位置寄存器口径） */
#define ENCODER_STEPS_PER_REV 16384

/* 步数换算公式：脉冲 = 角度° × 减速比 ÷ 360 × 16384
 * 示例：关节2 转角 90°，脉冲 = 90 × 100 / 360 × 16384 = 409600
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

/* ================= 回零参数（堵转模式） =================
 * 堵转回零：0x00C8 速度模式顶硬限位 → PC 侧读 0x001A 电流超阈值判定堵转 →
 *          急停(0x00C8=0x0100) → 清零位置(0x00D2~0x00D3=0) → 运动到机械原点
 * 分组：组0={1,2,3,5,6}并行堵转回零 → 组1={4}堵转回零 →
 *       全部到位后运动到机械原点位姿 HOME_POSE_DEG
 * HOME_DIR 决定各关节归零方向（+1 正转 / -1 反转，对应 0x00C8 写 1/257）。
 */

/* 回零堵转电流（mA），参考用户提供的 home.c 实机标定值
 * 下标 0 不用，1..6 对应关节1..6
 * 关节1=500，2=500，3=800，4=500，5=500，6=0（无堵转情况，示例值待实机标定） */
#define HOME_STALL_CURRENT_MA  {0, 500, 500, 800, 500, 500, 0}

/* 堵转检测电流余量（mA）：电流 > 堵转阈值 + 此余量 时也判定堵转（兜底） */
#define HOME_STALL_MARGIN_MA   400u

/* 回零固定速度（rpm，绝对值，方向由 HOME_DIR 定；写入 0x00D8~0x00D9 运行速度 INT32，
 * 0.01 rpm，与 robot_movej 一致；0x009A 为 SV113 以下固件 UINT16 速度寄存器，不再使用） */
#define HOME_SPEED_RPM         100

/* 回零方向（+1 正转 / -1 反转） */
#define HOME_DIR               {+1, -1, +1, -1, -1, +1}

/* 超时参数（ms） */
#define HOME_GRP_TIMEOUT_MS    20000u  /* 单组堵转回零超时 */
#define HOME_ALL_TIMEOUT_MS    30000u  /* 整个回零序列兜底超时 */
#define HOME_STUCK_MS           1500u  /* 非运行态超此时长→重新发送速度命令 */
#define HOME_MIN_RUN_MS         2000u  /* 组启动后最小运行时长（防误判到位） */

/* 机械原点位姿（6 关节目标角度，度）。占位 0，待用户经指令配置后掉电保存 */
#define HOME_POSE_DEG        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

#endif /* ROBOT_CONFIG_H */
