#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <stdint.h>

/* 关节数量 */
#define ROBOT_JOINT_COUNT 6
/* 屏蔽表：1=屏蔽（跳过不发指令），默认全不屏蔽，运行时 mask:N / unmask:N 调整 */
#define JOINT_MASK_DEFAULT {0, 0, 0, 0, 0, 0}
/* 减速比表（关节 1..6），运行时 ini [motion] reduction_* 可覆盖 */
#define ROBOT_REDUCTION_TABLE {50, 100, 50, 50, 50, 50}
/* 每转脉冲数（0x0024 细分寄存器，出厂 4000，非编码器 16384） */
#define ENCODER_STEPS_PER_REV 10000
/* DEG2STEPS：轴角度° → 电机端脉冲（绝对运动目标换算；支持负/小数，按 double 截断为 int32） */
#define DEG2STEPS(deg, red) \
    ((int32_t)(((double)(deg) * (double)(red)) / 360.0 * (double)ENCODER_STEPS_PER_REV))
/* STEPS2DEG：电机端脉冲 → 轴角度°（位置回读/校验换算；与 DEG2STEPS 互逆） */
#define STEPS2DEG(steps, red) \
    (((double)(steps) * 360.0) / ((double)(red) * (double)ENCODER_STEPS_PER_REV))
/* 串口波特率 */
#define MODBUS_BAUDRATE  115200u
/* 从站地址（六轴 1..6），运行时 ini [modbus] slave_addr_* 可覆盖 */
#define ROBOT_SLAVE_ADDR_TABLE {1, 2, 3, 4, 5, 6}
/*机械电机角度零点标定*/
#define ROBOT_JOINT_ZERO_DEG {-176.54, 74.55, -179.94, 3.74, 115.81, 86.89}
/*回零后各轴目标机械角（姿态设计量，回零退让量 = 该值 + q0[j]）*/
#define ROBOT_HOME_MECH_DEG {0.0, 0.0, 90.0, 0.0, 0.0, 0.0}
/*机械电机角度限位*/
#define ROBOT_JOINT_LIMIT_MIN_DEG { -170.0, -72.0,  30.0, -360.0, -95.0, -360.0 }
#define ROBOT_JOINT_LIMIT_MAX_DEG {  179.0,  90.0, 180.0,  360.0,  95.0,  360.0 }

#endif /* ROBOT_CONFIG_H */
