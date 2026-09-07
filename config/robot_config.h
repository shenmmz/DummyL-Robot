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

/* 步数换算：脉冲 = 角度° × 减速比 ÷ 360 × 每转脉冲数 */
#define DEG2STEPS(deg, red) \
    ((int32_t)(((double)(deg) * (double)(red)) / 360.0 * (double)ENCODER_STEPS_PER_REV))
#define STEPS2DEG(steps, red) \
    (((double)(steps) * 360.0) / ((double)(red) * (double)ENCODER_STEPS_PER_REV))

/* 串口波特率 */
#define MODBUS_BAUDRATE  115200u

/* 从站地址（六轴 1..6），运行时 ini [modbus] slave_addr_* 可覆盖 */
#define ROBOT_SLAVE_ADDR_TABLE {1, 2, 3, 4, 5, 6}

#endif /* ROBOT_CONFIG_H */