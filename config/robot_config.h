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

/* ------------------------------------------------------------------
 * 关节零点标定（度）：【机械零位 = 上位机电机角 q0】
 * ------------------------------------------------------------------
 * 背景：回零是"顶硬限位 → 位置清零"，故上位机的 q=0 是【硬限位位置】，
 *       不是机械设计零位；运动学（DH）用的是机械角，两者差一个常数。
 * 换算：机械角 = 上位机电机角 − q0；　上位机电机角 = 机械角 + q0
 *
 * 标定方法（2026-09-10 实测）：回零结束停在 forward_deg 姿态，
 *   实测上位机角 (−180.04, 73.51, −88.04, 6.01, 114.03, 0)
 *   对应机械姿态 (0, 0, 90, 0, 0, 0)  ← 大臂竖直、前臂水平（CAD 图姿态）
 * 故 q0 = forward_deg − 该姿态的机械角：
 *   轴1 −180.04；轴2 73.51；轴3 −88.04 − 90 = −178.04；
 *   轴4 6.01； 轴5 114.03； 轴6 0
 *
 * 由此推出【立正姿态】（机械角全 0，末端法兰高 584mm）的上位机角 = q0 本身。
 *
 * 注意：重装限位 / 换电机 / 回零退让角(home.c forward_deg)改动后需重标此表。
 * ------------------------------------------------------------------ */
#define ROBOT_JOINT_ZERO_DEG { -180.04, 73.51, -178.04, 6.01, 114.03, 0.0 }

#endif /* ROBOT_CONFIG_H */