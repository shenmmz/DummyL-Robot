#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <stdint.h>

/* ================= 机械本体 ================= */
#define ROBOT_JOINT_COUNT 6                                          /* 关节轴数 */
#define ROBOT_REDUCTION_TABLE {50, 100, 50, 50, 50, 50}              /* 各轴减速比：J2=100:1，其余 50:1 */
#define ENCODER_STEPS_PER_REV 10000                                  /* 每转脉冲数（驱动器 0x0024 细分；出厂 4000 启动时对齐） */

/* 角度(度) ↔ 脉冲(step)：step = deg × red ÷ 360 × steps_per_rev。 */
#define DEG2STEPS(deg, red) \
    ((int32_t)(((double)(deg) * (double)(red)) / 360.0 * (double)ENCODER_STEPS_PER_REV))
#define STEPS2DEG(steps, red) \
    (((double)(steps) * 360.0) / ((double)(red) * (double)ENCODER_STEPS_PER_REV))

#define MODBUS_BAUDRATE  921600u // 波特率
#define ROBOT_SLAVE_ADDR_TABLE {1, 2, 3, 4, 5, 6}//从站地址表ID号

/* ================= 零点与就位姿态 ================= */
/* 各轴零点（电机角度）；约定：机械角 = 电机角 − q0。ini [joint_zero] 存在则覆盖。 */
#define ROBOT_JOINT_ZERO_DEG {-176.54, 74.55, -179.94, 3.74, 115.81, 86.89}
/* 回零完成后各轴就位的机械角：0-0-90 双臂形。 */
#define ROBOT_HOME_MECH_DEG {0.0, 0.0, 90.0, 0.0, 0.0, 0.0}

/* ================= 软限位（度，机械角） ================= */
/* 安全闸门比对：目标越此区间即拒发。J4/J6 允许整周旋转 ±360°。 */
#define ROBOT_JOINT_LIMIT_MIN_DEG { -170.0, -72.0,  30.0, -360.0, -95.0, -360.0 }
#define ROBOT_JOINT_LIMIT_MAX_DEG {  179.0,  90.0, 180.0,  360.0,  95.0,  360.0 }

/* ================= 屏蔽与保护 ================= */
#define JOINT_MASK_DEFAULT {0, 0, 0, 0, 0, 0}    /* 轴屏蔽表，1=跳过该轴；默认全启用 */

/* 目标绝对步数上限（电机端行程）；超限一般意味着单位/坐标系错，直接拒发防猛冲。 */
#define ROBOT_ABS_MOVE_STEPS_LIMIT  100000000

/* 堵转电流阈值兜底(mA)，0=该轴不启用保护。ini [stall] j1..j6 存在则覆盖。 */
#define ROBOT_STALL_CURRENT_MA_TABLE {0, 0, 0, 0, 0, 0}

#endif
