#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <stdint.h>

#define ROBOT_JOINT_COUNT 6
#define JOINT_MASK_DEFAULT {0, 0, 0, 0, 0, 0}
#define ROBOT_REDUCTION_TABLE {50, 100, 50, 50, 50, 50}
#define ENCODER_STEPS_PER_REV 10000
#define DEG2STEPS(deg, red) \
    ((int32_t)(((double)(deg) * (double)(red)) / 360.0 * (double)ENCODER_STEPS_PER_REV))
#define STEPS2DEG(steps, red) \
    (((double)(steps) * 360.0) / ((double)(red) * (double)ENCODER_STEPS_PER_REV))
/* ini 缺 baudrate 键时的兜底值。必须与驱动器 0x0009 档位一致（当前六轴=档位15=921600，
 * 已固化），否则兜底生效时会当场失联。改 ini 的 baudrate 时这里要跟着改。 */
#define MODBUS_BAUDRATE  921600u
#define ROBOT_SLAVE_ADDR_TABLE {1, 2, 3, 4, 5, 6}
#define ROBOT_JOINT_ZERO_DEG {-176.54, 74.55, -179.94, 3.74, 115.81, 86.89}
#define ROBOT_HOME_MECH_DEG {0.0, 0.0, 90.0, 0.0, 0.0, 0.0}
#define ROBOT_JOINT_LIMIT_MIN_DEG { -170.0, -72.0,  30.0, -360.0, -95.0, -360.0 }
#define ROBOT_JOINT_LIMIT_MAX_DEG {  179.0,  90.0, 180.0,  360.0,  95.0,  360.0 }

#define ROBOT_ABS_MOVE_STEPS_LIMIT  100000000

#define ROBOT_STALL_CURRENT_MA_TABLE {0, 0, 0, 0, 0, 0}

#endif
