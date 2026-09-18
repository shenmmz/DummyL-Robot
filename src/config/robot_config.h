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

/* 绝对位置写入的步数上限（|steps| 超过即拒绝）。
 *
 * 【为什么要有这条】它是所有运动指令的【最后一道出口闸门】，位于
 * motor_move_abs —— 无论目标是 MoveJ 单轴、MoveJ 多轴、MoveL、回零
 * 还是将来的任何新命令，只要写 0x00E8 就绕不过它。
 *
 * 【挡的是什么】int32 级别的垃圾目标。真机出过两次：
 *   ① NaN 关节角转 int32 ⇒ ±2147483647（2026-09-18 万向锁 asin 越界，
 *      表现为「还差 ±2147228222 步」、末端甩 34mm、超时急停）；
 *   ② 脚本写 0x00E8 时高低字写反 ⇒ 3.28 亿（同一天的另一次事故）。
 * 两次的共同点：目标值看起来是个"合法的 int32"，所以 isfinite 挡不住，
 * 只有"这个数大得不合物理"能挡住。
 *
 * 【定多少】编码器 10000 步/圈，J1 减速比 50 ⇒ 机械角一圈 = 50 万步；
 * J2 减速比 100 ⇒ 一圈 100 万步。取 1e8 步，对 J2 而言是 100 圈机械角，
 * 而本机任一关节的机械行程都在 ±360°（1 圈）内 ⇒ 留了 100 倍余量，
 * 正常作业绝不会碰线，但 3.28 亿和 21 亿必被拦下。
 *
 * 注：这条只做"绝对量级"过滤，防不住"数值在范围内但离当前位置很远"的
 * 逻辑错误 —— 那一类由 movej_issue 里的【相对位移闸门】负责（见 ini [safety]）。 */
#define ROBOT_ABS_MOVE_STEPS_LIMIT  100000000

/* 逐轴堵转/碰撞电流阈值（mA），下标 0..5 对应关节 1..6。0 = 该轴不检测。
 *
 * 【为什么必须逐轴 —— 旧设计是错的】
 * 旧版只有一个全局单值 ROBOT_STALL_CURRENT_MA，且 main.c 里传的是 0，
 * 等于堵转检测【从来没开过】。但就算填了值也照样不对：六轴机座大小不同
 * （大臂 J1~J3 与大电机、小臂 J4~J6 与小电机），额定电流能差好几倍，
 * 用同一个阈值必然要么大轴一动就误报、要么小轴撞死了都不报。
 *
 * 【本机唯一可用的判据】立三没有堵转状态寄存器，0x001A 实时电流是唯一依据。
 * 【阈值由用户定】，代码只搭框架 + 提供实测工具（curtest 命令）。
 * 运行时 ini [stall] j1..j6 可覆盖，无需重新编译。 */
#define ROBOT_STALL_CURRENT_MA_TABLE {0, 0, 0, 0, 0, 0}

#endif /* ROBOT_CONFIG_H */
