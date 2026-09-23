
#include "control/robot.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "comm/comm_if.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "kinematics/joint_zero.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#endif

static const uint8_t SLAVE_ADDR_TABLE[ROBOT_JOINT_COUNT] = ROBOT_SLAVE_ADDR_TABLE;

/* 关节号 → Modbus 从站地址。本机实测地址是 2 与 3。 */
uint8_t joint_slave(int joint)
{
    return SLAVE_ADDR_TABLE[joint - 1];
}

struct Robot {
    const CommOps *ops;
    uint32_t baudrate;
    int online[ROBOT_JOINT_COUNT];
    int masked[ROBOT_JOINT_COUNT];
    int subdiv_ok[ROBOT_JOINT_COUNT];
    CRITICAL_SECTION lock;
};

/* 【持总线锁】发一帧并等响应。所有普通事务都要走这里。 */
ErrCode robot_request(Robot *r, const uint8_t *frame, size_t len, ModbusFrame *out)
{
    ErrCode rc;

    if (r == NULL) {
        return ERR_ARG;
    }
    EnterCriticalSection(&r->lock);
    rc = modbus_transact(frame, len, out);
    LeaveCriticalSection(&r->lock);
    return rc;
}

/* 【持总线锁】发一帧、不等响应。 */
ErrCode robot_request_noread(Robot *r, const uint8_t *frame, size_t len)
{
    ErrCode rc;

    if (r == NULL) {
        return ERR_ARG;
    }
    EnterCriticalSection(&r->lock);
    rc = modbus_transact_noread(frame, len);
    LeaveCriticalSection(&r->lock);
    return rc;
}

/* 手工持总线锁：把【多帧】当成一次不可分割的批处理（例如"先连发 6 个请求、
 * 再收 6 个响应"的流水线读）。持锁期间不要再调 robot_request ——
 * 临界区可重入、不会死锁，但语义上等于把批处理切碎，失去意义。
 * 持锁期间也不要调用会长时间阻塞的操作（会卡住监控线程）。 */
void robot_bus_lock(Robot *r)
{
    if (r == NULL) return;
    EnterCriticalSection(&r->lock);
}

/* 释放总线锁。 */
void robot_bus_unlock(Robot *r)
{
    if (r == NULL) return;
    LeaveCriticalSection(&r->lock);
}

/* 上电时把每转脉冲数(0x0024)写成 10000 并【读回校验】。
 * ⚠️ 该寄存器断电即回 4000，未对齐时同样角度会被放大 2.5 倍（实测 90° 转成 225°）。
 * 编码器在电机侧 ⇒ 程序读回永远自洽，只有写后读回比对才能发现。 */
void robot_apply_subdivision(Robot *robot)
{
    int i, ok_cnt = 0, fail_cnt = 0;

    if (robot == NULL) {
        return;
    }
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        robot->subdiv_ok[i] = 0;
    }
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        int joint = i + 1;
        int32_t rb;

        if (robot->masked[i]) {
            printf("关节%d 已屏蔽，跳过细分写入\n", joint);
            continue;
        }
        if (!robot->online[i]) {
            printf("[警告] 关节%d 离线，跳过细分写入\n", joint);
            continue;
        }
        if (motor_write_subdivision(robot, joint, ENCODER_STEPS_PER_REV) != ERR_NONE) {
            printf("[警告] 关节%d 写细分 %d 失败，角度换算将失真\n", joint,
                     (int)ENCODER_STEPS_PER_REV);
            fail_cnt++;
            continue;
        }
        rb = motor_read_subdivision(robot, joint);
        if (rb == ENCODER_STEPS_PER_REV) {
            ok_cnt++;
            robot->subdiv_ok[i] = 1;
        } else if (rb < 0) {
            printf("[警告] 关节%d 写细分成功但读回失败，实际值未知\n", joint);
            fail_cnt++;
        } else {
            printf("[错误] 关节%d 细分写入后读回 %d，与配置 %d 不符\n", joint,
                      (int)rb, (int)ENCODER_STEPS_PER_REV);
            fail_cnt++;
        }
    }
    if (fail_cnt > 0) {
        printf("[警告] 细分对齐完成：成功 %d 轴，失败 %d 轴（角度换算可能失真）\n",
                 ok_cnt, fail_cnt);
        printf("[警告] 未确认的关节【禁止运动】——驱动器出厂值是 4000，若仍是 4000，\n"
               "       下发的角度会被放大 2.5 倍（实测 90° → 225°）。\n"
               "       处理：检查该轴供电/接线后重启本程序，启动时会重跑对齐。\n");
    } else {
        printf("[细分对齐] %d 轴已确认：0x0024 = %d 脉冲/转（出厂 4000，写后读回比对通过）\n",
               ok_cnt, (int)ENCODER_STEPS_PER_REV);
    }
}

/* 某轴每转脉冲数是否已确认对齐（未确认就拒绝下发运动）。 */
int robot_subdivision_ok(const Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    if (!robot->online[joint - 1] || robot->masked[joint - 1]) {
        return 1;
    }
    return robot->subdiv_ok[joint - 1] ? 1 : 0;
}

/* 打开串口并建立 Robot 对象。
 * ⚠️ 只要求【串口能打开】，六轴全离线也不报错退出 ⇒ 不接机械臂也能跑 looptest 这类离线测试。 */
Robot *robot_init(const char *port_name, uint32_t baudrate)
{
    Robot *r = (Robot *)calloc(1, sizeof(Robot));
    const CommOps *ops = modbus_comm_get();
    int i;

    if (r == NULL) {
        return NULL;
    }
    if (baudrate == 0) {
        baudrate = MODBUS_BAUDRATE;
    }
    if (ops == NULL || ops->open == NULL) {
        printf("[错误] 总线 CommOps 未注入，无法初始化\n");
        free(r);
        return NULL;
    }
    r->ops = ops;
    r->baudrate = baudrate;
    if (ops->open(port_name, baudrate) != 0) {
        printf("[错误] 串口打开失败: %s\n", port_name);
        free(r);
        return NULL;
    }
    InitializeCriticalSection(&r->lock);
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        r->online[i] = 0;
    }
    {
        static const int mask_def[ROBOT_JOINT_COUNT] = JOINT_MASK_DEFAULT;
        for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
            r->masked[i] = mask_def[i];
            if (mask_def[i]) {
                printf("关节%d 已默认屏蔽（故障电机）\n", i + 1);
            }
        }
    }
    printf("机器人初始化完成，串口 %s @ %lu 8N1\n", port_name, (unsigned long)baudrate);

    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        if (r->masked[i]) {
            printf("关节%d 已屏蔽，跳过检测\n", i + 1);
            continue;
        }
        {
            int id = motor_read_device_addr(r, i + 1);
            if (id > 0) {
                r->online[i] = 1;
                printf("关节%d 在线，电机ID=%d\n", i + 1, id);
            } else {
                r->online[i] = 0;
                printf("[警告] 关节%d 离线\n", i + 1);
            }
        }
    }

    robot_apply_subdivision(r);

    return r;
}

/* 关闭 Robot（释放串口与锁）。 */
void robot_close(Robot *robot)
{
    if (robot == NULL) {
        return;
    }
    if (robot->ops != NULL && robot->ops->close != NULL) {
        robot->ops->close();
        robot->ops = NULL;
    }
    DeleteCriticalSection(&robot->lock);
    free(robot);
}

/* 屏蔽某轴：后续所有读写都跳过它（用于某轴故障但还要用其余轴）。 */
ErrCode robot_mask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    robot->masked[joint - 1] = 1;
    printf("关节%d 已屏蔽\n", joint);
    return ERR_NONE;
}

/* 解除某轴的屏蔽。 */
ErrCode robot_unmask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    robot->masked[joint - 1] = 0;
    printf("关节%d 已恢复\n", joint);
    return ERR_NONE;
}

/* 某轴是否被屏蔽。 */
int robot_is_masked(const Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    return robot->masked[joint - 1];
}

/* 使能某轴。⚠️ 不使能时读回的位置是假数，所以标准顺序是 enable → getpos → alarm。 */
ErrCode robot_enable(Robot *robot, int joint)
{
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        printf("关节%d 已屏蔽，跳过使能\n", joint);
        return ERR_MASKED;
    }
    rc = motor_enable(robot, joint);
    if (rc == ERR_NONE) {
        robot->online[joint - 1] = 1;
        printf("关节%d 已使能\n", joint);
    } else {
        printf("[警告] 关节%d 使能失败：%s\n", joint, err_str(rc));
        robot->online[joint - 1] = 0;
    }
    return rc;
}

/* 失能某轴。 */
ErrCode robot_disable(Robot *robot, int joint)
{
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        printf("关节%d 已屏蔽，跳过失能\n", joint);
        return ERR_MASKED;
    }
    rc = motor_disable(robot, joint);
    if (rc == ERR_NONE) {
        printf("关节%d 已失能\n", joint);
    } else {
        printf("[警告] 关节%d 失能失败：%s\n", joint, err_str(rc));
    }
    return rc;
}

/* 判断机械角是否在软限位内：1=在内 / 0=越界（out_min 与 out_max 传出边界）/ <0=关节号非法。
 * 越界的目标必然让电机顶到机械极限并触发堵转/过流，是一次纯粹的无效冲撞。 */
int robot_angle_in_soft_limit(int joint, double deg, double *out_min, double *out_max)
{
    const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;

    if (joint < 1 || joint > ROBOT_JOINT_COUNT) return -1;
    if (out_min != NULL) *out_min = lmin[joint - 1];
    if (out_max != NULL) *out_max = lmax[joint - 1];
    return (deg >= lmin[joint - 1] && deg <= lmax[joint - 1]) ? 1 : 0;
}

/* 判"读回值是不是离谱"—— 用来区分【真位置】和【总线/读回异常】。
 * 命中条件（满足其一）：① 最严重越限轴超出 big_margin_deg；
 *                      ② 同时有 >=3 个轴越限（每轴越限须 >0.5° 死区）。
 * 依据：2026-09-19 实测六轴读回同时变成越限值、末端偏差冻结在 258.92mm 不动
 * ⇒ 这是通信异常，不是机械臂真的动了。返回 1 时调用方应立刻中止而不是等超时。 */
int robot_readback_anomaly(const double deg[6], const int ok[6],
                           double big_margin_deg,
                           int *bad_joint, double *bad_excess)
{
    const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
    const double DEADZONE = 0.5;
    int n_over = 0, worst_j = 0;
    double worst_ex = 0.0;
    int i;

    if (deg == NULL || ok == NULL) return 0;

    for (i = 0; i < 6; i++) {
        double ex = 0.0;
        if (!ok[i]) continue;
        if (deg[i] < lmin[i]) ex = lmin[i] - deg[i];
        else if (deg[i] > lmax[i]) ex = deg[i] - lmax[i];
        else continue;
        if (ex > DEADZONE) n_over++;
        if (ex > worst_ex) { worst_ex = ex; worst_j = i + 1; }
    }
    if (bad_joint  != NULL) *bad_joint  = worst_j;
    if (bad_excess != NULL) *bad_excess = worst_ex;

    if (worst_ex > big_margin_deg) return 1;
    if (n_over >= 3) return 1;
    return 0;
}

/* 单轴 MoveJ：机械角 → 电机角 → 步数 → 下发 → 等到位。 */
ErrCode robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    double mech[6], motor[6];
    int32_t steps;
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        printf("关节%d 已屏蔽，跳过运动\n", joint);
        return ERR_MASKED;
    }
    if (!isfinite(angle_deg)) {
        printf("[错误] 关节%d 目标角不是有限数（%s），已拒绝下发\n",
               joint, isnan(angle_deg) ? "NaN" : "Inf");
        return ERR_ARG;
    }
    for (int i = 0; i < 6; i++) {
        mech[i] = 0.0;
    }
    mech[joint - 1] = angle_deg;
    joint_zero_mech_to_motor(mech, motor);
    steps = DEG2STEPS(motor[joint - 1], reductions[joint - 1]);
    if (speed_rpm <= 0.0) {
        speed_rpm = 60.0;
    }

    rc = motor_set_speed(robot, joint, speed_rpm);
    if (rc != ERR_NONE) {
        printf("[警告] 关节%d 写速度失败：%s\n", joint, err_str(rc));
        return rc;
    }

    rc = motor_move_abs(robot, joint, steps);
    if (rc == ERR_NONE) {
        printf("关节%d 运动到 %.2f 度 (脉冲 %d)\n", joint, angle_deg, (int)steps);
    } else {
        printf("[警告] 关节%d 运动指令失败：%s\n", joint, err_str(rc));
    }
    return rc;
}

/* 读状态字（带重试）。 */
ErrCode robot_read_status(Robot *robot, int joint, uint32_t *status)
{
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT || status == NULL) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        return ERR_MASKED;
    }
    rc = motor_read_status(robot, joint, status);
    if (rc == ERR_NONE) {
        robot->online[joint - 1] = 1;
    }
    return rc;
}

/* 读 32 位状态字。 */
ErrCode robot_read_status32(Robot *robot, int joint, uint32_t *status)
{
    return robot_read_status(robot, joint, status);
}

/* 状态读成功即在位。 */
int robot_is_online(Robot *robot, int joint)
{
    uint32_t st;
    ErrCode rc = robot_read_status(robot, joint, &st);
    return (rc == ERR_NONE) ? 1 : 0;
}

/* 读位置（步）。⚠️ 必须看 *ok：失联时返回 0 但 ok=0，把 0 当真实位置会做出错误决策。 */
int32_t robot_read_position_steps(Robot *robot, int joint, int *ok)
{
    int32_t val;
    ErrCode rc;

    if (ok) *ok = 0;
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    rc = motor_read_i32(robot, joint, LEESN_REG_POS, &val);
    if (rc != ERR_NONE) {
        return 0;
    }
    if (ok) *ok = 1;
    return val;
}

/* 读位置（机械角，度）= 步数 → 度 → 减零点。
 * ⚠️ 失联时会打 0,0,90,0,0,0 这种"看起来正常"的假数 ⇒ 必须用 alarm 交叉验证。 */
double robot_read_position_deg(Robot *robot, int joint, int *ok)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    double motor[6], mech[6];
    int32_t steps;
    int rd_ok = 0;

    if (ok) *ok = 0;
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0.0;
    }
    steps = robot_read_position_steps(robot, joint, &rd_ok);
    if (!rd_ok) {
        return 0.0;
    }
    for (int i = 0; i < 6; i++) {
        motor[i] = 0.0;
    }
    motor[joint - 1] = STEPS2DEG(steps, reductions[joint - 1]);
    joint_zero_motor_to_mech(motor, mech);
    if (ok) *ok = 1;
    return mech[joint - 1];
}

/* 读电流（mA）。 */
int robot_read_current_ma(Robot *robot, int joint)
{
    return motor_read_current(robot, joint);
}

/* 读实时速度（rpm）。 */
int robot_read_speed_rpm(Robot *robot, int joint)
{
    return motor_read_speed(robot, joint);
}

/* 读报警码（低 4 位 = 当前报警）。 */
int robot_read_alarm(Robot *robot, int joint)
{
    return motor_read_alarm(robot, joint);
}

/* 报警码 → 中文说明。 */
const char *leesn_alarm_text(int code)
{
    switch (code) {
    case LEESN_ALARM_NONE:          return "正常";
    case LEESN_ALARM_PHASE_OVERCUR: return "电机相位过流";
    case LEESN_ALARM_VBUS_HIGH:     return "供电电压过高";
    case LEESN_ALARM_VBUS_LOW:      return "供电电压过低";
    case LEESN_ALARM_PHASE_A_OPEN:  return "电机A相开路";
    case LEESN_ALARM_PHASE_B_OPEN:  return "电机B相开路";
    case LEESN_ALARM_POS_OVERDIFF:  return "位置超差";
    case LEESN_ALARM_24V_OFFSET:    return "内部24V电压偏移";
    case LEESN_ALARM_AI_VOLT:       return "AI电压错误";
    case LEESN_ALARM_BI_VOLT:       return "BI电压错误";
    case LEESN_ALARM_ENCODER:       return "编码器错误";
    default:                        return "未知报警";
    }
}
