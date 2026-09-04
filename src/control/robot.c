/*
 * robot.c —— 机器人高层控制：立三（LEESN）闭环步进驱动器使能/运动/状态读取
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：robot_init、robot_close、robot_enable、robot_disable、robot_movej、
 *           robot_read_status、robot_is_online、
 *           robot_read_position_steps、robot_read_current_ma
 * 依赖模块：comm/comm_if（CommOps 总线接口）、comm/modbus_rtu（帧构造/解析）、
 *           config/robot_config、utils/logger
 * 寄存器映射依据：external/485通讯手册_sv126.1.pdf（LEESN V126）。
 * 本文件全部寄存器与命令值均按立三手册实现，
 * 特别注意 0x00D4 使能语义：写 0 = 使能。
 */

#include "control/robot.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "comm/comm_if.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* joint_slave：关节号 -> Modbus 从站地址查表（各电机驱动器基地址 0x0066 已设为 1~6） */
static const uint8_t SLAVE_ADDR_TABLE[ROBOT_JOINT_COUNT] = ROBOT_SLAVE_ADDR_TABLE;

uint8_t joint_slave(int joint)
{
    return SLAVE_ADDR_TABLE[joint - 1];
}

struct Robot {
    const CommOps *ops;              /* 注入的总线操作（CommOps，接口抽象） */
    uint32_t baudrate;
    int online[ROBOT_JOINT_COUNT];
    int masked[ROBOT_JOINT_COUNT];   /* 1=屏蔽（故障电机跳过） */
    CRITICAL_SECTION lock;           /* 总线互斥：多线程（CLI + 监控线程）共享单串口时逐帧串行 */
};

/* 发送请求并等待响应：通过注入的 CommOps 完成一次 Modbus 主从交互。
 * 帧构造由调用方完成，此处统一走 modbus_transact（flush->write->read->parse）。
 * 注意：RS485 半双工单主站，任何时刻只允许一个线程占用总线，
 * 故此处在整帧事务期间持锁（Sleep 等空闲期由上层自行控制，不持锁）。 */
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

/* robot_init：初始化机器人，注入的 CommOps 打开总线，失败返回 NULL */
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
        LOG_ERROR("总线 CommOps 未注入，无法初始化");
        free(r);
        return NULL;
    }
    r->ops = ops;
    r->baudrate = baudrate;
    if (ops->open(port_name, baudrate) != 0) {
        LOG_ERROR("串口打开失败: %s", port_name);
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
                LOG_INFO("关节%d 已默认屏蔽（故障电机）", i + 1);
            }
        }
    }
    LOG_INFO("机器人初始化完成，串口 %s @ %lu 8N1", port_name, (unsigned long)baudrate);

    /* 启动时查询各电机在线状态（读 0x0066 设备地址） */
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        if (r->masked[i]) {
            LOG_INFO("关节%d 已屏蔽，跳过检测", i + 1);
            continue;
        }
        {
            int id = motor_read_device_addr(r, i + 1);
            if (id > 0) {
                r->online[i] = 1;
                LOG_INFO("关节%d 在线，电机ID=%d", i + 1, id);
            } else {
                r->online[i] = 0;
                LOG_WARN("关节%d 离线", i + 1);
            }
        }
    }
    return r;
}

/* robot_close：关闭总线并释放机器人对象 */
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

ErrCode robot_mask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    robot->masked[joint - 1] = 1;
    LOG_INFO("关节%d 已屏蔽", joint);
    return ERR_NONE;
}

ErrCode robot_unmask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    robot->masked[joint - 1] = 0;
    LOG_INFO("关节%d 已恢复", joint);
    return ERR_NONE;
}

int robot_is_masked(const Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    return robot->masked[joint - 1];
}

/* robot_enable：使能指定关节（写 0x00D4 = 0，马达使能），返回 ErrCode */
ErrCode robot_enable(Robot *robot, int joint)
{
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        LOG_INFO("关节%d 已屏蔽，跳过使能", joint);
        return ERR_MASKED;
    }
    rc = motor_enable(robot, joint);
    if (rc == ERR_NONE) {
        robot->online[joint - 1] = 1;
        LOG_INFO("关节%d 已使能", joint);
    } else {
        LOG_WARN("关节%d 使能失败：%s", joint, err_str(rc));
        robot->online[joint - 1] = 0;
    }
    return rc;
}

/* robot_disable：失能指定关节（写 0x00D4 = 1，释放马达），返回 ErrCode */
ErrCode robot_disable(Robot *robot, int joint)
{
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        LOG_INFO("关节%d 已屏蔽，跳过失能", joint);
        return ERR_MASKED;
    }
    rc = motor_disable(robot, joint);
    if (rc == ERR_NONE) {
        LOG_INFO("关节%d 已失能", joint);
    } else {
        LOG_WARN("关节%d 失能失败：%s", joint, err_str(rc));
    }
    return rc;
}

/* robot_movej：关节绝对运动到指定角度（立三 0x00E8~0x00E9 绝对位置，
 * 速度写 0x00D8~0x00D9，单位 0.01 rpm），返回 ErrCode */
ErrCode robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    int32_t steps;
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        LOG_INFO("关节%d 已屏蔽，跳过运动", joint);
        return ERR_MASKED;
    }
    steps = DEG2STEPS(angle_deg, reductions[joint - 1]);
    if (speed_rpm <= 0.0) {
        speed_rpm = 3000.0;
    }

    rc = motor_set_speed(robot, joint, speed_rpm);
    if (rc != ERR_NONE) {
        LOG_WARN("关节%d 写速度失败：%s", joint, err_str(rc));
        return rc;
    }

    rc = motor_move_abs(robot, joint, steps);
    if (rc == ERR_NONE) {
        LOG_INFO("关节%d 运动到 %.2f 度 (脉冲 %d)", joint, angle_deg, (int)steps);
    } else {
        LOG_WARN("关节%d 运动指令失败：%s", joint, err_str(rc));
    }
    return rc;
}

/* robot_read_status：读取关节完整 32 位状态字（0x0006 低字 + 0x0007 高字）。
 * 成功返回 ERR_NONE 并置 *status（含 bit16 使能电平 / bit21 报警等全部标志）；
 * 关节被屏蔽返回 ERR_MASKED；失败返回对应 ErrCode。 */
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

/* robot_read_status32：读取关节完整 32 位状态字（0x0006~0x0007），含报警位等高位标志 */
ErrCode robot_read_status32(Robot *robot, int joint, uint32_t *status)
{
    int32_t val;
    ErrCode rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT || status == NULL) {
        return ERR_ARG;
    }
    if (robot_is_masked(robot, joint)) {
        return ERR_MASKED;
    }
    rc = motor_read_i32(robot, joint, LEESN_REG_STATUS, &val);
    if (rc == ERR_NONE) {
        robot->online[joint - 1] = 1;
        *status = (uint32_t)val;
    }
    return rc;
}

/* robot_is_online：读状态成功视为在线，返回 1/0（被屏蔽关节返回 0） */
int robot_is_online(Robot *robot, int joint)
{
    uint32_t st;
    ErrCode rc = robot_read_status(robot, joint, &st);
    return (rc == ERR_NONE) ? 1 : 0;
}

/* robot_read_position_steps：读取关节实时位置（脉冲，0x0004~0x0005 INT32），ok 指示成功 */
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

/* robot_read_current_ma：读取关节实时电流（mA，0x001A），失败返回 -1 */
int robot_read_current_ma(Robot *robot, int joint)
{
    return motor_read_current(robot, joint);
}

/* robot_read_speed_rpm：读取关节实时速度（rpm，0x0019），失败返回 -1 */
int robot_read_speed_rpm(Robot *robot, int joint)
{
    return motor_read_speed(robot, joint);
}

/* robot_read_alarm：读取关节报警代码（0x00A3），失败返回 -1 */
int robot_read_alarm(Robot *robot, int joint)
{
    return motor_read_alarm(robot, joint);
}

/* leesn_alarm_text：报警代码 -> 中文描述 */
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
