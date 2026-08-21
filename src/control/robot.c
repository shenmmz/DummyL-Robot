/*
 * robot.c —— 机器人高层控制：Zeta 伺服使能/运动/回零/状态读取
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：robot_init、robot_close、robot_enable、robot_disable、robot_movej、
 *           robot_home、robot_read_status、robot_is_online、
 *           robot_read_position_steps、robot_read_current_ma
 * 依赖模块：comm/serial_win、comm/modbus_rtu、comm/crc16、config/robot_config、utils/logger
 */

#include "control/robot.h"
#include "comm/serial_win.h"
#include "comm/modbus_rtu.h"
#include "comm/crc16.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================= Zeta 寄存器地址 ================= */
#define ZETA_REG_STATUS      0x0000  /* 状态字 */
#define ZETA_REG_POS_HI      0x0001  /* 实际步数高位 */
#define ZETA_REG_POS_LO      0x0002  /* 实际步数低位 */
#define ZETA_REG_CURRENT     0x0005  /* 电流 (mA) */
#define ZETA_REG_ENABLE      0x0006  /* 使能: 1 使能 / 0 失能 */
#define ZETA_REG_LIMIT       0x0008  /* 限位开关使能 */
#define ZETA_REG_TARGET_HI   0x0010  /* 位置模式目标步数高位 */
#define ZETA_REG_TARGET_LO   0x0011  /* 位置模式目标步数低位 */
#define ZETA_REG_TARGET_VEL  0x0013  /* 位置模式目标速度 (rpm) */
#define ZETA_REG_TARGET_ACC  0x0014  /* 位置模式加速度 (rpm/s) */
#define ZETA_REG_TARGET_PREC 0x0015  /* 位置模式到位精度 (步) */
#define ZETA_REG_HOME_CMD    0x001F  /* 归零指令（写入归零速度） */
#define ZETA_REG_SPEED_MODE  0x0061  /* 速度模式速度 */
#define ZETA_REG_DEVICE_ADDR 0x00E0  /* 设备地址 */
#define ZETA_REG_STALL_CUR   0x00E1  /* 堵转电流 (mA) */

/* joint_slave：关节号 -> Modbus 从站地址查表（各电机 0x00E0 已设为 1~6） */
static const uint8_t SLAVE_ADDR_TABLE[ROBOT_JOINT_COUNT] = ROBOT_SLAVE_ADDR_TABLE;

static uint8_t joint_slave(int joint)
{
    return SLAVE_ADDR_TABLE[joint - 1];
}

struct Robot {
    SerialPort *port;
    uint32_t baudrate;
    int online[ROBOT_JOINT_COUNT];
    int masked[ROBOT_JOINT_COUNT];   /* 1=屏蔽（故障电机跳过） */
};

/* 发送请求并等待响应：返回解析结果（0 成功 / 负错误码） */
static int robot_request(Robot *r, const uint8_t *frame, size_t len, ModbusFrame *out)
{
    uint8_t rx[512];
    int rx_len;

    serial_flush(r->port);
    if (serial_write(r->port, frame, len) != (int)len) {
        LOG_ERROR("串口写入失败");
        return -1;
    }
    rx_len = serial_read(r->port, rx, sizeof(rx), SERIAL_READ_TIMEOUT_MS);
    if (rx_len <= 0) {
        return -1;
    }
    return modbus_parse_response(rx, (size_t)rx_len, out);
}

/* robot_init：初始化机器人，打开串口并设置超时，失败返回 NULL */
Robot *robot_init(const char *port_name, uint32_t baudrate)
{
    Robot *r = (Robot *)calloc(1, sizeof(Robot));
    int i;

    if (r == NULL) {
        return NULL;
    }
    if (baudrate == 0) {
        baudrate = MODBUS_BAUDRATE;
    }
    r->baudrate = baudrate;
    r->port = serial_open(port_name, baudrate);
    if (r->port == NULL) {
        LOG_ERROR("串口打开失败: %s", port_name);
        free(r);
        return NULL;
    }
    serial_set_timeout(r->port, SERIAL_READ_TIMEOUT_MS, SERIAL_WRITE_TIMEOUT_MS);
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
    return r;
}

/* robot_close：关闭串口并释放机器人对象 */
void robot_close(Robot *robot)
{
    if (robot == NULL) {
        return;
    }
    if (robot->port != NULL) {
        serial_close(robot->port);
        robot->port = NULL;
    }
    free(robot);
}

int robot_mask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    robot->masked[joint - 1] = 1;
    LOG_INFO("关节%d 已屏蔽", joint);
    return 0;
}

int robot_unmask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    robot->masked[joint - 1] = 0;
    LOG_INFO("关节%d 已恢复", joint);
    return 0;
}

int robot_is_masked(const Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    return robot->masked[joint - 1];
}

/* robot_enable：使能指定关节（写使能寄存器 0x0006） */
int robot_enable(Robot *robot, int joint)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;
    int rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    if (robot_is_masked(robot, joint)) {
        LOG_INFO("关节%d 已屏蔽，跳过使能", joint);
        return 0;
    }
    len = modbus_build_write_single(joint_slave(joint), ZETA_REG_ENABLE, 0x0001, frame);
    rc = robot_request(robot, frame, len, &resp);
    if (rc == 0) {
        robot->online[joint - 1] = 1;
        LOG_INFO("关节%d 已使能", joint);
    } else {
        LOG_WARN("关节%d 使能失败 (rc=%d)", joint, rc);
        robot->online[joint - 1] = 0;
    }
    return rc;
}

/* robot_disable：失能指定关节（写使能寄存器为 0） */
int robot_disable(Robot *robot, int joint)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;
    int rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    if (robot_is_masked(robot, joint)) {
        LOG_INFO("关节%d 已屏蔽，跳过失能", joint);
        return 0;
    }
    len = modbus_build_write_single(joint_slave(joint), ZETA_REG_ENABLE, 0x0000, frame);
    rc = robot_request(robot, frame, len, &resp);
    if (rc == 0) {
        LOG_INFO("关节%d 已失能", joint);
    } else {
        LOG_WARN("关节%d 失能失败 (rc=%d)", joint, rc);
    }
    return rc;
}

/* robot_movej：关节绝对运动到指定角度（位置模式写 4 个寄存器） */
int robot_movej(Robot *robot, int joint, double angle_deg, double speed_rpm)
{
    uint8_t frame[64];
    ModbusFrame resp;
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    int32_t steps;
    uint16_t vals[6];
    uint16_t speed;
    size_t len;
    int rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    if (robot_is_masked(robot, joint)) {
        LOG_INFO("关节%d 已屏蔽，跳过运动", joint);
        return 0;
    }
    steps = DEG2STEPS(angle_deg, reductions[joint - 1]);
    speed = (uint16_t)((speed_rpm > 0.0) ? speed_rpm : 3000.0);

    /* 位置模式从 0x0010 连续写 6 个寄存器（手册示例）：
     * {目标步数高, 目标步数低, 0(0x12保留占位), 速度, 加速度, 精度} */
    vals[0] = (uint16_t)((uint32_t)steps >> 16);
    vals[1] = (uint16_t)((uint32_t)steps & 0xFFFF);
    vals[2] = 0;             /* 0x0012 保留 */
    vals[3] = speed;          /* 0x0013 速度 */
    vals[4] = 10000;          /* 0x0014 加速度 rpm/s */
    vals[5] = 100;            /* 0x0015 到位精度 步 */

    len = modbus_build_write_multi(joint_slave(joint), ZETA_REG_TARGET_HI, vals, 6, frame);
    rc = robot_request(robot, frame, len, &resp);
    if (rc == 0) {
        LOG_INFO("关节%d 运动到 %.2f 度 (步数 %d)", joint, angle_deg, (int)steps);
    } else {
        LOG_WARN("关节%d 运动指令失败 (rc=%d)", joint, rc);
    }
    return rc;
}

/* robot_home：依次向全部关节发送回零指令 */
/* robot_home 实现已移至文件末尾（依赖 robot_read_status 等后置函数） */

int robot_read_status(Robot *robot, int joint)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;
    int rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    if (robot_is_masked(robot, joint)) {
        return -2;  /* 屏蔽 */
    }
    len = modbus_build_read(joint_slave(joint), ZETA_REG_STATUS, 1, frame);
    rc = robot_request(robot, frame, len, &resp);
    if (rc != 0 || resp.data_len < 2) {
        return -1;
    }
    robot->online[joint - 1] = 1;
    return (int)((resp.data[0] << 8) | resp.data[1]);
}

/* robot_is_online：读状态成功视为在线，返回 1/0 */
int robot_is_online(Robot *robot, int joint)
{
    int st = robot_read_status(robot, joint);
    return (st >= 0) ? 1 : 0;
}

/* robot_read_position_steps：读取关节实际位置（步数），ok 指示成功 */
int32_t robot_read_position_steps(Robot *robot, int joint, int *ok)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;
    int rc;

    if (ok) *ok = 0;
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    len = modbus_build_read(joint_slave(joint), ZETA_REG_POS_HI, 2, frame);
    rc = robot_request(robot, frame, len, &resp);
    if (rc != 0 || resp.data_len < 4) {
        return 0;
    }
    if (ok) *ok = 1;
    return (int32_t)(((uint32_t)resp.data[0] << 24) |
                     ((uint32_t)resp.data[1] << 16) |
                     ((uint32_t)resp.data[2] << 8) |
                     (uint32_t)resp.data[3]);
}

/* robot_read_current_ma：读取关节电流（mA），失败返回 -1 */
int robot_read_current_ma(Robot *robot, int joint)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;
    int rc;

    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return -1;
    }
    len = modbus_build_read(joint_slave(joint), ZETA_REG_CURRENT, 1, frame);
    rc = robot_request(robot, frame, len, &resp);
    if (rc != 0 || resp.data_len < 2) {
        return -1;
    }
    return (int)((resp.data[0] << 8) | resp.data[1]);
}

/* 向一组关节发送归零指令（0x001F，速度按 HOME_DIR 取正负） */
static int home_send_group(Robot *robot, const int *joints, int cnt)
{
    static const int home_dir[ROBOT_JOINT_COUNT] = HOME_DIR;
    int j, rc = 0;

    for (j = 0; j < cnt; j++) {
        int joint = joints[j];
        if (robot_is_masked(robot, joint)) {
            LOG_INFO("关节%d 已屏蔽，跳过归零", joint);
            continue;
        }
        int dir = home_dir[joint - 1];
        uint16_t speed = (uint16_t)(int16_t)(dir * (int)HOME_SPEED_RPM);
        uint8_t frame[16];
        ModbusFrame resp;
        size_t len = modbus_build_write_single(joint_slave(joint), ZETA_REG_HOME_CMD, speed, frame);
        int r = robot_request(robot, frame, len, &resp);
        if (r == 0) {
            LOG_INFO("关节%d 归零指令已发送 (%+d rpm)", joint, dir * (int)HOME_SPEED_RPM);
        } else {
            LOG_WARN("关节%d 归零失败 (rc=%d)", joint, r);
            rc = r;
        }
    }
    return rc;
}

/* 轮询一组关节直到全部非运行态（0000 待机/到位 或 0002~0004 停止），超时返回 -1 */
static int wait_group_idle(Robot *robot, const int *joints, int cnt)
{
    uint32_t elapsed = 0;

    while (elapsed < HOME_POLL_TIMEOUT_MS) {
        int all_done = 1;
        int j;
        for (j = 0; j < cnt; j++) {
            int joint = joints[j];
            int st;
            if (robot_is_masked(robot, joint)) {
                continue;  /* 屏蔽关节视为已到位 */
            }
            st = robot_read_status(robot, joint);
            if (st < 0 || st == 0x0001) {  /* 离线或运行中 */
                all_done = 0;
            }
        }
        if (all_done) {
            return 0;
        }
        elapsed += SERIAL_READ_TIMEOUT_MS;  /* 每轮约一次串口往返 */
    }
    return -1;
}

int robot_home(Robot *robot)
{
    static const double home_pose[ROBOT_JOINT_COUNT] = HOME_POSE_DEG;
    const int group0[] = {1, 2, 3, 5, 6};
    const int group1[] = {4};
    const int all[] = {1, 2, 3, 4, 5, 6};
    int j, rc = 0;

    if (robot == NULL) {
        return -1;
    }

    LOG_INFO("回零：组0 {1,2,3,5,6} 并行找零...");
    home_send_group(robot, group0, 5);
    if (wait_group_idle(robot, group0, 5) != 0) {
        LOG_WARN("组0 找零超时");
        rc = -1;
    }

    LOG_INFO("回零：组1 {4} 找零...");
    home_send_group(robot, group1, 1);
    if (wait_group_idle(robot, group1, 1) != 0) {
        LOG_WARN("组1 找零超时");
        rc = -1;
    }

    LOG_INFO("回零完成，运动到机械原点位姿...");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        if (robot_movej(robot, j, home_pose[j - 1], (double)HOME_SPEED_RPM) != 0) {
            rc = -1;
        }
    }
    wait_group_idle(robot, all, 6);

    LOG_INFO("回零流程结束");
    return rc;
}
