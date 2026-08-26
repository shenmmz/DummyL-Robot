/*
 * monitor.c —— 关节在线检测与堵转报警监控
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：monitor_create、monitor_destroy、monitor_poll、
 *           monitor_check_stall、monitor_online_count
 * 依赖模块：control/robot、config/robot_config、utils/logger
 */

#include "control/monitor.h"
#include "control/robot_internal.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdlib.h>

struct Monitor {
    Robot *robot;
    int stall_threshold_ma; /* 堵转电流阈值；<=0 表示不检测 */
    int online[ROBOT_JOINT_COUNT];
    int online_count;
};

/* monitor_create：创建监控对象，stall_threshold_ma<=0 表示不检测堵转 */
Monitor *monitor_create(Robot *robot, int stall_threshold_ma)
{
    Monitor *m = (Monitor *)calloc(1, sizeof(Monitor));
    int i;
    if (m == NULL) {
        return NULL;
    }
    m->robot = robot;
    m->stall_threshold_ma = stall_threshold_ma;
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        m->online[i] = 0;
    }
    return m;
}

/* monitor_destroy：释放监控对象 */
void monitor_destroy(Monitor *m)
{
    free(m);
}

/* monitor_poll：轮询全部关节在线状态并报警，返回在线数。
 * 立三状态字 0x0006~0x0007 为位定义（无碰撞停/光电停状态值）：
 *   bit21 报警、bit12 到位、bit13/14 软件限位；报警代码读 0x00A3 低 4 位。 */
int monitor_poll(Monitor *m)
{
    int j;
    int cnt = 0;

    if (m == NULL || m->robot == NULL) {
        return 0;
    }
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        uint16_t st;
        ErrCode rc = robot_read_status(m->robot, j, &st);
        if (rc == ERR_NONE) {
            m->online[j - 1] = 1;
            cnt++;
            if (st & LEESN_STAT_SOFT_NEG) {
                LOG_WARN("关节%d 到达软件负限位", j);
            } else if (st & LEESN_STAT_SOFT_POS) {
                LOG_WARN("关节%d 到达软件正限位", j);
            }
        } else {
            m->online[j - 1] = 0;
        }
    }
    m->online_count = cnt;
    return cnt;
}

/* monitor_check_alarm：读 0x00A3 报警状态，有报警时打印代码并返回报警代码（0=正常） */
int monitor_check_alarm(Monitor *m, int joint)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;
    ErrCode rc;
    int code;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    len = modbus_build_read(joint_slave(joint), LEESN_REG_ALARM_STAT, 1, frame);
    rc = robot_request(m->robot, frame, len, &resp);
    if (rc != ERR_NONE || resp.data_len < 2) {
        return 0;
    }
    code = (int)((resp.data[0] << 8) | resp.data[1]) & 0x0F;  /* 低 4 位为当前报警 */
    if (code != 0) {
        LOG_ERROR("关节%d 驱动器报警：%s（代码 %d）", joint, leesn_alarm_text(code), code);
    }
    return code;
}

/* monitor_clear_alarm：写 0x00A4 = 0 清除报警，返回 ERR_NONE 成功 */
ErrCode monitor_clear_alarm(Monitor *m, int joint)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    len = modbus_build_write_single(joint_slave(joint), LEESN_REG_CLEAR_ALARM, 0x0000, frame);
    return robot_request(m->robot, frame, len, &resp);
}

/* monitor_check_stall：单关节电流超阈值检测（立三无堵转寄存器，碰撞/堵转
 * 判定唯一依据为 0x001A 实时电流），超阈值报警返回 1 */
int monitor_check_stall(Monitor *m, int joint)
{
    int cur;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    if (m->stall_threshold_ma <= 0) {
        return 0; /* 阈值未配置，跳过检测 */
    }
    cur = robot_read_current_ma(m->robot, joint);
    if (cur < 0) {
        return 0;
    }
    if (cur > m->stall_threshold_ma) {
        LOG_ERROR("关节%d 堵转报警：电流 %d mA 超阈值 %d mA",
                  joint, cur, m->stall_threshold_ma);
        return 1;
    }
    return 0;
}

/* monitor_online_count：返回最近一次轮询的在线关节数 */
int monitor_online_count(const Monitor *m)
{
    return (m != NULL) ? m->online_count : 0;
}
