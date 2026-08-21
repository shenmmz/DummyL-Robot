/*
 * monitor.c —— 关节在线检测与堵转报警监控
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：monitor_create、monitor_destroy、monitor_poll、
 *           monitor_check_stall、monitor_online_count
 * 依赖模块：control/robot、config/robot_config、utils/logger
 */

#include "control/monitor.h"
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

/* monitor_poll：轮询全部关节在线状态并报警，返回在线数 */
int monitor_poll(Monitor *m)
{
    int j;
    int cnt = 0;

    if (m == NULL || m->robot == NULL) {
        return 0;
    }
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        int st = robot_read_status(m->robot, j);
        if (st >= 0) {
            m->online[j - 1] = 1;
            cnt++;
            if (st == MONITOR_STATUS_COLLISION) {
                LOG_WARN("关节%d 碰撞停", j);
            } else if (st == MONITOR_STATUS_PHOTO_POS) {
                LOG_WARN("关节%d 正光电停", j);
            } else if (st == MONITOR_STATUS_PHOTO_NEG) {
                LOG_WARN("关节%d 反光电停", j);
            }
        } else {
            m->online[j - 1] = 0;
        }
    }
    m->online_count = cnt;
    return cnt;
}

/* monitor_check_stall：单关节堵转电流检测，超阈值报警返回 1 */
int monitor_check_stall(Monitor *m, int joint)
{
    int cur;

    if (m == NULL || m->robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return 0;
    }
    if (m->stall_threshold_ma <= 0) {
        return 0; /* 阈值未配置，跳过堵转检测 */
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
