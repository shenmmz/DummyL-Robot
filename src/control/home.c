/*
 * home.c —— 回零实现（堵转模式 + 传感器模式）
 * ------------------------------------------------------------
 * 关节1-4：堵转回零（速度模式顶硬限位 → 电流超阈值 → 清零位置）
 * 关节5：  堵转回零 → 清零 → 正向移动
 * 关节6：  传感器回零（反向→碰IN0降速→离开→正向碰IN0→极慢速离开→清零）
 *
 * 所有电机寄存器操作统一走 motor_reg API，不直接操作 Modbus 帧。
 */

#include "control/home.h"
#include "api/motor_reg.h"
#include "control/robot_internal.h"   /* LEESN_STAT_* 状态位定义 */
#include "utils/logger.h"
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

/* ====================== 回零参数 ====================== */

/* 关节1-5 堵转回零参数（下标1-5对应关节1-5） */
typedef struct {
    int    speed_rpm;     /* 回零速度 rpm */
    int    accel_ms;      /* 加减速时间 ms */
    int    dir;           /* 方向 +1正转 / -1反转 */
    int    stall_current; /* 堵转电流 mA，0=仅靠状态字判定 */
    double forward_deg;   /* 堵转后正向移动角度，0=不移动 */
} StallHome;

static StallHome stall[6] = {
    [1] = { 80, 300, +1, 500, 0   },
    [2] = { 60, 400, -1, 500, 0   },
    [3] = { 80, 300, +1, 800, 0   },
    [4] = { 60, 500, -1, 500, 0   },
    [5] = {100, 200, -1, 500, 5.0 },
};

/* 关节6 传感器回零参数 */
static struct {
    int fast_rpm;         /* 快速寻找传感器 */
    int slow_rpm;         /* 碰到IN0后降速 */
    int crawl_rpm;        /* 极慢速离开IN0 */
    int dir;              /* 初始方向 -1反向 */
    int accel_ms;         /* 加减速时间 ms */
} sensor = { 300, 90, 30, -1, 100 };

/* 通用参数 */
static int    home_timeout_ms  = 20000;
static double home_pose_deg[6] = {0, 0, 0, 0, 0, 0};

/* 传感器输入位（状态寄存器 0x0006 的 bit0=IN0, bit1=IN1） */
#define SENSOR_IN0  0x0001u
#define SENSOR_IN1  0x0002u

/* ====================== 传感器回零阶段 ====================== */

typedef enum {
    SEN_REV_FAST,  /* 反向快速找IN0或IN1 */
    SEN_REV_SLOW,  /* 反向慢速（在IN0区域，等离开） */
    SEN_FWD_FAST,  /* 从IN1转向正向，快速找IN0 */
    SEN_FWD_SLOW,  /* 慢速正向碰IN0 */
    SEN_CRAWL,     /* 极慢速正向离开IN0 */
    SEN_DONE       /* 完成 */
} SensorPhase;

/* ====================== 内部辅助函数 ====================== */

/* home_arm：回零前准备 —— 使能电机 + 关闭硬限位
 * 回零时电机要顶到硬限位，必须先关闭限位否则会报警 */
static ErrCode home_arm(Robot *robot, int joint)
{
    ErrCode rc = motor_enable(robot, joint);
    if (rc != ERR_NONE) return rc;
    Sleep(50);
    return motor_set_limit(robot, joint, 0);  /* 关闭限位 */
}

/* home_restore：回零后恢复 —— 保持使能 + 开启硬限位 */
static ErrCode home_restore(Robot *robot, int joint)
{
    ErrCode rc = motor_enable(robot, joint);
    if (rc != ERR_NONE) LOG_WARN("关节%d 恢复使能失败", joint);
    Sleep(30);
    rc = motor_set_limit(robot, joint, 1);     /* 开启限位 */
    if (rc != ERR_NONE) LOG_WARN("关节%d 恢复限位失败", joint);
    return rc;
}

/* home_stall_start：启动堵转回零 —— 设加速度 → 设速度 → 按方向运行 */
static void home_stall_start(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];
    motor_set_accel(robot, joint, p->accel_ms);
    motor_set_speed(robot, joint, p->speed_rpm);
    Sleep(20);
    motor_run(robot, joint, p->dir);
    LOG_INFO("关节%d 堵转启动 (方向=%+d, %drpm, %dms, 阈值=%dmA)",
             joint, p->dir, p->speed_rpm, p->accel_ms, p->stall_current);
}

/* home_check_stall：检查是否堵转到位
 * 返回 1=堵转到位，0=还在运行 */
static int home_check_stall(Robot *robot, int joint)
{
    uint16_t st;
    int cur, pos_ok = 0;
    int32_t pos;
    int threshold = stall[joint].stall_current;

    if (motor_read_status(robot, joint, &st) != ERR_NONE) return 0;
    cur = motor_read_current(robot, joint);
    if (cur < 0 || cur > 3000) cur = 0;
    pos = motor_read_position(robot, joint, &pos_ok);

    LOG_INFO("关节%d 状态=0x%04X 电流=%dmA 位置=%d (阈值=%dmA)",
             joint, st, cur, pos_ok ? (int)pos : -99999, threshold);

    return (threshold > 0 && cur > threshold);
}

/* home_stall_done：堵转到位处理 —— 急停 → 清零位置 */
static void home_stall_done(Robot *robot, int joint)
{
    int cur = motor_read_current(robot, joint);
    int pos_ok = 0;
    int32_t pos;

    motor_estop(robot, joint);
    Sleep(50);
    int clear_ok = (motor_clear_pos(robot, joint) == ERR_NONE);
    Sleep(30);
    pos = motor_read_position(robot, joint, &pos_ok);
    LOG_INFO("关节%d 堵转到位 (电流=%dmA)，清零%s，位置=%d",
             joint, cur, clear_ok ? "成功" : "失败", pos_ok ? (int)pos : -99999);
}

/* home_wait_inpos：等待关节运动到位（非运行状态） */
static void home_wait_inpos(Robot *robot, int joint, int timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < (uint32_t)timeout_ms) {
        uint16_t st;
        if (motor_read_status(robot, joint, &st) != ERR_NONE) {
            Sleep(200); elapsed += 200; continue;
        }
        if ((st & LEESN_STAT_RUN_MASK) != LEESN_STAT_RUN_ACTIVE) break;
        Sleep(200);
        elapsed += 200;
    }
}

/* ====================== 关节1-4：并行堵转回零 ====================== */

/* home_stall_group：一组关节并行堵转回零
 * joints - 关节号数组
 * cnt    - 关节数量 */
static void home_stall_group(Robot *robot, const int *joints, int cnt)
{
    uint8_t done = 0;
    uint32_t start_ms = GetTickCount();

    /* 逐个启动 */
    for (int j = 0; j < cnt; j++) {
        int joint = joints[j];
        if (robot_is_masked(robot, joint)) continue;
        if (home_arm(robot, joint) != ERR_NONE) {
            LOG_WARN("关节%d arm 失败，跳过", joint);
            done |= (1u << (joint - 1));
            continue;
        }
        home_stall_start(robot, joint);
        Sleep(10);
    }

    /* 轮询堵转状态 */
    while ((GetTickCount() - start_ms) < (uint32_t)home_timeout_ms) {
        int all_done = 1;
        for (int j = 0; j < cnt; j++) {
            int joint = joints[j];
            if (robot_is_masked(robot, joint)) continue;
            if (done & (1u << (joint - 1))) continue;
            all_done = 0;

            if (home_check_stall(robot, joint)) {
                home_stall_done(robot, joint);
                done |= (1u << (joint - 1));
            }
        }
        if (all_done) break;
        Sleep(200);
    }

    /* 超时处理 */
    for (int j = 0; j < cnt; j++) {
        int joint = joints[j];
        if (robot_is_masked(robot, joint)) continue;
        if (!(done & (1u << (joint - 1)))) {
            LOG_WARN("关节%d 堵转回零超时", joint);
            motor_estop(robot, joint);
        }
    }
}

/* ====================== 关节5：堵转 + 正向移动 ====================== */

/* home_joint5：关节5 堵转回零 → 清零 → 正向移动 forward_deg 度 */
static void home_joint5(Robot *robot)
{
    StallHome *p = &stall[5];
    uint32_t start_ms;

    if (home_arm(robot, 5) != ERR_NONE) { LOG_WARN("关节5 arm 失败"); return; }
    home_stall_start(robot, 5);

    /* 等待堵转 */
    start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)home_timeout_ms) {
        if (home_check_stall(robot, 5)) {
            home_stall_done(robot, 5);
            break;
        }
        Sleep(200);
    }

    /* 正向移动一段距离 */
    LOG_INFO("关节5 正向移动 %.1f°", p->forward_deg);
    robot_movej(robot, 5, p->forward_deg, (double)p->speed_rpm);
    home_wait_inpos(robot, 5, home_timeout_ms);
    home_restore(robot, 5);
    LOG_INFO("关节5 回零完成");
}

/* ====================== 关节6：传感器回零 ====================== */

/* sensor_phase_name：阶段名（用于日志） */
static const char *sensor_phase_name(SensorPhase ph)
{
    static const char *names[] = {
        "反向快速", "反向慢速", "正向快速", "正向慢速", "极慢速离开", "完成"
    };
    if (ph < 0 || ph >= (int)(sizeof(names) / sizeof(names[0]))) return "未知";
    return names[ph];
}

/* home_joint6：关节6 传感器回零状态机
 * 覆盖3种情况：
 *   1. 反向→碰IN0降速→离开→慢速正向碰IN0→极慢速离开→清零
 *   2. 反向→碰IN1→转向正向→碰IN0→极慢速离开→清零
 *   3. 初始就在IN0上→反向离开→慢速正向碰IN0→极慢速离开→清零 */
static void home_joint6(Robot *robot)
{
    SensorPhase ph = SEN_REV_FAST;
    int in0_was_on = 0;
    uint32_t start_ms, crawl_start = 0;

    if (home_arm(robot, 6) != ERR_NONE) { LOG_WARN("关节6 arm 失败"); return; }
    motor_set_accel(robot, 6, sensor.accel_ms);
    motor_set_speed(robot, 6, sensor.fast_rpm);
    Sleep(20);
    motor_run(robot, 6, sensor.dir);
    LOG_INFO("关节6 传感器回零启动 (反向快速 %drpm)", sensor.fast_rpm);

    start_ms = GetTickCount();

    while (ph != SEN_DONE) {
        uint16_t inputs;
        int in0, in1, cur, pos_ok = 0;
        int32_t pos;
        uint32_t now = GetTickCount();

        /* 超时检测 */
        if (ph == SEN_CRAWL && crawl_start > 0 && (now - crawl_start) >= 10000) {
            LOG_WARN("关节6 极慢速离开IN0超时");
            break;
        }
        if (ph != SEN_CRAWL && (now - start_ms) >= (uint32_t)home_timeout_ms) {
            LOG_WARN("关节6 传感器回零超时");
            break;
        }

        /* 读传感器状态 */
        if (motor_read_status(robot, 6, &inputs) != ERR_NONE) { Sleep(100); continue; }
        in0 = (inputs & SENSOR_IN0) ? 1 : 0;
        in1 = (inputs & SENSOR_IN1) ? 1 : 0;
        cur = motor_read_current(robot, 6);
        if (cur < 0 || cur > 3000) cur = 0;
        pos = motor_read_position(robot, 6, &pos_ok);

        LOG_INFO("关节6 IN0=%d IN1=%d 电流=%dmA 位置=%d 阶段=%s",
                 in0, in1, cur, pos_ok ? (int)pos : -99999, sensor_phase_name(ph));

        /* 状态机切换 */
        switch (ph) {
        case SEN_REV_FAST:
            if (in1) {
                /* 碰到反向限位IN1，转向正向找IN0 */
                LOG_INFO("关节6 碰到IN1，改为正向快速找IN0");
                motor_set_speed(robot, 6, sensor.fast_rpm);
                Sleep(10);
                motor_run(robot, 6, +1);
                ph = SEN_FWD_FAST;
            } else if (in0) {
                /* 碰到IN0，降速继续反向，等离开IN0 */
                LOG_INFO("关节6 碰到IN0，降速至%drpm", sensor.slow_rpm);
                motor_set_speed(robot, 6, sensor.slow_rpm);
                ph = SEN_REV_SLOW;
            }
            break;

        case SEN_REV_SLOW:
            if (!in0) {
                /* 离开IN0，转为慢速正向，准备精确对齐 */
                LOG_INFO("关节6 离开IN0，慢速正向找IN0 (%drpm)", sensor.slow_rpm);
                motor_set_speed(robot, 6, sensor.slow_rpm);
                Sleep(10);
                motor_run(robot, 6, +1);
                ph = SEN_FWD_SLOW;
            }
            break;

        case SEN_FWD_FAST:
        case SEN_FWD_SLOW:
            if (in0) {
                /* 正向碰到IN0，极慢速正向离开，离开瞬间清零 */
                LOG_INFO("关节6 正向碰到IN0，极慢速离开 (%drpm)", sensor.crawl_rpm);
                motor_set_speed(robot, 6, sensor.crawl_rpm);
                in0_was_on = 1;
                crawl_start = GetTickCount();
                ph = SEN_CRAWL;
            }
            break;

        case SEN_CRAWL:
            if (in0_was_on && !in0) {
                /* 刚离开IN0，停止并清零位置 */
                LOG_INFO("关节6 极慢速离开IN0，回零完成");
                motor_estop(robot, 6);
                Sleep(50);
                ph = SEN_DONE;
            }
            break;

        default:
            break;
        }

        Sleep(100);
    }

    /* 完成后清零 / 超时急停 */
    if (ph == SEN_DONE) {
        int pos_ok = 0;
        int32_t pos;
        int clear_ok = (motor_clear_pos(robot, 6) == ERR_NONE);
        Sleep(30);
        pos = motor_read_position(robot, 6, &pos_ok);
        LOG_INFO("关节6 位置清零%s，位置=%d",
                 clear_ok ? "成功" : "失败", pos_ok ? (int)pos : -99999);
    } else {
        motor_estop(robot, 6);
    }

    home_restore(robot, 6);
    LOG_INFO("关节6 回零完成");
}

/* ====================== 公开接口 ====================== */

/* robot_home：执行回零流程
 * 步骤：
 *   1. 关节1-4 并行堵转回零
 *   2. 关节5 堵转回零 + 正向移动
 *   3. 关节6 传感器回零
 *   4. 关节1-4 运动到机械原点
 * 返回 ERR_NONE 成功，ERR_TIMEOUT 超时 */
ErrCode robot_home(Robot *robot)
{
    const int group0[] = {1, 2, 3, 4};
    int j;
    ErrCode rc = ERR_NONE;

    if (robot == NULL) return ERR_ARG;

    /* 打印参数总览 */
    LOG_INFO("=== 回零参数 ===");
    for (j = 1; j <= 5; j++) {
        if (robot_is_masked(robot, j)) {
            LOG_INFO("  关节%d: 已屏蔽", j);
        } else {
            LOG_INFO("  关节%d: 堵转 %drpm %dms 方向=%+d 阈值=%dmA 正向=%.1f°",
                     j, stall[j].speed_rpm, stall[j].accel_ms, stall[j].dir,
                     stall[j].stall_current, stall[j].forward_deg);
        }
    }
    if (!robot_is_masked(robot, 6)) {
        LOG_INFO("  关节6: 传感器 快%d/慢%d/极慢%drpm 方向=%+d",
                 sensor.fast_rpm, sensor.slow_rpm, sensor.crawl_rpm, sensor.dir);
    }
    LOG_INFO("================");

    /* 第1步：关节1-4 堵转回零 */
    LOG_INFO("回零：{1,2,3,4} 堵转回零...");
    home_stall_group(robot, group0, 4);

    /* 第2步：关节5 堵转 + 正向移动 */
    if (!robot_is_masked(robot, 5)) {
        LOG_INFO("回零：关节5 堵转 + 正向%.1f°...", stall[5].forward_deg);
        home_joint5(robot);
    }

    /* 第3步：关节6 传感器回零 */
    if (!robot_is_masked(robot, 6)) {
        LOG_INFO("回零：关节6 传感器回零...");
        home_joint6(robot);
    }

    /* 第4步：关节1-4 运动到机械原点 */
    LOG_INFO("运动到机械原点位姿...");
    for (j = 1; j <= 4; j++) {
        if (robot_is_masked(robot, j)) continue;
        if (robot_movej(robot, j, home_pose_deg[j - 1],
                        (double)stall[j].speed_rpm) != ERR_NONE)
            rc = ERR_TIMEOUT;
    }

    /* 等待到位 */
    LOG_INFO("等待运动到位...");
    {
        uint32_t elapsed = 0;
        while (elapsed < (uint32_t)home_timeout_ms) {
            int all_done = 1;
            for (j = 1; j <= 4; j++) {
                uint16_t st;
                if (robot_is_masked(robot, j)) continue;
                if (motor_read_status(robot, j, &st) != ERR_NONE) continue;
                if ((st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) all_done = 0;
            }
            if (all_done) { LOG_INFO("运动到位"); break; }
            Sleep(200);
            elapsed += 200;
        }
    }

    /* 恢复限位 */
    for (j = 1; j <= 4; j++) {
        if (robot_is_masked(robot, j)) continue;
        home_restore(robot, j);
        Sleep(20);
    }

    LOG_INFO("回零流程结束");
    return rc;
}
