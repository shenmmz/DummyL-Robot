/*
 * home.c —— 回零实现
 * 关节1-5：堵转回零（顶硬限位→电流/输出切断判据→清零→movej 到 forward_deg）
 * 关节6：IN0/IN1 传感器回零（零点=IN0 正向 on→off 沿，兼容三种初始位置）
 * 寄存器操作统一走 motor_reg API，不直接操作 Modbus 帧。
 */

#include "control/home.h"
#include "api/motor_reg.h"
#include "control/robot_internal.h"   /* LEESN_STAT_* 状态位定义 */
#include "config/robot_config.h"      /* DEG2STEPS */
#include "utils/logger.h"
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

/* ====================== 回零参数 ====================== */
typedef struct {
    int    speed_rpm;     /* 回零速度 rpm */
    int    accel_ms;      /* 加速时间 ms（0x0098，启动速度→目标速度） */
    int    decel_ms;      /* 减速时间 ms（0x0099，目标速度→停止速度） */
    int    dir;           /* 方向 +1正转 / -1反转 */
    int    stall_current; /* 堵转判定电流 mA，0=仅靠状态字判定 */
    double forward_deg;   /* 堵转后移动角度（绝对角度），0=不移动 */
} StallHome;

/* 堵转电流阈值 (mA) 为实机标定真值，关节1~5 顶死判定"电流>阈值+位置停涨=到位" */
static StallHome stall[6] = {
    [1] = { .speed_rpm = 100,  .accel_ms = 300, .decel_ms = 400, .dir = +1, .stall_current = 480, .forward_deg = -90.0 },
    [2] = { .speed_rpm = 60,   .accel_ms = 300, .decel_ms = 400, .dir = -1, .stall_current = 480, .forward_deg = +50.0 },  /* 降速回零，减轻顶死撞击 */
    [3] = { .speed_rpm = 100,  .accel_ms = 300, .decel_ms = 400, .dir = +1, .stall_current = 480, .forward_deg = -50.0 },
    [4] = { .speed_rpm = 60,   .accel_ms = 300, .decel_ms = 400, .dir = -1, .stall_current = 400, .forward_deg = +5.0 },
    [5] = { .speed_rpm = 100,  .accel_ms = 300, .decel_ms = 400, .dir = -1, .stall_current = 360, .forward_deg = +50.0 },
};

/* 关节6 传感器回零参数 */
static struct {
    int fast_rpm;         /* 快速寻找传感器 */
    int slow_rpm;         /* 碰到IN0后降速 */
    int crawl_rpm;        /* 极慢速离开IN0 */
    int dir;              /* 初始方向 -1反向 */
    int accel_ms;         /* 加减速时间 ms */
} sensor = { 300, 90, 30, -1, 100 };

static int    home_timeout_ms  = 60000;//设置回零超时时间

/* 堵转轮询采样周期 ms：顶死电流爬升极快，周期越短越早命中；
 * 实际节拍由读事务耗时(~7~10ms/轮)主导，已贴近 RS485 物理下限，再小无收益 */
#define HOME_STALL_POLL_MS  1

/* 传感器输入位（状态寄存器 0x0006 的 bit0=IN0, bit1=IN1） */
#define SENSOR_IN0  0x0001u
#define SENSOR_IN1  0x0002u

/* ====================== 传感器回零阶段 ====================== */

typedef enum {
    SEN_INIT,        /* 初始判定：按 IN0/IN1 当前状态选流程 */
    SEN_FWD_LEAVE,   /* 初始碰IN0：正向快速开3°（远离挡片再反向回找） */
    SEN_REV_FIND,    /* 反向快速找IN0（默认无传感器 / 开3°后回找） */
    SEN_FWD_FIND,    /* 正向快速找IN0（初始碰IN1 / 反向途中碰IN1 转来） */
    SEN_SLOW_LEAVE,  /* 慢速正向离开IN0（on→off 沿清零，即原点） */
    SEN_DONE         /* 完成 */
} SensorPhase;

/* ====================== 内部辅助函数 ====================== */

/* home_arm：使能电机并关硬限位（回零需顶限位，先关否则报警） */
static ErrCode home_arm(Robot *robot, int joint)
{
    ErrCode rc = motor_enable(robot, joint);
    if (rc != ERR_NONE) return rc;
    Sleep(50);
    return motor_set_limit(robot, joint, 0);  /* 关闭限位 */
}

/* home_restore：恢复使能/硬限位/超差报警/偏差预警默认值；单项失败仅告警不中断 */
static void home_restore(Robot *robot, int joint)
{
    if (motor_enable(robot, joint) != ERR_NONE) {
        LOG_WARN("关节%d 恢复使能失败", joint);
    }
    Sleep(30);
    if (motor_set_limit(robot, joint, 1) != ERR_NONE) {   /* 开启限位 */
        LOG_WARN("关节%d 恢复限位失败", joint);
    }
    /* 恢复超差报警：回零期间被关闭（0x000B/0x000C=0），
     * 结束后写回默认 200/100，恢复正常运动的超差保护 */
    if (motor_restore_pos_err_alarm(robot, joint) != ERR_NONE) {
        LOG_WARN("关节%d 恢复超差报警失败", joint);
    }
    /* 恢复位置偏差预警默认值 0x0010=20：回零期间被临时放宽(STALL_PREWARN_STEPS)，
     * 结束后写回，恢复正常运动的失步预警保护 */
    if (motor_set_pos_err_prewarn(robot, joint, 20) != ERR_NONE) {
        LOG_WARN("关节%d 恢复位置偏差预警失败", joint);
    }
}

/* home_stall_start：启动堵转回零 —— 关超差报警 → 设加速度 → 设速度 → 按方向运行 */

/* 位置停涨判据（脉冲≈0.05°）：正常运动每帧位移上千脉冲，仅顶死/停转可达 */
#define STALL_POS_STOP_DELTA  100

/* 回零期间把偏差预警 0x0010 放宽到 10000 步（不能写 0；默认 20 步会让顶死瞬间
 * bit10 抢先切断输出，电流判据来不及命中）；放宽后仍约 2s 预警兜底切断输出 */
#define STALL_PREWARN_STEPS   10000

/* 电流低于此值视为"输出已切断"：固件失步保护切断后电流归 0/读失败(-1 钳 0)，
 * 正常转动 440~470mA 远高于此；须配位置停涨+RUN_ACTIVE 排除运动帧误判 */
#define STALL_CUTOFF_MA       300

/* 各轴上一帧位置缓存（顶死检测：电流超阈值+位置停涨判定用） */
static int32_t s_stall_last_pos[7];
static int     s_stall_last_ok[7];

/* 【标定临时】各轴采样时刻与本次堵转启动时刻：
 *   周期 = 本帧与上一帧的时间差 → 真实轮询间隔，随并行轴数变化（单轴 ~7-10ms、
 *          整机 4~5 轴 30~50ms），是定窗口点数与死区的基准量；
 *   T+   = 启动后经过时间 → 识别加速段与顶死时刻，用于标定起步屏蔽窗与窗口时长。
 * 待 A3 窗口化改造时并入环形缓冲（每点同时记 pos/cur/时刻），届时移除本组变量。 */
static uint32_t s_last_tick_ms[7];
static uint32_t s_stall_t0_ms[7];

static void home_stall_start(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];
    s_stall_last_ok[joint] = 0;   /* 新一次堵转，清位置缓存 */
    s_last_tick_ms[joint]  = 0;   /* 清周期基准（首帧周期显示 0） */
    s_stall_t0_ms[joint]   = GetTickCount();   /* 【标定】起步时刻 */
    /* 关超差报警：顶死改由 home_check_stall 电流+停涨判定，不再报警锁存 */
    if (motor_disable_pos_err_alarm(robot, joint) != ERR_NONE) {
        LOG_WARN("关节%d 关闭超差报警失败，顶死仍会报警锁存", joint);
    }
    /* 放宽偏差预警到 STALL_PREWARN_STEPS（0x0010 写不了 0，默认 20 步会让顶死瞬间
     * bit10 抢先切断输出、电流判据来不及命中）；固件切断挡不住时由主判据B 立即兜底 */
    if (motor_set_pos_err_prewarn(robot, joint, STALL_PREWARN_STEPS) != ERR_NONE) {
        LOG_WARN("关节%d 放宽位置偏差预警失败，顶死仍会提前切断输出", joint);
    }
    motor_set_profile(robot, joint, p->accel_ms, p->decel_ms);
    /* Bug1：连续运行(0x00C8)速度源是 0x009A，与 0x00D8 不同源，须双写否则按记忆速度运行 */
    if (motor_set_speed16(robot, joint, p->speed_rpm) != ERR_NONE) {
        LOG_WARN("关节%d 写连续运行速度源0x009A失败，仍按驱动器记忆速度运行", joint);
    }
    motor_set_speed(robot, joint, p->speed_rpm);
    Sleep(20);
    motor_run(robot, joint, p->dir);
    LOG_INFO("关节%d 堵转启动 (方向=%+d, %drpm, 加速%dms/减速%dms, 阈值=%dmA)",
             joint, p->dir, p->speed_rpm, p->accel_ms, p->decel_ms, p->stall_current);
}

/* home_check_stall：判堵转到位。返回 1=命中 / 0=运行中。
 * 前置：已关 0x000B/C 报警、放宽 0x0010。判据分级兜底：
 * ①报警 bit21 ②超差 bit10 ③电流+位置停涨确认：A 超阈值&停涨（顶着型）/
 * B 电流归零或读失败&停涨&RUN_ACTIVE（固件切断型）；位置停涨排除运动帧误判。
 * 出参 cur_out：读到电流后立即写回本帧 mA(-1=读失败)，供主循环电流快照 */
static int home_check_stall(Robot *robot, int joint, int *cur_out)
{
    uint32_t st;
    int cur, cur_ok, pos_ok = 0, stopped;
    int32_t pos = 0, delta = 0;
    int threshold = stall[joint].stall_current;
    uint32_t now_ms, period_ms = 0, elaps_ms = 0;

    /* 【标定】周期 = 与上一帧间隔（随并行轴数变化）；T+ = 启动后经过时间 */
    now_ms = GetTickCount();
    if (s_last_tick_ms[joint] != 0) period_ms = now_ms - s_last_tick_ms[joint];
    s_last_tick_ms[joint] = now_ms;
    if (s_stall_t0_ms[joint] != 0)  elaps_ms  = now_ms - s_stall_t0_ms[joint];

    if (motor_read_status(robot, joint, &st) != ERR_NONE) return 0;
    cur_ok = 1;
    cur = motor_read_current(robot, joint);
    if (cur < 0 || cur > 3000) { cur_ok = 0; cur = 0; }
    if (cur_out) *cur_out = cur_ok ? cur : -1;

    /* 兜底1：驱动器报警（关报警未生效或其它报警） */
    if (st & LEESN_STAT_ALARM) {
        LOG_INFO("关节%d T+%ums 周期=%ums [兜底1-bit21] 驱动器报警，判定堵转到位",
                 joint, elaps_ms, period_ms);
        return 1;
    }
    /* 兜底2：位置超差（阈值寄存器写 0 未生效时仍会触发） */
    if (st & LEESN_STAT_OVERRUN) {
        LOG_INFO("关节%d T+%ums 周期=%ums [兜底2-bit10] 位置超差，判定堵转到位",
                 joint, elaps_ms, period_ms);
        return 1;
    }

    /* 读位置帧判停涨：顶死即挡停、先于电流变化出现，与电流判据组合排除偶发读失败误判 */
    pos = motor_read_position(robot, joint, &pos_ok);
    stopped = 0;
    if (pos_ok && s_stall_last_ok[joint]) {
        delta = pos - s_stall_last_pos[joint];
        stopped = (delta >= -STALL_POS_STOP_DELTA) &&
                  (delta <=  STALL_POS_STOP_DELTA);
    }

    /* 主判据A：电流顶高 + 位置停涨，双条件确认顶死（驱动器持续顶着型） */
    if (threshold > 0 && cur > threshold && stopped) {
        LOG_INFO("关节%d T+%ums 周期=%ums [A型] 电流超阈值(%dmA>%dmA)且位置停涨(Δ=%d)，判定堵转到位",
                 joint, elaps_ms, period_ms, cur, threshold,
                 pos_ok ? (int)delta : -99999);
        return 1;
    }

    /* 主判据B：固件失步保护切断输出型（切断比 0x0010 预警更早且参数挡不住，
     * 电流归 0/读失败 -1 均钳 0，只依赖 A/bit10 会无限轮询）；
     * 位置停涨+RUN_ACTIVE 已排除运动帧读失败误判 */
    if (cur < STALL_CUTOFF_MA && stopped &&
        (st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) {
        LOG_INFO("关节%d T+%ums 周期=%ums [B型] 电流归零/读失败(%smA<%dmA)且位置停涨(Δ=%d)，判定到位(输出被切断)",
                 joint, elaps_ms, period_ms, cur_ok ? "0" : "ERR",
                 STALL_CUTOFF_MA, pos_ok ? (int)delta : -99999);
        return 1;
    }

    /* 未命中：更新位置缓存，供下一帧停涨判定 */
    if (pos_ok) {
        s_stall_last_pos[joint] = pos;
        s_stall_last_ok[joint] = 1;
    }

    LOG_INFO("关节%d T+%ums 周期=%ums 状态=0x%08X 电流=%dmA 位置=%d 帧差=%d (阈值=%dmA)",
             joint, elaps_ms, period_ms, (unsigned)st, cur_ok ? cur : -1,
             pos_ok ? (int)pos : -99999,
             (pos_ok && s_stall_last_ok[joint]) ? (int)delta : -99999,
             threshold);
    return 0;
}

/* home_stall_done：堵转收尾 —— 压短减速退出连续运行 → 兜底清报警 → 清零位置
 * 顶死时连续运行命令仍在、直接 move_abs 会被忽略，须先退出（压短减速防持续压紧报警） */
static void home_stall_done(Robot *robot, int joint)
{
    int pos_ok = 0;
    int32_t pos;
    int clear_ok;
    int alarm;
    StallHome *p = &stall[joint];

    /* 退出连续运行：否则绝对定位(0x00E8)不执行；压短减速再停，防按原减速持续压紧报警 */
    if (motor_set_profile(robot, joint, p->accel_ms, 50) != ERR_NONE) {
        LOG_WARN("关节%d 临时压短减速时间失败，仍按原减速停止", joint);
    }
    if (motor_stop_slow(robot, joint) != ERR_NONE) {
        LOG_WARN("关节%d 减速停止失败，后续退让 move_abs 可能被忽略", joint);
    }
    Sleep(150);
    if (motor_set_profile(robot, joint, p->accel_ms, p->decel_ms) != ERR_NONE) {
        LOG_WARN("关节%d 恢复减速时间失败，后续 movej 减速将偏短", joint);
    }

    /* 兜底：先清报警再清零（read_alarm 返回 -1 为读失败，仅处理 >0 真报警） */
    alarm = motor_read_alarm(robot, joint);
    if (alarm > 0) {
        int retry, cleared = 0;
        LOG_WARN("关节%d 检测到驱动器报警 码=%d", joint, alarm);
        for (retry = 0; retry < 3; retry++) {
            if (motor_clear_alarm(robot, joint) == ERR_NONE) {
                cleared = 1;
                break;
            }
            Sleep(50);   /* 清报警重试间隔 */
        }
        if (cleared) {
            LOG_INFO("关节%d 已清除驱动器报警（原码=%d）", joint, alarm);
        } else {
            LOG_WARN("关节%d 清除驱动器报警失败，清零可能被驱动器拒绝", joint);
        }
        Sleep(10);
    }

    /* 清零：当前位置置 0，闭环目标=当前位置，驱动器停止顶着。
     * 注：0x00D2 为【无记忆】RAM 寄存器，零点不跨断电保持，上电须重新回零。
     * 禁止在此追加 motor_save_params(0x00DC=1)——既存不住零点，又会把回零期
     * 临时关闭的报警/限位固化进 flash，理由见 motor_reg.h 声明处。 */
    clear_ok = (motor_clear_pos(robot, joint) == ERR_NONE);
    Sleep(30);
    pos = motor_read_position(robot, joint, &pos_ok);
    if (!clear_ok) {
        LOG_ERROR("关节%d 堵转清零失败，后续退让 move_abs 将基于旧零点，位置会错", joint);
    } else {
        LOG_INFO("关节%d 堵转清零成功，位置=%d",
                 joint, pos_ok ? (int)pos : -99999);
    }
}

/* home_wait_inpos：等待运动到位，须"曾 RUN_ACTIVE 且现退出、无报警"，
 * 防未启动/报警停机误判。返回 1=到位 / 0=超时 / -1=报警异常 */
static int home_wait_inpos(Robot *robot, int joint, int timeout_ms)
{
    uint32_t start_ms = GetTickCount();
    int saw_active = 0;

    while ((GetTickCount() - start_ms) < (uint32_t)timeout_ms) {
        uint32_t st;
        if (motor_read_status(robot, joint, &st) != ERR_NONE) {
            Sleep(200); continue;
        }
        if (st & LEESN_STAT_ALARM) return -1;
        if ((st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) {
            saw_active = 1;
            Sleep(200);
            continue;
        }
        if (saw_active) return 1;   /* 曾启动、现已退出且无报警 → 到位 */
        Sleep(200);                 /* 未见 ACTIVE：启动延迟，继续等待 */
    }
    return 0;
}

/* home_wait_stall：轮询等堵转命中（home_check_stall）。返回 1=命中 / 0=超时 */
static int home_wait_stall(Robot *robot, int joint, int timeout_ms)
{
    uint32_t start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)timeout_ms) {
        if (home_check_stall(robot, joint, NULL)) return 1;
        Sleep(HOME_STALL_POLL_MS);
    }
    return 0;
}

/* home_stall_forward：堵转清零后退让到 forward_deg（仅非 0 轴），避免久顶硬限位 */
static void home_stall_forward(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];

    if (p->forward_deg == 0.0) return;
    LOG_INFO("关节%d 堵转后移动到 %.1f°", joint, p->forward_deg);
    robot_movej(robot, joint, p->forward_deg, (double)p->speed_rpm);
    {
        int rc = home_wait_inpos(robot, joint, home_timeout_ms);
        if (rc == 0) {
            LOG_WARN("关节%d 退让到 %.1f° 超时", joint, p->forward_deg);
        } else if (rc < 0) {
            LOG_WARN("关节%d 退让到 %.1f° 报警/异常", joint, p->forward_deg);
        }
    }
}

/* ====================== 关节6：传感器回零 ====================== */

/* sensor_phase_name：阶段名（用于日志） */
static const char *sensor_phase_name(SensorPhase ph)
{
    static const char *names[] = {
        "初始判定", "正向开3°", "反向快找", "正向快找", "慢速离开", "完成"
    };
    if (ph < 0 || ph >= (int)(sizeof(names) / sizeof(names[0]))) return "未知";
    return names[ph];
}

/* SensorCtx：关节6 传感器回零上下文。
 * start/tick/finish 拆分使传感器回零可与堵转轮询共用主循环并行推进 */
typedef struct {
    SensorPhase ph;          /* 当前状态机阶段 */
    int in0_was_on;          /* SEN_SLOW_LEAVE：记录 IN0 曾为 on（等待 on->off 沿） */
    int32_t pos_base;        /* SEN_FWD_LEAVE：正向开 3 度的起始位置 */
    uint32_t start_ms;       /* 总超时起点 */
    uint32_t leave_start;    /* SEN_SLOW_LEAVE 起点（单独 10s 超时） */
    int done;                /* 已过 on->off 沿，待清零 */
    int failed;              /* 失败/超时（已急停） */
    int last_cur;            /* 最近一帧读到的电流 mA（供主循环电流快照） */
} SensorCtx;

/* sensor6_set_rpm：双写 0x009A+0x00D8 速度源（Bug1：连续运行只认 0x009A，
 * 单写 0x00D8 会按驱动器记忆速度运行） */
static void sensor6_set_rpm(Robot *robot, int rpm)
{
    if (motor_set_speed16(robot, 6, rpm) != ERR_NONE) {
        LOG_WARN("关节6 写连续运行速度源0x009A(%drpm)失败，按记忆速度运行", rpm);
    }
    motor_set_speed(robot, 6, rpm);
}

/* sensor6_start：启动 -- arm + profile，记录超时起点，回到 SEN_INIT */
static ErrCode sensor6_start(Robot *robot, SensorCtx *c)
{
    ErrCode rc;

    c->ph = SEN_INIT;
    c->in0_was_on = 0;
    c->pos_base = 0;
    c->start_ms = 0;
    c->leave_start = 0;
    c->done = 0;
    c->failed = 0;
    c->last_cur = 0;

    rc = home_arm(robot, 6);
    if (rc != ERR_NONE) {
        LOG_WARN("关节6 arm 失败：%s", err_str(rc));
        return rc;
    }
    motor_set_profile(robot, 6, sensor.accel_ms, sensor.accel_ms);

    LOG_INFO("关节6 传感器回零启动 (快速%drpm/慢速%drpm/极慢%drpm)",
             sensor.fast_rpm, sensor.slow_rpm, sensor.crawl_rpm);
    c->start_ms = GetTickCount();
    return ERR_NONE;
}

/* sensor6_tick：推进传感器回零状态机一帧。返回 0=继续 / 1=完成(IN0 on→off 沿已过，
 * 已急停，清零在 sensor6_finish) / -1=失败超时(已急停)。零点=IN0 正向离开沿。
 * 三种初始情形：碰 IN0(正开3°再反找)/碰 IN1(正找)/无传感器(反找，先碰 IN1 转情形2)。
 * 寻的 fast_rpm、离沿 crawl_rpm 低速保精度 */
static int sensor6_tick(Robot *robot, SensorCtx *c)
{
    SensorPhase ph = c->ph;
    uint32_t inputs;
    int in0, in1, cur, pos_ok = 0;
    int32_t pos;
    uint32_t now = GetTickCount();

    if (c->done)   return 1;
    if (c->failed) return -1;

    /* 超时检测：离开沿阶段单独 10s，其余用总超时 */
    if (ph == SEN_SLOW_LEAVE && c->leave_start > 0 &&
        (now - c->leave_start) >= 10000) {
        LOG_WARN("关节6 慢速离开IN0超时");
        c->failed = 1;
        motor_estop(robot, 6);
        return -1;
    }
    if (ph != SEN_SLOW_LEAVE && c->start_ms > 0 &&
        (now - c->start_ms) >= (uint32_t)home_timeout_ms) {
        LOG_WARN("关节6 传感器回零超时");
        c->failed = 1;
        motor_estop(robot, 6);
        return -1;
    }

    /* 读传感器状态 */
    if (motor_read_status(robot, 6, &inputs) != ERR_NONE) return 0;
    in0 = (inputs & SENSOR_IN0) ? 1 : 0;
    in1 = (inputs & SENSOR_IN1) ? 1 : 0;
    cur = motor_read_current(robot, 6);
    if (cur < 0 || cur > 3000) cur = 0;
    c->last_cur = cur;
    pos = motor_read_position(robot, 6, &pos_ok);

    LOG_INFO("关节6 IN0=%d IN1=%d 电流=%dmA 位置=%d 阶段=%s",
             in0, in1, cur, pos_ok ? (int)pos : -99999, sensor_phase_name(ph));

    /* 状态机切换 */
    switch (ph) {
    case SEN_INIT:
        if (in0) {
            /* 情况1：初始就在IN0上，正向快速开3度（离开挡片） */
            LOG_INFO("关节6 初始碰IN0（情况1），正向快速开3度");
            c->pos_base = pos_ok ? pos : 0;
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_LEAVE;
        } else if (in1) {
            /* 情况2：初始碰IN1，正向快速找IN0 */
            LOG_INFO("关节6 初始碰IN1（情况2），正向快速找IN0");
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_FIND;
        } else {
            /* 情况3：初始无传感器，反向快速找IN0 */
            LOG_INFO("关节6 初始无传感器（情况3），反向快速找IN0");
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, sensor.dir);
            c->ph = SEN_REV_FIND;
        }
        break;

    case SEN_FWD_LEAVE:
        /* 正向快速开3度：位移到位后反向快速回找 IN0 */
        if (pos_ok && (pos - c->pos_base) >= DEG2STEPS(3.0, 50)) {
            LOG_INFO("关节6 正向开3度到位，反向快速回找IN0");
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, sensor.dir);
            c->ph = SEN_REV_FIND;
        }
        break;

    case SEN_REV_FIND:
        if (in0) {
            /* 反向途中碰到IN0：改慢速正向离开（on->off 清零） */
            LOG_INFO("关节6 反向碰到IN0，慢速正向离开 (%drpm)", sensor.crawl_rpm);
            sensor6_set_rpm(robot, sensor.crawl_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->in0_was_on = 1;
            c->leave_start = GetTickCount();
            c->ph = SEN_SLOW_LEAVE;
        } else if (in1) {
            /* 反向途中先碰IN1：转入情况2（正向快速找IN0） */
            LOG_INFO("关节6 反向碰到IN1，转正向快速找IN0（同情况2）");
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_FIND;
        }
        break;

    case SEN_FWD_FIND:
        if (in0) {
            /* 正向碰到IN0：改慢速正向离开（穿过挡片到正向沿清零） */
            LOG_INFO("关节6 正向碰到IN0，慢速正向离开 (%drpm)", sensor.crawl_rpm);
            sensor6_set_rpm(robot, sensor.crawl_rpm);
            c->in0_was_on = 1;
            c->leave_start = GetTickCount();
            c->ph = SEN_SLOW_LEAVE;
        }
        break;

    case SEN_SLOW_LEAVE:
        if (c->in0_was_on && !in0) {
            /* 慢速离开IN0（on->off 沿），急停并清零，完成 */
            LOG_INFO("关节6 慢速离开IN0，回零完成");
            motor_estop(robot, 6);
            Sleep(50);
            c->ph = SEN_DONE;
            c->done = 1;
            return 1;
        }
        break;

    default:
        break;
    }

    return 0;
}

/* sensor6_finish：收尾 -- 成功则清零校验，失败则急停兜底；统一恢复 6 轴限位 */
static void sensor6_finish(Robot *robot, SensorCtx *c)
{
    if (c->done) {
        int pos_ok = 0;
        int32_t pos;
        /* 清零：IN0 离开沿即原点。注意 0x00D2 为【无记忆】RAM 寄存器，
         * 零点不跨断电保持，上电须重新回零；此处不得追加
         * motor_save_params(0x00DC=1)，理由见 motor_reg.h 声明处。 */
        int clear_ok = (motor_clear_pos(robot, 6) == ERR_NONE);
        Sleep(30);
        pos = motor_read_position(robot, 6, &pos_ok);
        LOG_INFO("关节6 位置清零%s，位置=%d",
                 clear_ok ? "成功" : "失败", pos_ok ? (int)pos : -99999);
    } else {
        /* 主循环整体超时等未走 tick 超时分支的兜底 */
        if (!c->failed) {
            c->failed = 1;
            motor_estop(robot, 6);
        }
        LOG_WARN("关节6 传感器回零失败：%s", err_str(ERR_TIMEOUT));
    }

    home_restore(robot, 6);
    if (c->done) LOG_INFO("关节6 回零完成");
}

/* home_joint6：关节6 单轴传感器回零 = sensor6_start → 循环 tick → sensor6_finish */
static ErrCode home_joint6(Robot *robot)
{
    SensorCtx c;
    ErrCode rc = sensor6_start(robot, &c);
    if (rc != ERR_NONE) return rc;

    while (!c.done && !c.failed) {
        if (sensor6_tick(robot, &c) != 0) break;
        Sleep(HOME_STALL_POLL_MS);
    }
    sensor6_finish(robot, &c);
    return c.done ? ERR_NONE : ERR_TIMEOUT;
}

/* 转角到位位置容差（脉冲）：闭环步进到位后实际位置应等于命令目标；
 * 给 ±100 步抖动/量化余量（约 0.09°~0.18°，视减速比），远小于
 * "指令发出却未运动 / 半途停住"的位置差，可区分真到位与假到位。 */
#define HOME_INPOS_TOL_STEPS   100

/* home_inpos_query：校验 movej 真到位 = 无报警+退出运行+位置在目标±容差，
 * 防"指令发出但没转/半途停住"误判。返回 1=到位 / 0=未到位 / -1=报警超差 / -2=读失败 */
static int home_inpos_query(Robot *robot, int joint, int32_t target_steps)
{
    uint32_t st;
    int32_t pos;
    int pos_ok;

    if (motor_read_status(robot, joint, &st) != ERR_NONE) return -2;
    if (st & (LEESN_STAT_ALARM | LEESN_STAT_OVERRUN)) return -1;
    if ((st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) return 0;

    pos = motor_read_position(robot, joint, &pos_ok);
    if (!pos_ok) return -2;
    if (pos >= target_steps - HOME_INPOS_TOL_STEPS &&
        pos <= target_steps + HOME_INPOS_TOL_STEPS) {
        return 1;
    }
    return 0;
}

/* ====================== 公开接口 ====================== */

/* home_goto_pose：全轴回零后统一 movej 到 forward_deg。
 * 先一次性发全部 movej 再统一轮询；单轴异常只结算自身不拖累其余轴 */
static void home_goto_pose(Robot *robot)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    uint8_t pend[7] = {0};   /* 1~5：已发 movej、待等待到位 */
    int32_t tgt[7] = {0};    /* 1~5：期望绝对脉冲 */
    uint32_t start_ms;
    int remain = 0, j;

    /* 第一遍：全部发出 movej（各轴同时开始运动） */
    for (j = 1; j <= 5; j++) {
        StallHome *p = &stall[j];
        if (robot_is_masked(robot, j)) continue;
        if (p->forward_deg == 0.0) continue;

        /* 发指令前快照状态/位置并暴露异常，防电机没动却被轮询误判为到位 */
        tgt[j] = DEG2STEPS(p->forward_deg, reductions[j - 1]);
        {
            uint32_t st;
            int32_t pos;
            int pos_ok, st_ok;
            st_ok = (motor_read_status(robot, j, &st) == ERR_NONE);
            pos = motor_read_position(robot, j, &pos_ok);
            if (st_ok && pos_ok) {
                LOG_INFO("关节%d movej 前: 状态=0x%08X 报警=%d 位置=%d 目标=%d 差=%d",
                         j, (unsigned)st, motor_read_alarm(robot, j), (int)pos,
                         (int)tgt[j], (int)(tgt[j] - pos));
            } else {
                LOG_WARN("关节%d movej 前: 状态读%s 位置读%s，无法确认就绪",
                         j, st_ok ? "OK" : "失败", pos_ok ? "OK" : "失败");
            }
        }
        LOG_INFO("关节%d movej 到 %.1f°", j, p->forward_deg);
        if (robot_movej(robot, j, p->forward_deg, (double)p->speed_rpm) != ERR_NONE) {
            LOG_WARN("关节%d 发 movej 失败", j);
            continue;
        }
        pend[j] = 1;
        remain++;
    }
    if (remain == 0) return;

    /* 第二遍：统一轮询等待，直到全部结算 */
    start_ms = GetTickCount();
    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= (uint32_t)home_timeout_ms) {
            for (j = 1; j <= 5; j++) {
                int32_t pos;
                int pos_ok;
                if (!pend[j]) continue;
                pos = motor_read_position(robot, j, &pos_ok);
                LOG_WARN("关节%d 转角到 %.1f° 超时（目标=%d 位置=%d 差=%d）",
                         j, stall[j].forward_deg, (int)tgt[j],
                         pos_ok ? (int)pos : -99999,
                         pos_ok ? (int)(tgt[j] - pos) : -99999);
                pend[j] = 0;
            }
            break;
        }
        for (j = 1; j <= 5; j++) {
            int q;
            if (!pend[j]) continue;
            q = home_inpos_query(robot, j, tgt[j]);
            if (q == 1) {
                int32_t pos;
                int pos_ok;
                pos = motor_read_position(robot, j, &pos_ok);
                LOG_INFO("关节%d 到位（%.1f° 位置=%d）", j, stall[j].forward_deg,
                         pos_ok ? (int)pos : -99999);
                pend[j] = 0; remain--;
            } else if (q == -1) {
                LOG_WARN("关节%d 转角到 %.1f° 报警/异常，已结算该轴",
                         j, stall[j].forward_deg);
                pend[j] = 0; remain--;
            } else if (q == -2) {
                Sleep(HOME_STALL_POLL_MS);   /* 读失败：避让一帧下轮再试 */
            }
        }
        if (remain > 0) Sleep(HOME_STALL_POLL_MS);
    }
}

/* robot_home：整机回零（并行 + 事件联动）
 * ①{1,2,3,5}堵转与 6 传感器并行归零 ②3 号到位后启动 4 号（依赖 3 停稳防干涉）
 * ③全轴成功后统一转角 forward_deg，有失败轴则跳过 */
ErrCode robot_home(Robot *robot)
{
    const int group_stall[] = {1, 2, 3, 5};
    uint8_t active[7] = {0};    /* 已启动参与回零的轴 */
    uint8_t sdone[7] = {0};     /* 已回零成功 */
    uint8_t sfail[7] = {0};     /* 回零失败/超时 */
    SensorCtx c6;
    uint32_t start_ms;
    int j, all_ok;
    int cur_now[7] = {-1, -1, -1, -1, -1, -1, -1};  /* 1~6 本帧电流 mA；-1=未启动/已完成/读取失败 */

    if (robot == NULL) return ERR_ARG;

    /* 打印参数总览 */
    LOG_INFO("=== 回零参数 ===");
    for (j = 1; j <= 5; j++) {
        if (robot_is_masked(robot, j)) {
            LOG_INFO("  关节%d: 已屏蔽", j);
        } else {
            LOG_INFO("  关节%d: 堵转 %drpm 加速%dms/减速%dms 方向=%+d 阈值=%dmA 正向=%.1f°",
                     j, stall[j].speed_rpm, stall[j].accel_ms, stall[j].decel_ms,
                     stall[j].dir, stall[j].stall_current, stall[j].forward_deg);
        }
    }
    if (!robot_is_masked(robot, 6)) {
        LOG_INFO("  关节6: 传感器 快%d/慢%d/极慢%drpm 方向=%+d",
                 sensor.fast_rpm, sensor.slow_rpm, sensor.crawl_rpm, sensor.dir);
    }
    LOG_INFO("================");

    /* 阶段1：{1,2,3,5} 并行堵转归零 与 关节6 传感器回零 同时启动 */
    LOG_INFO("回零：{1,2,3,5} 堵转 + 关节6 传感器 并行归零...");
    for (int gi = 0; gi < (int)(sizeof(group_stall) / sizeof(group_stall[0])); gi++) {
        int jj = group_stall[gi];
        if (robot_is_masked(robot, jj)) continue;
        if (home_arm(robot, jj) != ERR_NONE) {
            LOG_WARN("关节%d arm 失败，跳过", jj);
            continue;
        }
        home_stall_start(robot, jj);
        active[jj] = 1;
        Sleep(10);
    }
    if (!robot_is_masked(robot, 6)) {
        if (sensor6_start(robot, &c6) == ERR_NONE) {
            active[6] = 1;
        }
    }

    /* 主循环：堵转组与 6 传感器并行推进；3 号到位即联动启动 4 号 */
    start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)home_timeout_ms) {
        int all_done = 1;

        /* 每轮电流快照复位：仅本帧真正读到电流的轴显示数值，其余显示 '-' */
        for (j = 1; j <= 6; j++) cur_now[j] = -1;

        /* 6 轴传感器推进一帧 */
        if (active[6] && !sdone[6] && !sfail[6]) {
            int r = sensor6_tick(robot, &c6);
            if (r == 1)       sdone[6] = 1;
            else if (r == -1) sfail[6] = 1;
            else              all_done = 0;
            cur_now[6] = c6.last_cur;   /* tick 帧内已更新 last_cur */
        }

        /* 堵转组轮询（4 号由 3 号到位事件加入 active） */
        for (j = 1; j <= 5; j++) {
            if (!active[j] || sdone[j] || sfail[j]) continue;
            all_done = 0;
            if (!home_check_stall(robot, j, &cur_now[j])) continue;
            home_stall_done(robot, j);
            sdone[j] = 1;
            LOG_INFO("关节%d 堵转回零完成", j);
            if (j == 3 && !active[4] && !robot_is_masked(robot, 4)) {
                /* 阶段2：3 号到位 -> 立即启动 4 号 */
                if (home_arm(robot, 4) == ERR_NONE) {
                    home_stall_start(robot, 4);
                    active[4] = 1;
                    LOG_INFO("关节3 到位，立即启动关节4 堵转归零");
                } else {
                    LOG_WARN("关节4 arm 失败，跳过（未归零）");
                }
            }
        }

        /* 每轮六轴电流统一快照（- = 未启动/已完成/读取失败）
         * 【标定】附带在跑轴数：轮询周期随该数变化（1 轴 ~7-10ms / 5 轴 30~50ms），
         * 日志里能直接把"周期变化"与"轴数减少"对上，便于标定窗口点数与死区 */
        {
            char snap[160];
            size_t off = 0;
            int nrun = 0;
            for (j = 1; j <= 6; j++) {
                if (active[j] && !sdone[j] && !sfail[j]) nrun++;
            }
            off += (size_t)snprintf(snap + off, sizeof(snap) - off,
                                    "实时电流[在跑%d轴]", nrun);
            for (j = 1; j <= 6; j++) {
                if (cur_now[j] < 0)
                    off += (size_t)snprintf(snap + off, sizeof(snap) - off, " %d=-", j);
                else
                    off += (size_t)snprintf(snap + off, sizeof(snap) - off, " %d=%dmA", j, cur_now[j]);
            }
            LOG_INFO("%s", snap);
        }

        if (all_done) break;
        Sleep(HOME_STALL_POLL_MS);
    }

    /* 超时兜底：未完成堵转轴急停并标记失败 */
    for (j = 1; j <= 5; j++) {
        if (!active[j] || sdone[j]) continue;
        LOG_WARN("关节%d 堵转回零超时", j);
        motor_estop(robot, j);
        sfail[j] = 1;
    }
    if (active[6] && !sdone[6]) {
        sfail[6] = 1;   /* 未在 tick 超时分支收尾的兜底 */
    }
    if (active[6]) {
        sensor6_finish(robot, &c6);   /* done->清零校验；失败->急停；统一恢复 6 轴限位 */
    }

    /* 阶段3：全轴回零成功后统一转角，有失败轴则跳过防乱跑 */
    all_ok = 1;
    for (j = 1; j <= 6; j++) {
        if (robot_is_masked(robot, j)) continue;
        if (!active[j] || sfail[j]) {
            all_ok = 0;
            break;
        }
    }
    if (all_ok) {
        LOG_INFO("回零：全部轴归零完成，统一转角到 forward_deg...");
        home_goto_pose(robot);
    } else {
        LOG_WARN("回零：存在未归零/失败轴，跳过统一转角");
    }

    /* 恢复限位 */
    for (j = 1; j <= 5; j++) {
        if (robot_is_masked(robot, j)) continue;
        home_restore(robot, j);
        Sleep(20);
    }

    LOG_INFO("回零流程结束");
    return ERR_NONE;
}

/* robot_home_single：单轴独立回零（home:N）：arm→堵转→判到位→清零→退让 forward_deg；
 * 关节6 走传感器状态机；只操作目标轴 */
ErrCode robot_home_single(Robot *robot, int joint)
{
    StallHome *p;
    ErrCode rc;
    int hit;

    if (robot == NULL) return ERR_ARG;
    if (joint < 1 || joint > 6) return ERR_ARG;
    if (robot_is_masked(robot, joint)) return ERR_MASKED;

    /* 关节6 是传感器回零轴（不在 stall 堵转表内），走专用传感器状态机 */
    if (joint == 6) {
        LOG_INFO("=== 单轴传感器回零：关节6 ===");
        return home_joint6(robot);
    }
    p = &stall[joint];

    LOG_INFO("=== 单轴独立堵转回零：关节%d ===", joint);
    rc = home_arm(robot, joint);
    if (rc != ERR_NONE) {
        LOG_WARN("关节%d arm 失败：%s", joint, err_str(rc));
        return rc;
    }

    home_stall_start(robot, joint);

    /* 等待堵转到位 */
    hit = home_wait_stall(robot, joint, home_timeout_ms);
    if (!hit) {
        LOG_WARN("关节%d 堵转回零超时", joint);
        motor_estop(robot, joint);
        home_restore(robot, joint);
        return ERR_TIMEOUT;
    }
    home_stall_done(robot, joint);

    /* 堵转成功 → 自动跑到配置角度 forward_deg（0 则停在清零点） */
    home_stall_forward(robot, joint);

    home_restore(robot, joint);
    LOG_INFO("关节%d 单轴回零完成（目标 %.1f°）", joint, p->forward_deg);
    return ERR_NONE;
}

/* robot_home_joint：屏蔽其余轴 → robot_home 只回零目标轴 → movej 到 angle_deg → 恢复屏蔽 */
ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm)
{
    int saved[6];
    int j;
    ErrCode rc = ERR_NONE;

    if (robot == NULL) return ERR_ARG;
    if (joint < 1 || joint > 6) return ERR_ARG;
    if (robot_is_masked(robot, joint)) return ERR_MASKED;

    LOG_INFO("单轴回零：关节%d 回零后自动运动到 %.1f° ...", joint, angle_deg);

    /* 保存并临时屏蔽其余轴，使 robot_home 只回零目标轴 */
    for (j = 1; j <= 6; j++) {
        saved[j - 1] = robot_is_masked(robot, j);
        if (j != joint) {
            robot_mask(robot, j);
        }
    }

    /* 回零（只动目标轴） */
    rc = robot_home(robot);

    /* 回零成功 → 自动运动到目标角度 */
    if (rc == ERR_NONE) {
        double spd = (speed_rpm > 0.0) ? speed_rpm : (double)stall[joint].speed_rpm;

        rc = robot_movej(robot, joint, angle_deg, spd);
        if (rc != ERR_NONE) {
            LOG_ERROR("关节%d 自动运动到 %.1f° 失败：%s",
                      joint, angle_deg, err_str(rc));
        } else {
            LOG_INFO("关节%d 自动运动到 %.1f° ...", joint, angle_deg);
            {
                int wr = home_wait_inpos(robot, joint, home_timeout_ms);
                if (wr == 1) {
                    LOG_INFO("关节%d 到位（%.1f°）", joint, angle_deg);
                } else if (wr == 0) {
                    LOG_WARN("关节%d 运动到 %.1f° 超时", joint, angle_deg);
                    rc = ERR_TIMEOUT;
                } else {
                    LOG_WARN("关节%d 运动到 %.1f° 报警/异常", joint, angle_deg);
                    rc = ERR_ALARM;
                }
            }
        }
    }

    /* 恢复原屏蔽状态 */
    for (j = 1; j <= 6; j++) {
        if (saved[j - 1]) {
            robot_mask(robot, j);
        } else {
            robot_unmask(robot, j);
        }
    }

    LOG_INFO("关节%d 回零并运动到 %.1f°：%s",
             joint, angle_deg, rc == ERR_NONE ? "完成" : err_str(rc));
    return rc;
}
