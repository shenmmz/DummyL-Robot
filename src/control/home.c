/*
 * home.c —— 回零实现（堵转模式 + 传感器模式）
 * ------------------------------------------------------------
 * 关节1-5：堵转回零（速度模式顶硬限位 → 电流/输出切断判据 → 清零位置；
 *           forward_deg!=0 的轴堵转清零后立即退让到该角，如 1 轴收 -90.0°）
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
typedef struct {
    int    speed_rpm;     /* 回零速度 rpm */
    int    accel_ms;      /* 加速时间 ms（0x0098，启动速度→目标速度） */
    int    decel_ms;      /* 减速时间 ms（0x0099，目标速度→停止速度） */
    int    dir;           /* 方向 +1正转 / -1反转 */
    int    stall_current; /* 堵转判定电流 mA，0=仅靠状态字判定 */
    double forward_deg;   /* 堵转后移动角度（绝对角度），0=不移动 */
    int    cur_only;      /* 1=电流超过阈值即判定堵转（不要求位置停涨） */
} StallHome;

static StallHome stall[6] = {
    [1] = { .speed_rpm = 80,  .accel_ms = 300, .decel_ms = 400, .dir = +1, .stall_current = 490, .forward_deg = -180.0 },
    [2] = { .speed_rpm = 80,  .accel_ms = 300, .decel_ms = 400, .dir = -1, .stall_current = 500, .forward_deg = +20.0 },
    [3] = { .speed_rpm = 80,  .accel_ms = 300, .decel_ms = 400, .dir = +1, .stall_current = 500, .forward_deg = -20.0 },
    [4] = { .speed_rpm = 60,  .accel_ms = 500, .decel_ms = 500, .dir = -1, .stall_current = 400, .forward_deg = +5.0, .cur_only = 1 },
    [5] = { .speed_rpm = 100, .accel_ms = 200, .decel_ms = 200, .dir = -1, .stall_current = 360, .forward_deg = +20.0 },
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

/* 堵转轮询采样周期 ms：顶死瞬间电流爬升极快（实测可在一帧内从 ~460mA
 * 冲到 2000mA+），周期越短越能早于报警/大电流命中。
 * 回零期间后台监控线程已暂停（main.c），总线由堵转采样独占；
 * 命中路径每轮 2 帧(状态+电流)约 4~8ms @115200。
 * Sleep(1) 仅作最小间隙：每轮真实节拍仍由读事务耗时(~6~9ms)主导，
 * 实际约 7~10ms/轮，已贴近 RS485@115200 物理下限，再小无收益。 */
#define HOME_STALL_POLL_MS  1

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

/* home_restore：回零后恢复 —— 保持使能 + 开启硬限位 + 恢复超差报警默认值。
 * 逐项恢复，任一项失败仅告警，不中断后续恢复（使能失败也继续尝试恢复其它项）。 */
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
/* 位置停涨判据：位置帧增量低于该值视为已顶死停住（脉冲）。
 * 4000 脉冲/电机转口径下，100 脉冲 = 0.025 电机转 = 输出轴约 0.18°(50:1)；
 * 正常运行每 10ms 帧位移上千脉冲，此阈值远低于正常值，仅顶死/停转可达。 */
#define STALL_POS_STOP_DELTA  100

/* 回零期间位置偏差预警值（0x0010，值域 1~65535，无法写 0，默认 20）
 * 顶死时序：偏差超 0x0010(默认20步)→bit10 置位并切断输出(电流归0)，
 *           偏差超 0x000B/C→bit21 报警锁存。
 * 0x000B/C 可写 0 取消，但 0x0010 不能——顶死瞬间几毫秒就超 20 步，
 * 电流根本没时间爬过阈值，主判据永远抢不过 bit10。
 * 故回零期间临时放宽到 10000 步（≈2s @80rpm×4000pul/rev）：
 * 顶死后驱动器保持输出，电流爬升被 home_check_stall 主判据命中；
 * 即使检测失效，约 2s 后仍会预警切断输出兜底，不会长时间大电流顶死。 */
#define STALL_PREWARN_STEPS   10000

/* 顶死"输出切断"判据电流上限（mA）：
 * 实测顶死序列：接触限位→电流短暂爬高(如1111mA)→固件失步保护切断输出→
 * 电流归 0（实测该帧 0x001A 可能读失败返回 -1，与真 0 同样钳为 0 显示）、
 * 位置停涨(±1 抖动)、状态字仍 RUN_ACTIVE。该固件切断比 0x0010 预警更早且
 * 参数挡不住；正常转动电流 440~470mA 远高于此值。读失败/归零/小电流统一
 * 视为"输出已切断"，靠"位置停涨 + RUN_ACTIVE"排除运动帧误判。 */
#define STALL_CUTOFF_MA       300

/* 各轴上一帧位置缓存（顶死检测：电流顶高+位置停涨判定用） */
static int32_t s_stall_last_pos[7];
static int     s_stall_last_ok[7];

static void home_stall_start(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];
    s_stall_last_ok[joint] = 0;   /* 新一次堵转，清位置缓存 */
    /* 关闭超差报警：顶死不再报警锁存/切断输出，改为持续顶出电流，
     * 由 home_check_stall 用"电流超阈值+位置停涨"判定 */
    if (motor_disable_pos_err_alarm(robot, joint) != ERR_NONE) {
        LOG_WARN("关节%d 关闭超差报警失败，顶死仍会报警锁存", joint);
    }
    /* 放宽位置偏差预警 0x0010（默认 20 step 无法写 0）：
     * 顶死瞬间偏差几毫秒就超 20 步 → bit10 置位并切断输出(电流归0)，
     * 主判据A"电流超阈值"永远来不及命中；调大到 STALL_PREWARN_STEPS 后
     * 顶死瞬间驱动器保持输出、电流可短暂爬高(实测约 636mA 后仍被固件
     * 失步保护切断归 0——该切断挡不住，由主判据B"电流归零+位置停涨"立即
     * 判定到位，故放宽无风险，仅作安全网延迟)。 */
    if (motor_set_pos_err_prewarn(robot, joint, STALL_PREWARN_STEPS) != ERR_NONE) {
        LOG_WARN("关节%d 放宽位置偏差预警失败，顶死仍会提前切断输出", joint);
    }
    motor_set_profile(robot, joint, p->accel_ms, p->decel_ms);
    motor_set_speed(robot, joint, p->speed_rpm);
    Sleep(20);
    motor_run(robot, joint, p->dir);
    LOG_INFO("关节%d 堵转启动 (方向=%+d, %drpm, 加速%dms/减速%dms, 阈值=%dmA)",
             joint, p->dir, p->speed_rpm, p->accel_ms, p->decel_ms, p->stall_current);
}

/* home_check_stall：检查是否堵转到位
 * 返回 1=堵转到位，0=还在运行
 * 前置：home_stall_start 已关 0x000B/0x000C 报警并放宽 0x0010 预警。
 * 实测两种顶死形态都出现过，故按以下顺序分级兜底：
 *   1) 驱动器报警 bit21（关报警失败/其它报警兜底）；
 *   2) 位置超差 bit10（报警关闭/放宽未生效时兜底）；
 *   3a) 电流超阈值 AND 位置停涨（主判据A：驱动器持续顶着型顶死）；
 *   3b) 运行中电流归零/读失败 AND 位置停涨（主判据B：固件失步保护切断输出型，
 *       实测该切断比 0x0010 放宽更早且参数挡不住，电流归 0 后主判据A永不命中）。
 * 正常转动电流约 440~470mA、每帧位移上千脉冲，两类停涨判据只可能发生在顶死。 */
static int home_check_stall(Robot *robot, int joint)
{
    uint32_t st;
    int cur, cur_ok, pos_ok = 0, stopped;
    int32_t pos, delta;
    int threshold = stall[joint].stall_current;

    if (motor_read_status(robot, joint, &st) != ERR_NONE) return 0;
    cur_ok = 1;
    cur = motor_read_current(robot, joint);
    if (cur < 0 || cur > 3000) { cur_ok = 0; cur = 0; }

    /* 兜底1：驱动器报警（关报警未生效或其它报警） */
    if (st & LEESN_STAT_ALARM) {
        LOG_INFO("关节%d 驱动器报警(bit21)，判定堵转到位", joint);
        return 1;
    }
    /* 兜底2：位置超差（阈值寄存器写 0 未生效时仍会触发） */
    if (st & LEESN_STAT_OVERRUN) {
        LOG_INFO("关节%d 位置超差(bit10)，判定堵转到位", joint);
        return 1;
    }

    /* 读位置帧：判断与上帧是否停涨（|Δ| <= STALL_POS_STOP_DELTA） */
    pos = motor_read_position(robot, joint, &pos_ok);
    stopped = 0;
    if (pos_ok && s_stall_last_ok[joint]) {
        delta = pos - s_stall_last_pos[joint];
        stopped = (delta >= -STALL_POS_STOP_DELTA) &&
                  (delta <=  STALL_POS_STOP_DELTA);
    }

    /* 主判据A：电流顶高 + 位置停涨，双条件确认顶死
     * cur_only 关节（如4轴）不要求位置停涨：实测顶死时电流已超阈值、
     * 位置仍在每帧上千脉冲地持续增长，永不满足停涨条件，只能等到固件
     * 失步保护切断输出（电流 2185mA→读失败-1）才停，等待过久且电流冲击大；
     * 此类轴电流一超阈值即判定堵转到位。 */
    if (threshold > 0 && cur > threshold) {
        if (stall[joint].cur_only || stopped) {
            LOG_INFO("关节%d 电流超阈值(%dmA>%dmA)%s，判定堵转到位",
                     joint, cur, threshold,
                     stall[joint].cur_only ? "" : "且位置停涨");
            return 1;
        }
    }

    /* 主判据B：运行中电流归零/读失败 + 位置停涨
     * 实测顶死序列：接触限位→电流短暂爬高(如1111mA)→固件失步保护切断输出→
     * 电流归 0（该帧 0x001A 可能读失败返回 -1，同样钳 0）、位置停涨(±1抖动)、
     * 状态字仍 RUN_ACTIVE。该切断比 0x0010 预警更早且参数挡不住，且切断后
     * 命令位置不再累积，bit10 兜底永不触发；只依赖 A/bit10 会无限轮询。
     * 注意：不要求 cur_ok——实测切断帧 0x001A 读失败(-1)被钳 0 时同样成立，
     * 位置停涨 + RUN_ACTIVE 已排除"运动帧电流读失败"的误判。 */
    if (cur < STALL_CUTOFF_MA && stopped &&
        (st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) {
        LOG_INFO("关节%d 运行中电流归零/读失败(%smA<%dmA)且位置停涨，判定堵转到位(输出被切断)",
                 joint, cur_ok ? "0" : "ERR", STALL_CUTOFF_MA);
        return 1;
    }

    /* 未命中：更新位置缓存，供下一帧停涨判定 */
    if (pos_ok) {
        s_stall_last_pos[joint] = pos;
        s_stall_last_ok[joint] = 1;
    }

    LOG_INFO("关节%d 状态=0x%08X 电流=%dmA 位置=%d (阈值=%dmA)",
             joint, (unsigned)st, cur_ok ? cur : -1, pos_ok ? (int)pos : -99999, threshold);
    return 0;
}

/* home_stall_done：堵转到位处理 —— 退出连续运行 → 位置清零 → 校验
 * 前置：回零前已写 0x000B/0x000C=0 关闭超差报警。
 * 堵转命中时驱动器已因失步保护切断输出（电流 0/读失败、位置停涨），
 * 但连续运行命令(0x00C8=0x0001)仍在，若直接 move_abs 会被驱动器忽略，
 * 因此必须先发 0x00C8=0 减速停止退出连续运行模式，再清零把当前位置置 0；
 * 随后 home_stall_forward 的 move_abs 退让才会真正执行，全程不触发报警。 */
static void home_stall_done(Robot *robot, int joint)
{
    int pos_ok = 0;
    int32_t pos;
    int clear_ok;
    StallHome *p = &stall[joint];

    /* 退出连续运行模式：不清除的话后续绝对定位(0x00E8)不执行（实测） */
    if (motor_stop_slow(robot, joint) != ERR_NONE) {
        LOG_WARN("关节%d 减速停止失败，后续退让 move_abs 可能被忽略", joint);
    }
    Sleep((p->decel_ms > 0 ? p->decel_ms : 200) + 100);

    /* 兜底：若仍有报警（关报警未成功/其它报警触发），清零会被驱动器拒绝，须先清 */
    if (motor_read_alarm(robot, joint) != 0) {
        if (motor_clear_alarm(robot, joint) == ERR_NONE) {
            LOG_INFO("关节%d 已清除驱动器报警", joint);
        } else {
            LOG_WARN("关节%d 清除驱动器报警失败", joint);
        }
        Sleep(10);
    }

    /* 清零：当前位置置 0，闭环目标=当前位置，驱动器停止顶着 */
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

/* home_wait_inpos：等待关节运动到位（非运行状态）
 * 返回 1=到位，0=超时 */
static int home_wait_inpos(Robot *robot, int joint, int timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < (uint32_t)timeout_ms) {
        uint32_t st;
        if (motor_read_status(robot, joint, &st) != ERR_NONE) {
            Sleep(200); elapsed += 200; continue;
        }
        if ((st & LEESN_STAT_RUN_MASK) != LEESN_STAT_RUN_ACTIVE) return 1;
        Sleep(200);
        elapsed += 200;
    }
    return 0;
}

/* home_wait_stall：轮询等待关节堵转到位（home_check_stall）
 * 返回 1=堵转命中，0=超时 */
static int home_wait_stall(Robot *robot, int joint, int timeout_ms)
{
    uint32_t start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)timeout_ms) {
        if (home_check_stall(robot, joint)) return 1;
        Sleep(HOME_STALL_POLL_MS);
    }
    return 0;
}

/* home_stall_forward：堵转清零后立即退让到 forward_deg
 * 仅 forward_deg != 0 的轴执行（如 1 轴 +1 顶正限位后收 -90.0°），
 * 避免长时间顶在硬限位上等待其它轴回零导致报警/卡死 */
static void home_stall_forward(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];

    if (p->forward_deg == 0.0) return;
    LOG_INFO("关节%d 堵转后移动到 %.1f°", joint, p->forward_deg);
    robot_movej(robot, joint, p->forward_deg, (double)p->speed_rpm);
    if (!home_wait_inpos(robot, joint, home_timeout_ms)) {
        LOG_WARN("关节%d 退让到 %.1f° 超时", joint, p->forward_deg);
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
                /* 堵转清零后立即退让，不等整组/后续轴回零结束 */
                home_stall_forward(robot, joint);
                done |= (1u << (joint - 1));
            }
        }
        if (all_done) break;
        Sleep(HOME_STALL_POLL_MS);
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

/* home_joint5：关节5 堵转回零
 * 流程与关节1-4 单轴一致：堵转判定 → 清零 → 退让 forward_deg → 恢复限位，
 * 通用步骤复用 home_wait_stall / home_stall_done / home_stall_forward。 */
static void home_joint5(Robot *robot)
{
    if (home_arm(robot, 5) != ERR_NONE) { LOG_WARN("关节5 arm 失败"); return; }
    home_stall_start(robot, 5);

    /* 等待堵转到位 */
    if (!home_wait_stall(robot, 5, home_timeout_ms)) {
        LOG_WARN("关节5 堵转回零超时");
        motor_estop(robot, 5);
        home_restore(robot, 5);
        return;
    }
    home_stall_done(robot, 5);

    /* 堵转清零后退让到 forward_deg */
    home_stall_forward(robot, 5);

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
static ErrCode home_joint6(Robot *robot)
{
    SensorPhase ph = SEN_REV_FAST;
    int in0_was_on = 0;
    uint32_t start_ms, crawl_start = 0;
    ErrCode rc;

    rc = home_arm(robot, 6);
    if (rc != ERR_NONE) { LOG_WARN("关节6 arm 失败：%s", err_str(rc)); return rc; }
    motor_set_profile(robot, 6, sensor.accel_ms, sensor.accel_ms);
    motor_set_speed(robot, 6, sensor.fast_rpm);
    Sleep(20);
    motor_run(robot, 6, sensor.dir);
    LOG_INFO("关节6 传感器回零启动 (反向快速 %drpm)", sensor.fast_rpm);

    start_ms = GetTickCount();

    while (ph != SEN_DONE) {
        uint32_t inputs;
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
        rc = ERR_NONE;
    } else {
        motor_estop(robot, 6);
        rc = ERR_TIMEOUT;
    }

    home_restore(robot, 6);
    if (rc == ERR_NONE) LOG_INFO("关节6 回零完成");
    else                 LOG_WARN("关节6 传感器回零失败：%s", err_str(rc));
    return rc;
}

/* ====================== 公开接口 ====================== */

/* robot_home：执行回零流程
 * 步骤：
 *   1. 关节1-4 并行堵转回零（forward_deg!=0 的轴堵转清零后立即退让）
 *   2. 关节5 堵转回零 + 正向移动
 *   3. 关节6 传感器回零
 * 注：历史上曾有过"堵转后 movej 回零"步骤，因每轴都在堵转清零后立即
 * 经 home_stall_forward 退让到 forward_deg，该步骤成为死代码已移除。
 * 返回 ERR_NONE（各轴失败仅告警，不中断整机流程） */
ErrCode robot_home(Robot *robot)
{
    const int group0[] = {1, 2, 3, 4};
    int j;

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
        (void)home_joint6(robot);   /* 失败内部已告警，不中断整机流程 */
    }

    /* 恢复限位 */
    for (j = 1; j <= 4; j++) {
        if (robot_is_masked(robot, j)) continue;
        home_restore(robot, j);
        Sleep(20);
    }

    LOG_INFO("回零流程结束");
    return ERR_NONE;
}

/* robot_home_single：单轴独立堵转回零（home:N）
 * 只操作目标轴，不触碰/不依赖其它轴：arm(使能+关限位) → 堵转运行 →
 * 电流超阈值判定到位 → 停稳清零 → 自动 movej 到 stall[joint].forward_deg。
 * 用于单独验证某轴回点流程（如 home:1 验证 1 轴堵转后收 -90.0°）。 */
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

/* robot_home_joint：单轴回零后自动运动到指定角度
 * 实现：临时屏蔽其余轴 → 复用 robot_home（只回零目标轴）→
 *       movej 到 angle_deg → 等待到位 → 恢复原屏蔽状态。
 * 用于“某轴回零成功后自动跑到指定角度”（如 1 号正转回零后自动反转 90°）。 */
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
            if (home_wait_inpos(robot, joint, home_timeout_ms)) {
                LOG_INFO("关节%d 到位（%.1f°）", joint, angle_deg);
            } else {
                LOG_WARN("关节%d 运动到 %.1f° 超时", joint, angle_deg);
                rc = ERR_TIMEOUT;
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
