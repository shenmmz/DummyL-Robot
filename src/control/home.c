

#include "control/home.h"
#include "api/motor_reg.h"
#include "control/robot_internal.h"
#include "config/robot_config.h"
#include "kinematics/joint_zero.h"
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

typedef struct {
    int    speed_rpm;
    int    accel_ms;
    int    decel_ms;
    int    dir;
    int    stall_current;
    int    torque_level;
} StallHome;

/* 0x009E 恒力矩模式（手冊第 49 条）。高 8 位 = 模式，低 8 位 = 力矩等级 0~255。 */
#define TORQUE_MODE_HOME  1        /* 碰撞回原点（唯一在用：堵转回零 / robot_torque_probe） */
#define TORQUE_MODE_GRAB  2        /* 抓取物体（未使用） */
#define TORQUE_MODE_HOLD_RUN 3     /* 恒力矩运行（未使用） */
#define TORQUE_MODE_HOLD_KEEP 4    /* 恒力矩保持（未使用） */
/* 堵转清零后「位置是否算回到 0」的容差，单位=电机侧脉冲；上电首次回零必然超差（清的是原点偏移，不清计数器）*/
#define HOME_ZERO_TOL_STEPS    500
 static StallHome stall[7] = {
     [1] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200, .dir = +1, .stall_current = 480, .torque_level = 120 },
     [2] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200,  .dir = -1, .stall_current = 490, .torque_level = 120 },
     [3] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200, .dir = +1, .stall_current = 480, .torque_level = 120 },
     [4] = { .speed_rpm = 60,   .accel_ms = 80,  .decel_ms = 100, .dir = -1, .stall_current = 400, .torque_level = 120 },
     [5] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200, .dir = -1, .stall_current = 390, .torque_level = 120 },
 };

static struct {
    int fast_rpm;
    int slow_rpm;
    int crawl_rpm;
    int dir;
    int accel_ms;
} sensor = { 300, 90, 30, -1, 100 };

#define HOME_J6_ZERO_RPM         100

/* 关节 j 回零后应处的【电机侧机械角】= 机械角表 + 零点偏置。
 * 全轴对齐凹槽时应得到 0,0,90,0,0,0。 */
static double home_forward_deg(int j)
{
    static const double mech[ROBOT_JOINT_COUNT] = ROBOT_HOME_MECH_DEG;
    const double *zero = joint_zero_get();
    return mech[j - 1] + zero[j - 1];
}
/* 回零后退让时的速度：关节 6 单列 100rpm。
 * 原因：stall[6] 从未配置（speed_rpm=0），直接用会变成"干等 20s"。 */
static int home_forward_rpm(int j)
{
    return (j == 6) ? HOME_J6_ZERO_RPM : stall[j].speed_rpm;
}

static int    home_timeout_ms  = 20000;

#define HOME_STALL_POLL_MS  1

#define HOME_STALL_MASK_MS   150

#define SENSOR_IN0  0x0001u
#define SENSOR_IN1  0x0002u


typedef enum {
    SEN_INIT,
    SEN_FWD_LEAVE,
    SEN_REV_FIND,
    SEN_FWD_FIND,
    SEN_SLOW_LEAVE,
    SEN_DONE
} SensorPhase;


/* 回零前的武装：使能 + 等 50ms + 关软限位。
 * 必须关软限位，否则退让动作会被限位挡住。 */
static ErrCode home_arm(Robot *robot, int joint)
{
    ErrCode rc = motor_enable(robot, joint);
    if (rc != ERR_NONE) return rc;
    Sleep(50);
    return motor_set_limit(robot, joint, 0);
}

/* 回零收尾：恢复使能、恢复位置超差报警、把位置偏差预警放宽回 20 步。 */
static void home_restore(Robot *robot, int joint)
{
    if (motor_enable(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 恢复使能失败\n", joint);
    }
    Sleep(30);
    if (motor_restore_pos_err_alarm(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 恢复超差报警失败\n", joint);
    }
    if (motor_set_pos_err_prewarn(robot, joint, 20) != ERR_NONE) {
        printf("[警告] 关节%d 恢复位置偏差预警失败\n", joint);
    }
}


#define STALL_PREWARN_STEPS   10000


static uint32_t s_stall_t0_ms[7];

static uint32_t s_stall_mask_end_ms[7];

/* 启动堵转回零：记录 t0 与 150ms 掩码窗口，关超差报警、放宽偏差预警到 10000 步，
 * 设 ACC/DEC 与速度，最后进力矩模式 TORQUE_MODE_HOME 并起转。
 * torque_level<=0 直接报错返回（纯电流回零的老路径已移除）。 */
static ErrCode home_stall_start(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];

    s_stall_t0_ms[joint]   = GetTickCount();

    s_stall_mask_end_ms[joint] = s_stall_t0_ms[joint] + HOME_STALL_MASK_MS;

    if (motor_disable_pos_err_alarm(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 关闭超差报警失败，顶死仍会报警锁存\n", joint);
    }
    if (motor_set_pos_err_prewarn(robot, joint, STALL_PREWARN_STEPS) != ERR_NONE) {
        printf("[警告] 关节%d 放宽位置偏差预警失败，顶死仍会提前切断输出\n", joint);
    }
    motor_set_profile(robot, joint, p->accel_ms, p->decel_ms);
    motor_set_speed(robot, joint, p->speed_rpm);
    if (p->torque_level <= 0) {
        printf("[错误] 关节%d torque_level=%d 未配置，纯电流回零已移除，无法回零\n", joint, p->torque_level);
        return ERR_ARG;
    }
    if (motor_set_torque_mode(robot, joint, TORQUE_MODE_HOME, p->torque_level) != ERR_NONE) {
        printf("[错误] 关节%d 设碰撞回原点力矩等级%d失败，回零中止\n", joint, p->torque_level);
        return ERR_PORT;
    }
    Sleep(20);
    if (motor_torque_run(robot, joint, p->dir, 0, 1) != ERR_NONE) {
        printf("[错误] 关节%d 启动碰撞回原点力矩模式失败，回零中止\n", joint);
        return ERR_PORT;
    }
    return ERR_NONE;
}

/* 判堵转，返回 1=已堵转 / 0=继续跑。判据按顺序：
 *   ① 状态字 ALARM  ② 状态字 OVERRUN  ③ 150ms 掩码期内一律不算
 *   ④ 电流 > stall[joint].stall_current(mA)。
 * 电流 >3000mA 视为读失败，不参与判断。
 * ⚠️ 阈值偏低：静止电流 500/499/495/385/371mA vs 阈值 480/490/480/400/390 ⇒ J1/J2/J3 上电即"已堵转"（未证实）。 */
static int home_check_stall(Robot *robot, int joint, int *cur_out)
{
    uint32_t st;
    int cur, cur_ok;
    int32_t pos = 0;
    int threshold = stall[joint].stall_current;
    uint32_t now_ms;

    now_ms = GetTickCount();

    cur_ok = 1;
    cur = motor_read_current(robot, joint);
    if (cur < 0 || cur > 3000) { cur_ok = 0; cur = 0; }
    if (cur_out) *cur_out = cur_ok ? cur : -1;

    if (motor_read_pos_status(robot, joint, &pos, &st) != ERR_NONE) return 0;

    if (st & LEESN_STAT_ALARM) {
        return 1;
    }
    if (st & LEESN_STAT_OVERRUN) {
        return 1;
    }

    if (s_stall_t0_ms[joint] != 0 && now_ms < s_stall_mask_end_ms[joint]) {
        return 0;
    }

    if (threshold > 0 && cur_ok && cur > threshold) {
        return 1;
    }

    return 0;
}

/* 堵转后的收尾：停力矩 → 临时压短减速时间到 50ms → 减速停 → 等 150ms → 恢复减速时间
 * → 读报警并最多清 3 次 → 清位置（写 0x00D2 把当前点定为原点）。
 * 清位置失败要报错：后续退让是相对位移，但绝对定位仍会带这个偏置。 */
static void home_stall_done(Robot *robot, int joint)
{
    int clear_ok;
    int alarm;

    if (stall[joint].torque_level > 0) {
        motor_torque_run(robot, joint, 0, 0, 0);
        motor_set_torque_mode(robot, joint, 0, 0);
    }
    StallHome *p = &stall[joint];

    if (motor_set_profile(robot, joint, p->accel_ms, 50) != ERR_NONE) {
        printf("[警告] 关节%d 临时压短减速时间失败，仍按原减速停止\n", joint);
    }
    if (motor_stop_slow(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 减速停止失败，后续退让 move_abs 可能被忽略\n", joint);
    }
    Sleep(150);
    if (motor_set_profile(robot, joint, p->accel_ms, p->decel_ms) != ERR_NONE) {
        printf("[警告] 关节%d 恢复减速时间失败，后续 movej 减速将偏短\n", joint);
    }

    alarm = motor_read_alarm(robot, joint);
    if (alarm > 0) {
        int retry, cleared = 0;
        printf("[警告] 关节%d 检测到驱动器报警 码=%d\n", joint, alarm);
        for (retry = 0; retry < 3; retry++) {
            if (motor_clear_alarm(robot, joint) == ERR_NONE) {
                cleared = 1;
                break;
            }
            Sleep(50);
        }
        if (!cleared) {
            printf("[警告] 关节%d 清除驱动器报警失败，清零可能被驱动器拒绝\n", joint);
        }
        Sleep(10);
    }

    clear_ok = (motor_clear_pos(robot, joint) == ERR_NONE);
    Sleep(30);
    if (!clear_ok) {
        printf("[错误] 关节%d 堵转清零失败，后续退让 move_abs 将基于旧零点，位置会错\n", joint);
    }
}

#define HOME_INPOS_TOL_STEPS   100

/* 到位查询，返回 1=到位 / 0=还没到 / -1=报警或超差 / -2=读失败。
 * ★ 到位 = 无报警 + 已退出 RUN_ACTIVE + 位置在 target±100 步内。
 * 绝不能用 STAT_INPOS(bit12) 单独判断 —— 那是粘滞位，实测差 28.5° 也报"到位"。 */
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

/* 等到位，轮询 200ms（报警时立刻返回 -1）。
 * ★ 必须先"见过 RUN_ACTIVE"再判到位，否则会在指令还没下发出去时误判成功。
 * 注意轮询 200ms 远大于一轮总线 10.2ms，这里不是瓶颈、也不需要更快。 */
static int home_wait_inpos(Robot *robot, int joint, int32_t target_steps, int timeout_ms)
{
    uint32_t start_ms = GetTickCount();
    int saw_active = 0;

    while ((GetTickCount() - start_ms) < (uint32_t)timeout_ms) {
        int q = home_inpos_query(robot, joint, target_steps);
        if (q == 1) {
            if (saw_active) return 1;
            Sleep(20);
            if (home_inpos_query(robot, joint, target_steps) == 1) return 1;
            continue;
        }
        if (q == -1) return -1;
        if (q == 0) {
            uint32_t st;
            if (motor_read_status(robot, joint, &st) == ERR_NONE &&
                (st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) {
                saw_active = 1;
            }
        }
        Sleep(200);
    }
    return 0;
}

/* 等堵转，1ms 轮询（HOME_STALL_POLL_MS）。 */
static int home_wait_stall(Robot *robot, int joint, int timeout_ms)
{
    uint32_t start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)timeout_ms) {
        int rc = home_check_stall(robot, joint, NULL);
        if (rc == 1) return 1;
        Sleep(HOME_STALL_POLL_MS);
    }
    return 0;
}

/* 堵转清零后，把轴退让到 home_forward_deg() 指定的机械角。
 * 用【相对位移】(当前位置 + delta) 而不是绝对目标：
 * 若 0x00D2 零点没生效（清零后位置非 0），绝对目标会带着旧偏置走错。
 * 超时/报警只打警告，不中断其他轴。 */
static void home_stall_forward(Robot *robot, int joint)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    StallHome *p = &stall[joint];
    double fdeg = home_forward_deg(joint);
    int32_t delta, pos = 0, target;
    int pos_ok = 0, rc;

    if (fdeg == 0.0) return;

    delta  = DEG2STEPS(fdeg, reductions[joint - 1]);
    pos    = motor_read_position(robot, joint, &pos_ok);
    target = pos_ok ? (pos + delta) : delta;
    if (!pos_ok) {
        printf("[警告] 关节%d 转角前读位置失败，退回绝对目标 %d\n", joint, (int)delta);
    } else if (pos > HOME_ZERO_TOL_STEPS || pos < -(int32_t)HOME_ZERO_TOL_STEPS) {
        printf("[警告] 关节%d 清零后位置非0(%d步≈%.2f°)：0x00D2 零点可能未生效，\n"
                 "本次已改用相对位移保证行程，但后续绝对定位仍带此偏置",
                 joint, (int)pos,
                 (double)pos * 360.0 /
                 ((double)reductions[joint - 1] * (double)ENCODER_STEPS_PER_REV));
    }

    if (motor_set_speed(robot, joint, p->speed_rpm) != ERR_NONE ||
        motor_move_abs(robot, joint, target) != ERR_NONE) {
        printf("[警告] 关节%d 转角指令发送失败，停在清零点\n", joint);
        return;
    }

    rc = home_wait_inpos(robot, joint, target, home_timeout_ms);
    if (rc == 0) {
        int pos2_ok = 0;
        int32_t pos2 = motor_read_position(robot, joint, &pos2_ok);
        printf("[警告] 关节%d 退让 %.1f° 超时（目标=%d 位置=%d 差=%d）\n",
                 joint, fdeg, (int)target,
                 pos2_ok ? (int)pos2 : -99999,
                 pos2_ok ? (int)(target - pos2) : -99999);
    } else if (rc < 0) {
        printf("[警告] 关节%d 退让 %.1f° 报警/异常\n", joint, fdeg);
    }
}

/* 力矩碰撞回零【诊断】：给定力矩等级跑一次，每 200ms 打一行
 * `T+ms 状态字 电流mA 位置 备注`，最多 15s。
 * 见到 bit15 HOMED 或退出 RUN_ACTIVE 就停并退出。
 * 用途：判断"碰撞回零为什么不停"到底是没进力矩模式、还是驱动器不认位。 */
ErrCode robot_torque_probe(Robot *robot, int joint, int level)
{
    uint32_t st;
    int cur, cur_ok, pos_ok;
    int32_t pos;
    uint32_t start_ms, now_ms;
    int seen_homed = 0, seen_stop = 0;
    StallHome *p = &stall[joint];

    if (robot == NULL) return ERR_ARG;
    if (joint < 1 || joint > 6) return ERR_ARG;
    if (level < 0 || level > 255) return ERR_ARG;

    printf("=== 力矩碰撞回原点诊断：关节%d 等级=%d 方向=%+d ===\n", joint, level, p->dir);

    if (home_arm(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d arm 失败\n", joint);
        return ERR_PORT;
    }
    motor_set_pos_err_prewarn(robot, joint, STALL_PREWARN_STEPS);
    motor_disable_pos_err_alarm(robot, joint);
    if (motor_set_torque_mode(robot, joint, TORQUE_MODE_HOME, level) != ERR_NONE) {
        printf("[警告] 关节%d 设力矩模式失败\n", joint);
        home_restore(robot, joint);
        return ERR_PORT;
    }
    Sleep(20);
    if (motor_torque_run(robot, joint, p->dir, 0, 1) != ERR_NONE) {
        printf("[警告] 关节%d 启动力矩碰撞失败\n", joint);
        motor_set_torque_mode(robot, joint, 0, 0);
        home_restore(robot, joint);
        return ERR_PORT;
    }

    start_ms = GetTickCount();
    printf("T+ms 状态字 电流mA 位置 备注\n");
    while ((now_ms = GetTickCount()) - start_ms < 15000) {
        uint32_t elaps = now_ms - start_ms;
        cur = motor_read_current(robot, joint);
        cur_ok = (cur >= 0 && cur <= 3000);
        pos_ok = (motor_read_pos_status(robot, joint, &pos, &st) == ERR_NONE);
        if (!pos_ok) {
            printf("%ums ? ? ? [读失败]\n", elaps);
            Sleep(200); continue;
        }
        {
            const char *tag = "";
            if (st & LEESN_STAT_ALARM)        tag = " [报警]";
            else if (st & LEESN_STAT_OVERRUN) tag = " [超差]";
            else if (st & LEESN_STAT_HOMED)  { tag = " [HOMED]"; seen_homed = 1; }
            else if ((st & LEESN_STAT_RUN_MASK) != LEESN_STAT_RUN_ACTIVE) { tag = " [退出RUN]"; seen_stop = 1; }
            printf("%ums 0x%08X %dmA %d%s%s\n", elaps, (unsigned)st,
                     cur_ok ? cur : -1, (int)pos, tag,
                     (st & LEESN_STAT_RUN_ACTIVE) ? " RUN" : "");
            if (seen_homed || seen_stop) {
                printf("关节%d 检测到到位信号（%s），停止力矩并退出诊断\n",
                         joint, seen_homed ? "bit15 HOMED" : "退出 RUN_ACTIVE");
                break;
            }
        }
        Sleep(200);
    }

    motor_torque_run(robot, joint, 0, 0, 0);
    motor_set_torque_mode(robot, joint, 0, 0);
    home_restore(robot, joint);
    printf("=== 力矩碰撞诊断结束：关节%d 等级=%d（HOMED=%d 退出RUN=%d）===\n",
             joint, level, seen_homed, seen_stop);
    return ERR_NONE;
}


typedef struct {
    SensorPhase ph;
    int in0_was_on;
    int32_t pos_base;
    uint32_t start_ms;
    uint32_t leave_start;
    int done;
    int failed;
    int last_cur;
    int pos_fail_cnt;
} SensorCtx;

#define SEN_POS_FAIL_MAX   10

/* 设关节 6 速度：同时写 0x009A（连续运行速度源）与记忆速度。
 * 只写记忆速度的话，连续运行(motor_run)会按驱动器里的旧值跑。 */
static void sensor6_set_rpm(Robot *robot, int rpm)
{
    if (motor_set_speed16(robot, 6, rpm) != ERR_NONE) {
        printf("[警告] 关节6 写连续运行速度源0x009A(%drpm)失败，按记忆速度运行\n", rpm);
    }
    motor_set_speed(robot, 6, rpm);
}

/* 关节 6 传感器回零的初始化：清上下文 → arm → 设 ACC/DEC → 提速到 300rpm。 */
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
    c->pos_fail_cnt = 0;

    rc = home_arm(robot, 6);
    if (rc != ERR_NONE) {
        printf("[警告] 关节6 arm 失败：%s\n", err_str(rc));
        return rc;
    }
    motor_set_profile(robot, 6, sensor.accel_ms, sensor.accel_ms);
    sensor6_set_rpm(robot, sensor.fast_rpm);

    c->start_ms = GetTickCount();
    return ERR_NONE;
}

/* 关节 6 传感器回零状态机（每次调用走一步）。
 * 传感器接在状态字 bit0(IN0)/bit1(IN1) 上；流程：
 *   若在 IN0 内 → 正向开 3° 离开 → 反向找 IN0 → 30rpm 慢速爬出 IN0 → 停 + 清零。
 *   若在 IN1 内 → 直接正向找 IN0。
 * 超时：整体 20s（home_timeout_ms），慢速离开 IN0 单独 10s。
 * 位置连读失败 10 次 ⇒ 急停保护（SEN_POS_FAIL_MAX）。
 * 返回 1=完成 / -1=失败 / 0=继续。 */
static int sensor6_tick(Robot *robot, SensorCtx *c)
{
    SensorPhase ph = c->ph;
    uint32_t inputs;
    int in0, in1, cur, pos_ok = 0;
    int32_t pos;
    uint32_t now = GetTickCount();

    if (c->done)   return 1;
    if (c->failed) return -1;

    if (ph == SEN_SLOW_LEAVE && c->leave_start > 0 &&
        (now - c->leave_start) >= 10000) {
        printf("[警告] 关节6 慢速离开IN0超时\n");
        c->failed = 1;
        motor_estop(robot, 6);
        return -1;
    }
    if (ph != SEN_SLOW_LEAVE && c->start_ms > 0 &&
        (now - c->start_ms) >= (uint32_t)home_timeout_ms) {
        printf("[警告] 关节6 传感器回零超时\n");
        c->failed = 1;
        motor_estop(robot, 6);
        return -1;
    }

    if (motor_read_status(robot, 6, &inputs) != ERR_NONE) return 0;
    in0 = (inputs & SENSOR_IN0) ? 1 : 0;
    in1 = (inputs & SENSOR_IN1) ? 1 : 0;
    cur = motor_read_current(robot, 6);
    if (cur < 0 || cur > 3000) cur = 0;
    c->last_cur = cur;
    pos = motor_read_position(robot, 6, &pos_ok);

    if (ph == SEN_FWD_LEAVE) {
        if (!pos_ok) {
            c->pos_fail_cnt++;
            if (c->pos_fail_cnt >= SEN_POS_FAIL_MAX) {
                printf("[警告] 关节6 正向开3°阶段位置读连续失败%d次，急停保护\n",
                         SEN_POS_FAIL_MAX);
                c->failed = 1;
                motor_estop(robot, 6);
                return -1;
            }
        } else {
            c->pos_fail_cnt = 0;
        }
    }


    switch (ph) {
    case SEN_INIT:
        if (in0) {
            c->pos_base = pos_ok ? pos : 0;
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_LEAVE;
        } else if (in1) {
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_FIND;
        } else {
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, sensor.dir);
            c->ph = SEN_REV_FIND;
        }
        break;

    case SEN_FWD_LEAVE:
        if (pos_ok && (pos - c->pos_base) >= DEG2STEPS(3.0, 50)) {
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, sensor.dir);
            c->ph = SEN_REV_FIND;
        }
        break;

    case SEN_REV_FIND:
        if (in0) {
            sensor6_set_rpm(robot, sensor.crawl_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->in0_was_on = 1;
            c->leave_start = GetTickCount();
            c->ph = SEN_SLOW_LEAVE;
        } else if (in1) {
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_FIND;
        }
        break;

    case SEN_FWD_FIND:
        if (in0) {
            sensor6_set_rpm(robot, sensor.crawl_rpm);
            c->in0_was_on = 1;
            c->leave_start = GetTickCount();
            c->ph = SEN_SLOW_LEAVE;
        }
        break;

    case SEN_SLOW_LEAVE:
        if (c->in0_was_on && !in0) {
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

/* 关节 6 收尾：成功则清位置(0x00D2)，失败则急停，最后恢复使能与报警设置。 */
static void sensor6_finish(Robot *robot, SensorCtx *c)
{
    if (c->done) {
        motor_clear_pos(robot, 6);
        Sleep(30);
    } else {
        if (!c->failed) {
            c->failed = 1;
            motor_estop(robot, 6);
        }
        printf("[警告] 关节6 传感器回零失败：%s\n", err_str(ERR_TIMEOUT));
    }

    home_restore(robot, 6);
    if (c->done) printf("关节6 回零完成\n");
}

static void home_goto_pose(Robot *robot, int only_joint);

/* 跑关节 6 传感器回零状态机直到完成或失败，完成后统一转角到 home 位姿。 */
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
    if (c.done) home_goto_pose(robot, 6);
    return c.done ? ERR_NONE : ERR_TIMEOUT;
}


/* 把关节统一转到 home 位姿（0,0,90,0,0,0），只等最慢的一根。
 * 各轴用相对位移下发；轮询 1ms。only_joint=0 表示全部，1~6 表示只动某一轴。
 * 用 home_forward_rpm() 取速度 ⇒ 关节 6 走 100rpm 而不是 0。 */
static void home_goto_pose(Robot *robot, int only_joint)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    uint8_t pend[7] = {0};
    int32_t tgt[7] = {0};
    uint32_t start_ms;
    int remain = 0, j;
    int j_first = (only_joint >= 1 && only_joint <= 6) ? only_joint : 1;
    int j_last  = (only_joint >= 1 && only_joint <= 6) ? only_joint : 6;

    for (j = j_first; j <= j_last; j++) {
        double fdeg = home_forward_deg(j);
        int    frpm = home_forward_rpm(j);
        if (robot_is_masked(robot, j)) continue;
        if (fdeg == 0.0) continue;

        {
            uint32_t st;
            int32_t pos = 0;
            int32_t delta = DEG2STEPS(fdeg, reductions[j - 1]);
            int pos_ok = 0, st_ok;

            st_ok = (motor_read_status(robot, j, &st) == ERR_NONE);
            pos = motor_read_position(robot, j, &pos_ok);
            if (st_ok && pos_ok) {
                tgt[j] = pos + delta;
                if (pos > HOME_ZERO_TOL_STEPS || pos < -(int32_t)HOME_ZERO_TOL_STEPS) {
                    printf("[警告] 关节%d 清零后位置非0(%d步≈%.2f°)：0x00D2 零点可能未生效，\n"
                             "本次已改用相对位移保证行程，但后续绝对定位仍带此偏置",
                             j, (int)pos,
                             (double)pos * 360.0 /
                             ((double)reductions[j - 1] * (double)ENCODER_STEPS_PER_REV));
                }
            } else {
                tgt[j] = delta;
            }
        }
        if (motor_set_speed(robot, j, frpm) != ERR_NONE ||
            motor_move_abs(robot, j, tgt[j]) != ERR_NONE) {
            printf("[警告] 关节%d 发 movej 失败\n", j);
            continue;
        }
        pend[j] = 1;
        remain++;
    }
    if (remain == 0) return;

    start_ms = GetTickCount();
    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= (uint32_t)home_timeout_ms) {
            for (j = j_first; j <= j_last; j++) {
                int32_t pos;
                int pos_ok;
                if (!pend[j]) continue;
                pos = motor_read_position(robot, j, &pos_ok);
                printf("[警告] 关节%d 转角到 %.1f° 超时（目标=%d 位置=%d 差=%d）\n",
                         j, home_forward_deg(j), (int)tgt[j],
                         pos_ok ? (int)pos : -99999,
                         pos_ok ? (int)(tgt[j] - pos) : -99999);
                pend[j] = 0;
            }
            break;
        }
        for (j = j_first; j <= j_last; j++) {
            int q;
            if (!pend[j]) continue;
            q = home_inpos_query(robot, j, tgt[j]);
            if (q == 1) {
                pend[j] = 0; remain--;
            } else if (q == -1) {
                printf("[警告] 关节%d 转角到 %.1f° 报警/异常，已结算该轴\n",
                         j, home_forward_deg(j));
                pend[j] = 0; remain--;
            } else if (q == -2) {
                Sleep(HOME_STALL_POLL_MS);
            }
        }
        if (remain > 0) Sleep(HOME_STALL_POLL_MS);
    }
}

/* 全机回零（CLI `home`）。
 * 顺序有讲究：堵转轴 2,3,5,1 一起起转，关节 3 到位后才启动 4
 *   （J4 与 J3 会互相顶，同时转会顶死）；关节 6 走传感器回零，与上面并行。
 * 全部成功 → 统一转角到 home 位姿；只要有失败轴就跳过统一转角（只恢复使能）。
 * 返回 ERR_TIMEOUT 表示有轴没归零。 */
ErrCode robot_home(Robot *robot)
{
    const int group_stall[] = {2, 3, 5, 1};
    uint8_t active[7] = {0};
    uint8_t sdone[7] = {0};
    uint8_t sfail[7] = {0};
    SensorCtx c6;
    uint32_t start_ms;
    int j, all_ok;
    int any_fail = 0;

    if (robot == NULL) return ERR_ARG;


    for (int gi = 0; gi < (int)(sizeof(group_stall) / sizeof(group_stall[0])); gi++) {
        int jj = group_stall[gi];
        if (robot_is_masked(robot, jj)) continue;
        if (home_arm(robot, jj) != ERR_NONE) {
            printf("[警告] 关节%d arm 失败，跳过\n", jj);
            sfail[jj] = 1;
            any_fail = 1;
            continue;
        }
        if (home_stall_start(robot, jj) != ERR_NONE) {
            printf("[警告] 关节%d 堵转启动失败，跳过\n", jj);
            sfail[jj] = 1;
            any_fail = 1;
            continue;
        }
        active[jj] = 1;
        Sleep(10);
    }
    if (!robot_is_masked(robot, 6)) {
        if (sensor6_start(robot, &c6) == ERR_NONE) {
            active[6] = 1;
        } else {
            sfail[6] = 1;
            any_fail = 1;
        }
    }

    start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)home_timeout_ms) {
        int all_done = 1;

        if (active[6] && !sdone[6] && !sfail[6]) {
            int r = sensor6_tick(robot, &c6);
            if (r == 1)       sdone[6] = 1;
            else if (r == -1) { sfail[6] = 1; any_fail = 1; }
            else              all_done = 0;
        }

        for (j = 1; j <= 5; j++) {
            int rc;
            if (!active[j] || sdone[j] || sfail[j]) continue;
            all_done = 0;
            rc = home_check_stall(robot, j, NULL);
            if (rc == 0) continue;
            home_stall_done(robot, j);
            sdone[j] = 1;
            printf("关节%d 堵转回零完成\n", j);
            if (j == 3 && !active[4] && !robot_is_masked(robot, 4)) {
                if (home_arm(robot, 4) == ERR_NONE &&
                    home_stall_start(robot, 4) == ERR_NONE) {
                    active[4] = 1;
                } else {
                    printf("[警告] 关节4 arm/启动失败，跳过（未归零）\n");
                    sfail[4] = 1;
                    any_fail = 1;
                }
            }
        }

        if (all_done) break;
        Sleep(HOME_STALL_POLL_MS);
    }

    for (j = 1; j <= 5; j++) {
        if (!active[j] || sdone[j]) continue;
        printf("[警告] 关节%d 堵转回零超时\n", j);
        motor_estop(robot, j);
        sfail[j] = 1;
        any_fail = 1;
    }
    if (active[6] && !sdone[6]) {
        sfail[6] = 1;
        any_fail = 1;
    }
    if (active[6]) {
        sensor6_finish(robot, &c6);
    }

    all_ok = 1;
    for (j = 1; j <= 6; j++) {
        if (robot_is_masked(robot, j)) continue;
        if (!active[j] || sfail[j]) {
            all_ok = 0;
            break;
        }
    }
    if (all_ok) {
        for (j = 1; j <= 5; j++) {
            if (robot_is_masked(robot, j)) continue;
            home_restore(robot, j);
            Sleep(20);
        }
        home_goto_pose(robot, 0);
    } else {
        printf("[警告] 回零：存在未归零/失败轴，跳过统一转角\n");
        for (j = 1; j <= 5; j++) {
            if (robot_is_masked(robot, j)) continue;
            home_restore(robot, j);
            Sleep(20);
        }
    }

    return any_fail ? ERR_TIMEOUT : ERR_NONE;
}

/* 单轴回零（CLI `home N`）。关节 6 走传感器回零，其余走堵转回零 + 退让。 */
ErrCode robot_home_single(Robot *robot, int joint)
{
    ErrCode rc;
    int hit;

    if (robot == NULL) return ERR_ARG;
    if (joint < 1 || joint > 6) return ERR_ARG;
    if (robot_is_masked(robot, joint)) return ERR_MASKED;

    if (joint == 6) {
        printf("=== 单轴传感器回零：关节6 ===\n");
        return home_joint6(robot);
    }

    printf("=== 单轴独立堵转回零：关节%d ===\n", joint);
    rc = home_arm(robot, joint);
    if (rc != ERR_NONE) {
        printf("[警告] 关节%d arm 失败：%s\n", joint, err_str(rc));
        return rc;
    }

    rc = home_stall_start(robot, joint);
    if (rc != ERR_NONE) {
        printf("[警告] 关节%d 堵转启动失败：%s\n", joint, err_str(rc));
        home_restore(robot, joint);
        return rc;
    }

    hit = home_wait_stall(robot, joint, home_timeout_ms);
    if (!hit) {
        printf("[警告] 关节%d 堵转回零超时\n", joint);
        motor_estop(robot, joint);
        home_restore(robot, joint);
        return ERR_TIMEOUT;
    }
    home_stall_done(robot, joint);

    home_stall_forward(robot, joint);

    home_restore(robot, joint);
    printf("关节%d 单轴回零完成（目标 %.1f°）\n", joint, home_forward_deg(joint));
    return ERR_NONE;
}

/* ⚠️ 死代码：`src/` 内没有任何调用者（已在 memory 记录，未获放行前不改）。
 * 另有一处真 bug：速度取 `stall[joint].speed_rpm`，关节 6 时为 0 ⇒ 会干等 20s。
 * 正确写法见同文件 home_forward_rpm()。 */
ErrCode robot_home_joint(Robot *robot, int joint, double angle_deg, double speed_rpm)
{
    int saved[6];
    int j;
    ErrCode rc = ERR_NONE;

    if (robot == NULL) return ERR_ARG;
    if (joint < 1 || joint > 6) return ERR_ARG;
    if (robot_is_masked(robot, joint)) return ERR_MASKED;

    printf("单轴回零：关节%d 回零后自动运动到 %.1f° ...\n", joint, angle_deg);

    for (j = 1; j <= 6; j++) {
        saved[j - 1] = robot_is_masked(robot, j);
        if (j != joint) {
            robot_mask(robot, j);
        }
    }

    rc = robot_home(robot);

    if (rc == ERR_NONE) {
        double spd = (speed_rpm > 0.0) ? speed_rpm : (double)stall[joint].speed_rpm;

        rc = robot_movej(robot, joint, angle_deg, spd);
        if (rc != ERR_NONE) {
            printf("[错误] 关节%d 自动运动到 %.1f° 失败：%s\n",
                      joint, angle_deg, err_str(rc));
        } else {
            printf("关节%d 自动运动到 %.1f° ...\n", joint, angle_deg);
            {
                const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
                double mech[6] = {0}, motor[6] = {0};
                int32_t target;
                mech[joint - 1] = angle_deg;
                joint_zero_mech_to_motor(mech, motor);
                target = DEG2STEPS(motor[joint - 1], reductions[joint - 1]);
                int wr = home_wait_inpos(robot, joint, target, home_timeout_ms);
                if (wr == 1) {
                    printf("关节%d 到位（%.1f°）\n", joint, angle_deg);
                } else if (wr == 0) {
                    printf("[警告] 关节%d 运动到 %.1f° 超时\n", joint, angle_deg);
                    rc = ERR_TIMEOUT;
                } else {
                    printf("[警告] 关节%d 运动到 %.1f° 报警/异常\n", joint, angle_deg);
                    rc = ERR_ALARM;
                }
            }
        }
    }

    for (j = 1; j <= 6; j++) {
        if (saved[j - 1]) {
            robot_mask(robot, j);
        } else {
            robot_unmask(robot, j);
        }
    }

    printf("关节%d 回零并运动到 %.1f°：%s\n",
             joint, angle_deg, rc == ERR_NONE ? "完成" : err_str(rc));
    return rc;
}
