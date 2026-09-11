/*
 * home.c —— 回零实现
 * 关节1-5：碰撞回原点力矩模式回零（顶硬限位→电流超阈判到位→清零→movej 到 forward_deg）
 * 关节6：IN0/IN1 传感器回零（零点=IN0 正向 on→off 沿，兼容三种初始位置）
 *        清零后再正转 q0_J6(+90°，见 config) 到机械零点
 * 寄存器操作统一走 motor_reg API，不直接操作 Modbus 帧。
 */

#include "control/home.h"
#include "api/motor_reg.h"
#include "control/robot_internal.h"   /* LEESN_STAT_* 状态位定义 */
#include "config/robot_config.h"      /* DEG2STEPS */
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
    int    torque_level;  /* 碰撞回原点力矩等级 1~255（必填；0=未配置=错误，纯电流回零已移除）；
                             走 0x009E 力矩模式顶限位，到等级即停，更柔和可控；
                             撞到位由 home_check_stall 电流超阈判定（本机不报 HOMED/退出RUN） */
} StallHome;

/* 力矩模式位（写 0x009E BIT15~8）：手册第 49 节 */
#define TORQUE_MODE_HOME  1        /* 碰撞回原点 */
#define TORQUE_MODE_GRAB  2        /* 抓取物体 */
#define TORQUE_MODE_HOLD_RUN 3     /* 恒力矩运行 */
#define TORQUE_MODE_HOLD_KEEP 4    /* 恒力矩保持 */
#define HOME_ZERO_TOL_STEPS    500 /**/
 static StallHome stall[7] = {
     [1] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200, .dir = +1, .stall_current = 480, .forward_deg = -176.85, .torque_level = 120 },
     [2] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200,  .dir = -1, .stall_current = 490, .forward_deg = 72.87, .torque_level = 120 },
     [3] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200, .dir = +1, .stall_current = 480, .forward_deg = -85.46, .torque_level = 120 },
     [4] = { .speed_rpm = 60,   .accel_ms = 80,  .decel_ms = 100, .dir = -1, .stall_current = 400, .forward_deg = 7.38, .torque_level = 120 },
     [5] = { .speed_rpm = 100,  .accel_ms = 150, .decel_ms = 200, .dir = -1, .stall_current = 390, .forward_deg = 118.08, .torque_level = 120 },
 };

/* 关节6 传感器回零参数 */
static struct {
    int fast_rpm;         /* 快速寻找传感器 */
    int slow_rpm;         /* 碰到IN0后降速 */
    int crawl_rpm;        /* 极慢速离开IN0 */
    int dir;              /* 初始方向 -1反向 */
    int accel_ms;         /* 加减速时间 ms */
} sensor = { 300, 90, 30, -1, 100 };

/* 关节6 零点偏置：传感器回零清零点 = 电机角 0，而机械零点 q0_J6=+90
 * （config ROBOT_JOINT_ZERO_DEG 第6项），故回零后须再正向转 q0_J6 度，机械角才归 0。
 * 直接引用 config，避免与零点表漂移；改 config 即自动同步。 */
static const double joint_zero_deg[ROBOT_JOINT_COUNT] = ROBOT_JOINT_ZERO_DEG;
#define HOME_J6_ZERO_OFFSET_DEG  (joint_zero_deg[5])
#define HOME_J6_ZERO_RPM         100

/* 回零后转角目标（电机角相对位移量）：
 * 1~5 取 stall 表 forward_deg（含 J3=-88 这类"目标非0"的轴）；
 * 6 轴 = q0_J6（目标机械角 0，故电机目标 = 0 + q0_J6）。 */
static double home_forward_deg(int j)
{
    return (j == 6) ? HOME_J6_ZERO_OFFSET_DEG : stall[j].forward_deg;
}
static int home_forward_rpm(int j)
{
    return (j == 6) ? HOME_J6_ZERO_RPM : stall[j].speed_rpm;
}

static int    home_timeout_ms  = 60000;//设置回零超时时间

/* 堵转轮询采样周期 ms：顶死电流爬升极快，周期越短越早命中；
 * 实际节拍由读事务耗时(~7~10ms/轮)主导，已贴近 RS485 物理下限，再小无收益 */
#define HOME_STALL_POLL_MS  1

/* 起步屏蔽窗 ms：躲过启动瞬间浪涌电流误判。
 * 说明：即使电机初始位置就在堵转位置，屏蔽窗结束后电流仍超阈，仍能正确判到位，
 * 只是延迟很短时间，不影响零点正确性。 */
#define HOME_STALL_MASK_MS   150

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

/* home_restore：恢复使能/超差报警/偏差预警默认值；单项失败仅告警不中断 */
static void home_restore(Robot *robot, int joint)
{
    if (motor_enable(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 恢复使能失败\n", joint);
    }
    Sleep(30);
    /* 限位不在此处恢复，回零后先不开启限位，
     * 使 movej 可自由运动到任意角度（回零后位置可能超出软限位）。
     * 恢复限位：motor_set_limit(robot, joint, 1) */
    /* 恢复超差报警：回零期间被关闭（0x000B/0x000C=0），
     * 结束后写回默认 200/100，恢复正常运动的超差保护 */
    if (motor_restore_pos_err_alarm(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 恢复超差报警失败\n", joint);
    }
    /* 恢复位置偏差预警默认值 0x0010=20：回零期间被临时放宽(STALL_PREWARN_STEPS)，
     * 结束后写回，恢复正常运动的失步预警保护 */
    if (motor_set_pos_err_prewarn(robot, joint, 20) != ERR_NONE) {
        printf("[警告] 关节%d 恢复位置偏差预警失败\n", joint);
    }
}

/* home_stall_start：启动堵转回零 —— 关超差报警 → 设加速度 → 设速度 → 按方向运行
 * 返回 ERR_NONE 成功，失败返回对应错误码 */

/* 回零期间把偏差预警 0x0010 放宽到 10000 步（不能写 0；默认 20 步会让顶死瞬间
 * bit10 抢先切断输出，电流判据来不及命中）；放宽后仍约 2s 预警兜底切断输出 */
#define STALL_PREWARN_STEPS   10000

/* 各轴上一帧位置缓存（仅供日志帧差观测，不参与判定） */

/* 【标定临时】各轴采样时刻与本次堵转启动时刻：
 *   周期 = 本帧与上一帧的时间差 → 真实轮询间隔，随并行轴数变化（单轴 ~7-10ms、
 *          整机 4~5 轴 30~50ms），是定窗口点数与死区的基准量；
 *   T+   = 启动后经过时间 → 识别加速段与顶死时刻，用于标定起步屏蔽窗与窗口时长。
 * 待 A3 窗口化改造时并入环形缓冲（每点同时记 pos/cur/时刻），届时移除本组变量。 */
static uint32_t s_stall_t0_ms[7];

/* 起步屏蔽窗截止时刻（ms）：T < mask_end 时不判堵转，躲过加速浪涌 */
static uint32_t s_stall_mask_end_ms[7];

static ErrCode home_stall_start(Robot *robot, int joint)
{
    StallHome *p = &stall[joint];

    s_stall_t0_ms[joint]   = GetTickCount();   /* 【标定】起步时刻 */

    /* 起步屏蔽窗 = HOME_STALL_MASK_MS，躲过启动瞬间浪涌 */
    s_stall_mask_end_ms[joint] = s_stall_t0_ms[joint] + HOME_STALL_MASK_MS;

    /* 关超差报警：顶死改由 home_check_stall 电流超阈判定，不再报警锁存 */
    if (motor_disable_pos_err_alarm(robot, joint) != ERR_NONE) {
        printf("[警告] 关节%d 关闭超差报警失败，顶死仍会报警锁存\n", joint);
    }
    /* 放宽偏差预警到 STALL_PREWARN_STEPS（0x0010 写不了 0，默认 20 步会让顶死瞬间
     * bit10 抢先切断输出、电流判据来不及命中）；挡不住时由兜底2(bit10 超差) 收尾 */
    if (motor_set_pos_err_prewarn(robot, joint, STALL_PREWARN_STEPS) != ERR_NONE) {
        printf("[警告] 关节%d 放宽位置偏差预警失败，顶死仍会提前切断输出\n", joint);
    }
    motor_set_profile(robot, joint, p->accel_ms, p->decel_ms);
    /* 设接近速度：力矩模式的接近速度由 0x00D8 决定，上电默认 300rpm。
     * 每次回零前显式写入 speed_rpm，保证每次运行速度一致，
     * 避免"上电第一次快、之后慢"的不一致现象。 */
    motor_set_speed(robot, joint, p->speed_rpm);
    /* 碰撞回原点力矩模式（手册第49节）：以设定等级恒力矩顶限位，到等级即停，
     * 比纯电流连续运行顶限位更柔和可控。撞到位由 home_check_stall 电流超阈判定
     * （本机固件力矩模式下不报 HOMED/退出RUN，电流才是唯一可靠触发）。
     * 纯电流连续运行回零模式已移除，torque_level 必须 >0，否则无法回零。 */
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

/* home_check_stall：判堵转到位。返回 1=命中（堵转/到位） / 0=运行中。
 * 前置：已关 0x000B/C 报警、放宽 0x0010。判据：
 *   ① 兜底：驱动器报警(bit21) 或 位置超差(bit10) → 判到位；
 *   ② 主判据：电流超阈(>stall_current) → 单帧即判到位（起步屏蔽窗内不判）。
 *      回零统一走碰撞回原点力矩模式（手册第49节），但本机固件在力矩模式下
 *      不报 HOMED/退出RUN，故电流超阈才是唯一可靠触发（阈值须低于该等级保持电流）。
 * 出参 cur_out：可选，读到电流时写回本帧 mA(-1=读失败)；不需要可传 NULL */
static int home_check_stall(Robot *robot, int joint, int *cur_out)
{
    uint32_t st;
    int cur, cur_ok;
    int32_t pos = 0;
    int threshold = stall[joint].stall_current;
    uint32_t now_ms;

    now_ms = GetTickCount();

    /* 先读电流：保证每帧快照都有值，不受位置/状态读取失败影响 */
    cur_ok = 1;
    cur = motor_read_current(robot, joint);
    if (cur < 0 || cur > 3000) { cur_ok = 0; cur = 0; }
    if (cur_out) *cur_out = cur_ok ? cur : -1;

    /* 位置+状态合并读（0x0004 起 4 寄存器，1 事务拿全） */
    if (motor_read_pos_status(robot, joint, &pos, &st) != ERR_NONE) return 0;

    /* 兜底1：驱动器报警（关报警未生效或其它报警） —— 屏蔽窗内也生效，
     * 因为启动瞬间真报警需要立即响应，不能被屏蔽窗挡住 */
    if (st & LEESN_STAT_ALARM) {
        return 1;
    }
    /* 兜底2：位置超差（阈值寄存器写 0 未生效时仍会触发） —— 屏蔽窗内也生效 */
    if (st & LEESN_STAT_OVERRUN) {
        return 1;
    }

    /* 起步屏蔽窗内：主判据不生效，只记录基线，防加速浪涌误判
     * （兜底报警/超差仍生效，因为那是真故障） */
    if (s_stall_t0_ms[joint] != 0 && now_ms < s_stall_mask_end_ms[joint]) {
        return 0;
    }

    /* 主判据：电流超阈（全部轴统一，不看位置），单帧即判到位。
     * 实测（home:1 力矩等级120）：推靠途中电流明显低于阈值（~180~440mA，随轴/负载），
     * 顶到限位瞬间飙到~1500mA（=该等级保持电流）且位置随即钉死。故本机固件在力矩模式下
     * 并不置 HOMED/退出RUN（上方力矩信号块在本机不生效），电流超阈才是力矩模式可靠的到位判据，
     * 阈值须低于该等级保持电流、高于推靠巡航电流。
     * 电流读失败帧不判定，不会误判到位。 */
    if (threshold > 0 && cur_ok && cur > threshold) {
        return 1;
    }

    return 0;
}

/* home_stall_done：堵转收尾 —— 压短减速退出连续运行 → 兜底清报警 → 清零位置
 * 顶死时连续运行命令仍在、直接 move_abs 会被忽略，须先退出（压短减速防持续压紧报警） */
static void home_stall_done(Robot *robot, int joint)
{
    int clear_ok;
    int alarm;

    /* 力矩模式收尾：先停力矩执行、再清力矩模式(写0x009E=0)。
     * 0x009E/0x00CB 为记忆寄存器，不清会残留力矩模式、干扰后续位置模式运动，
     * 故撞到位后必须显式退出。 */
    if (stall[joint].torque_level > 0) {
        motor_torque_run(robot, joint, 0, 0, 0);     /* 停止力矩执行 */
        motor_set_torque_mode(robot, joint, 0, 0);   /* 清除力矩模式 */
    }
    StallHome *p = &stall[joint];

    /* 退出连续运行：否则绝对定位(0x00E8)不执行；压短减速再停，防按原减速持续压紧报警 */
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

    /* 兜底：先清报警再清零（read_alarm 返回 -1 为读失败，仅处理 >0 真报警） */
    alarm = motor_read_alarm(robot, joint);
    if (alarm > 0) {
        int retry, cleared = 0;
        printf("[警告] 关节%d 检测到驱动器报警 码=%d\n", joint, alarm);
        for (retry = 0; retry < 3; retry++) {
            if (motor_clear_alarm(robot, joint) == ERR_NONE) {
                cleared = 1;
                break;
            }
            Sleep(50);   /* 清报警重试间隔 */
        }
        if (!cleared) {
            printf("[警告] 关节%d 清除驱动器报警失败，清零可能被驱动器拒绝\n", joint);
        }
        Sleep(10);
    }

    /* 清零：当前位置置 0，闭环目标=当前位置，驱动器停止顶着。
     * 注：0x00D2 为【无记忆】RAM 寄存器，零点不跨断电保持，上电须重新回零。
     * 禁止在此追加 motor_save_params(0x00DC=1)——既存不住零点，又会把回零期
     * 临时关闭的报警/限位固化进 flash，理由见 motor_reg.h 声明处。 */
    clear_ok = (motor_clear_pos(robot, joint) == ERR_NONE);
    Sleep(30);
    if (!clear_ok) {
        printf("[错误] 关节%d 堵转清零失败，后续退让 move_abs 将基于旧零点，位置会错\n", joint);
    }
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

/* home_wait_inpos：等待运动到位，使用严格判据（无报警+退出运行+位置在容差内），
 * 防未启动/半途停机误判。返回 1=到位 / 0=超时 / -1=报警异常 */
static int home_wait_inpos(Robot *robot, int joint, int32_t target_steps, int timeout_ms)
{
    uint32_t start_ms = GetTickCount();
    int saw_active = 0;

    while ((GetTickCount() - start_ms) < (uint32_t)timeout_ms) {
        int q = home_inpos_query(robot, joint, target_steps);
        if (q == 1) {
            if (saw_active) return 1;        /* 曾启动、现已到位 → 成功 */
            /* 没见过 ACTIVE 但已到位：可能目标=当前位置，也可能没启动。
             * 再等几帧确认不是初始状态巧合。 */
            Sleep(20);
            if (home_inpos_query(robot, joint, target_steps) == 1) return 1;
            continue;
        }
        if (q == -1) return -1;            /* 报警/超差 */
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

/* home_wait_stall：轮询等堵转命中（home_check_stall）。返回 1=命中 / 0=超时 */
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

/* home_stall_forward：堵转清零后退让 forward_deg（仅非 0 轴），避免久顶硬限位。
 * 【相对位移】与整机路径 home_goto_pose 保持同一口径：实测清零后 0x0004 常非 0
 * （0x00D2 未必生效），按绝对目标发令会被起点偏移吃掉行程、甚至反向撞回限位，
 * 故以清零后实际位置为基准发相对位移，保证行程恒为 forward_deg。
 * 使用严格到位判据（home_inpos_query：状态 + 位置容差）。 */
static void home_stall_forward(Robot *robot, int joint)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    StallHome *p = &stall[joint];
    int32_t delta, pos = 0, target;
    int pos_ok = 0, rc;

    if (p->forward_deg == 0.0) return;

    delta  = DEG2STEPS(p->forward_deg, reductions[joint - 1]);
    pos    = motor_read_position(robot, joint, &pos_ok);
    target = pos_ok ? (pos + delta) : delta;   /* 读不到位置：退回绝对目标 */
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
                 joint, p->forward_deg, (int)target,
                 pos2_ok ? (int)pos2 : -99999,
                 pos2_ok ? (int)(target - pos2) : -99999);
    } else if (rc < 0) {
        printf("[警告] 关节%d 退让 %.1f° 报警/异常\n", joint, p->forward_deg);
    }
}

/* robot_torque_probe：碰撞回原点力矩模式诊断（torque:N:L）。
 * 在关节 joint 上以力矩等级 level 启动碰撞回原点，每 200ms 打印状态字/电流/位置，
 * 直到检测到 bit15(HOMED) 或退出 RUN_ACTIVE（即手册所述"撞到位"），或超时 15s。
 * 全程只读与打印，不执行清零/转角，撞到位后停止力矩模式并退出，便于人工观察信号。
 * 用途：标定力矩等级与确认到位判据前，先实机采集一帧"到位瞬间"的寄存器值。 */
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
    /* 诊断期间放宽偏差预警，避免顶死被 bit10 提前切断（同 home_stall_start） */
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

    /* 收尾：停力矩执行并清力矩模式，避免记忆态卡在力矩模式 */
    motor_torque_run(robot, joint, 0, 0, 0);
    motor_set_torque_mode(robot, joint, 0, 0);
    home_restore(robot, joint);
    printf("=== 力矩碰撞诊断结束：关节%d 等级=%d（HOMED=%d 退出RUN=%d）===\n",
             joint, level, seen_homed, seen_stop);
    return ERR_NONE;
}

/* ====================== 关节6：传感器回零 ====================== */

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
    int pos_fail_cnt;        /* 位置读失败连续计数，用于 SEN_FWD_LEAVE 保护 */
} SensorCtx;

/* 位置读失败连续阈值：SEN_FWD_LEAVE 阶段连续读不到位置时急停，
 * 防止位置判据失效导致电机一直转出去 */
#define SEN_POS_FAIL_MAX   10

/* sensor6_set_rpm：双写 0x009A+0x00D8 速度源（Bug1：连续运行只认 0x009A，
 * 单写 0x00D8 会按驱动器记忆速度运行） */
static void sensor6_set_rpm(Robot *robot, int rpm)
{
    if (motor_set_speed16(robot, 6, rpm) != ERR_NONE) {
        printf("[警告] 关节6 写连续运行速度源0x009A(%drpm)失败，按记忆速度运行\n", rpm);
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
    c->pos_fail_cnt = 0;

    rc = home_arm(robot, 6);
    if (rc != ERR_NONE) {
        printf("[警告] 关节6 arm 失败：%s\n", err_str(rc));
        return rc;
    }
    motor_set_profile(robot, 6, sensor.accel_ms, sensor.accel_ms);
    /* 预设快速速度：双写 0x009A+0x00D8，确保回零启动时速度就是配置值，
     * 不依赖驱动器默认值或上次残留值，避免上电第一次速度不一致。 */
    sensor6_set_rpm(robot, sensor.fast_rpm);

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

    /* 读传感器状态 */
    if (motor_read_status(robot, 6, &inputs) != ERR_NONE) return 0;
    in0 = (inputs & SENSOR_IN0) ? 1 : 0;
    in1 = (inputs & SENSOR_IN1) ? 1 : 0;
    cur = motor_read_current(robot, 6);
    if (cur < 0 || cur > 3000) cur = 0;
    c->last_cur = cur;
    pos = motor_read_position(robot, 6, &pos_ok);

    /* SEN_FWD_LEAVE 阶段位置读失败保护：
     * 该阶段依赖位置判据判断是否开够 3°，若位置持续读失败电机将一直正转。
     * 连续 SEN_POS_FAIL_MAX 次读失败则急停，防止超出行程。 */
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


    /* 状态机切换 */
    switch (ph) {
    case SEN_INIT:
        if (in0) {
            /* 情况1：初始就在IN0上，正向快速开3度（离开挡片） */
            c->pos_base = pos_ok ? pos : 0;
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_LEAVE;
        } else if (in1) {
            /* 情况2：初始碰IN1，正向快速找IN0 */
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_FIND;
        } else {
            /* 情况3：初始无传感器，反向快速找IN0 */
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, sensor.dir);
            c->ph = SEN_REV_FIND;
        }
        break;

    case SEN_FWD_LEAVE:
        /* 正向快速开3度：位移到位后反向快速回找 IN0 */
        if (pos_ok && (pos - c->pos_base) >= DEG2STEPS(3.0, 50)) {
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, sensor.dir);
            c->ph = SEN_REV_FIND;
        }
        break;

    case SEN_REV_FIND:
        if (in0) {
            /* 反向途中碰到IN0：改慢速正向离开（on->off 清零） */
            sensor6_set_rpm(robot, sensor.crawl_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->in0_was_on = 1;
            c->leave_start = GetTickCount();
            c->ph = SEN_SLOW_LEAVE;
        } else if (in1) {
            /* 反向途中先碰IN1：转入情况2（正向快速找IN0） */
            sensor6_set_rpm(robot, sensor.fast_rpm);
            Sleep(10);
            motor_run(robot, 6, +1);
            c->ph = SEN_FWD_FIND;
        }
        break;

    case SEN_FWD_FIND:
        if (in0) {
            /* 正向碰到IN0：改慢速正向离开（穿过挡片到正向沿清零） */
            sensor6_set_rpm(robot, sensor.crawl_rpm);
            c->in0_was_on = 1;
            c->leave_start = GetTickCount();
            c->ph = SEN_SLOW_LEAVE;
        }
        break;

    case SEN_SLOW_LEAVE:
        if (c->in0_was_on && !in0) {
            /* 慢速离开IN0（on->off 沿），急停并清零，完成 */
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
        /* 清零：IN0 离开沿即原点。注意 0x00D2 为【无记忆】RAM 寄存器，
         * 零点不跨断电保持，上电须重新回零；此处不得追加
         * motor_save_params(0x00DC=1)，理由见 motor_reg.h 声明处。 */
        motor_clear_pos(robot, 6);
        Sleep(30);
    } else {
        /* 主循环整体超时等未走 tick 超时分支的兜底 */
        if (!c->failed) {
            c->failed = 1;
            motor_estop(robot, 6);
        }
        printf("[警告] 关节6 传感器回零失败：%s\n", err_str(ERR_TIMEOUT));
    }

    home_restore(robot, 6);
    if (c->done) printf("关节6 回零完成\n");
}

/* 前向声明：定义在本文件末尾（单轴回零路径复用同一转角逻辑） */
static void home_goto_pose(Robot *robot, int only_joint);

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
    /* 与整机路径同口径：清零后再走 q0_J6 零点偏置，机械角才归 0 */
    if (c.done) home_goto_pose(robot, 6);
    return c.done ? ERR_NONE : ERR_TIMEOUT;
}

/* ====================== 公开接口 ====================== */

/* home_goto_pose：回零后统一 movej 到 forward_deg（含 6 轴零点偏置）。
 * only_joint=0 表示 1~6 全轴；=N 表示只走第 N 轴（单轴回零路径复用）。
 * 先一次性发全部 movej 再统一轮询；单轴异常只结算自身不拖累其余轴。
 * 注意：调用前应已恢复限位/报警保护，转角过程有完整保护。 */
static void home_goto_pose(Robot *robot, int only_joint)
{
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    uint8_t pend[7] = {0};   /* 1~6：已发 movej、待等待到位 */
    int32_t tgt[7] = {0};    /* 1~6：期望绝对脉冲 */
    uint32_t start_ms;
    int remain = 0, j;
    int j_first = (only_joint >= 1 && only_joint <= 6) ? only_joint : 1;
    int j_last  = (only_joint >= 1 && only_joint <= 6) ? only_joint : 6;

    /* 第一遍：全部发出 movej（各轴同时开始运动） */
    for (j = j_first; j <= j_last; j++) {
        double fdeg = home_forward_deg(j);
        int    frpm = home_forward_rpm(j);
        if (robot_is_masked(robot, j)) continue;
        if (fdeg == 0.0) continue;

        /* 发指令前快照状态/位置并暴露异常，防电机没动却被轮询误判为到位。
         * 【相对位移】实测清零后 0x0004 常非 0（0x00D2 未必生效），若仍按
         * "绝对目标=DEG2STEPS(forward_deg)" 发令，实际行程会被起点偏移吃掉，
         * 甚至反向：关节4 曾因起点 +6.8° 大于目标 +5° 而倒走 1.8° 撞回硬限位。
         * 故改为以【清零后实际位置】为基准发相对位移，保证行程恒为 forward_deg。 */
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
                tgt[j] = delta;   /* 读不到位置：退回绝对目标（保持原行为） */
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

    /* 第二遍：统一轮询等待，直到全部结算 */
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
                Sleep(HOME_STALL_POLL_MS);   /* 读失败：避让一帧下轮再试 */
            }
        }
        if (remain > 0) Sleep(HOME_STALL_POLL_MS);
    }
}

/* robot_home：整机回零（并行 + 事件联动）
 * ①{1,2,3,5}堵转与 6 传感器并行归零 ②3 号到位后启动 4 号（依赖 3 停稳防干涉）
 * ③全轴成功后统一转角 forward_deg，有失败轴则跳过
 * 返回 ERR_NONE 全部成功 / ERR_TIMEOUT 有轴失败或超时 / ERR_ARG 参数错误 */
ErrCode robot_home(Robot *robot)
{
    const int group_stall[] = {2, 3, 5, 1};
    uint8_t active[7] = {0};    /* 已启动参与回零的轴 */
    uint8_t sdone[7] = {0};     /* 已回零成功 */
    uint8_t sfail[7] = {0};     /* 回零失败/超时 */
    SensorCtx c6;
    uint32_t start_ms;
    int j, all_ok;
    int any_fail = 0;

    if (robot == NULL) return ERR_ARG;

    /* 阶段1：{1,2,3,5} 并行堵转归零 与 关节6 传感器回零 同时启动 */

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

    /* 主循环：堵转组与 6 传感器并行推进；3 号到位即联动启动 4 号 */
    start_ms = GetTickCount();
    while ((GetTickCount() - start_ms) < (uint32_t)home_timeout_ms) {
        int all_done = 1;

        /* 6 轴传感器推进一帧 */
        if (active[6] && !sdone[6] && !sfail[6]) {
            int r = sensor6_tick(robot, &c6);
            if (r == 1)       sdone[6] = 1;
            else if (r == -1) { sfail[6] = 1; any_fail = 1; }
            else              all_done = 0;
        }

        /* 堵转组轮询（4 号由 3 号到位事件加入 active） */
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
                /* 阶段2：3 号到位 -> 立即启动 4 号 */
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

    /* 超时兜底：未完成堵转轴急停并标记失败 */
    for (j = 1; j <= 5; j++) {
        if (!active[j] || sdone[j]) continue;
        printf("[警告] 关节%d 堵转回零超时\n", j);
        motor_estop(robot, j);
        sfail[j] = 1;
        any_fail = 1;
    }
    if (active[6] && !sdone[6]) {
        sfail[6] = 1;   /* 未在 tick 超时分支收尾的兜底 */
        any_fail = 1;
    }
    if (active[6]) {
        sensor6_finish(robot, &c6);   /* done->清零校验；失败->急停；统一恢复 6 轴限位 */
    }

    /* 阶段3：全轴回零成功后统一转角，有失败轴则跳过防乱跑。
     * 转角前先恢复 1~5 轴的限位/报警/预警保护，转角过程有完整保护。 */
    all_ok = 1;
    for (j = 1; j <= 6; j++) {
        if (robot_is_masked(robot, j)) continue;
        if (!active[j] || sfail[j]) {
            all_ok = 0;
            break;
        }
    }
    if (all_ok) {
        /* 转角前恢复 1~5 轴保护（限位/超差报警/偏差预警），
         * 保证转角过程中限位和超差保护生效，6 轴已在 sensor6_finish 恢复 */
        for (j = 1; j <= 5; j++) {
            if (robot_is_masked(robot, j)) continue;
            home_restore(robot, j);
            Sleep(20);
        }
        home_goto_pose(robot, 0);
    } else {
        printf("[警告] 回零：存在未归零/失败轴，跳过统一转角\n");
        /* 失败场景也要恢复 1~5 轴保护（6 轴已在 sensor6_finish 恢复） */
        for (j = 1; j <= 5; j++) {
            if (robot_is_masked(robot, j)) continue;
            home_restore(robot, j);
            Sleep(20);
        }
    }

    return any_fail ? ERR_TIMEOUT : ERR_NONE;
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
        printf("=== 单轴传感器回零：关节6 ===\n");
        return home_joint6(robot);
    }
    p = &stall[joint];

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

    /* 等待堵转到位 */
    hit = home_wait_stall(robot, joint, home_timeout_ms);
    if (!hit) {
        printf("[警告] 关节%d 堵转回零超时\n", joint);
        motor_estop(robot, joint);
        home_restore(robot, joint);
        return ERR_TIMEOUT;
    }
    home_stall_done(robot, joint);

    /* 堵转成功 → 自动跑到配置角度 forward_deg（0 则停在清零点） */
    home_stall_forward(robot, joint);

    home_restore(robot, joint);
    printf("关节%d 单轴回零完成（目标 %.1f°）\n", joint, p->forward_deg);
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

    printf("单轴回零：关节%d 回零后自动运动到 %.1f° ...\n", joint, angle_deg);

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
            printf("[错误] 关节%d 自动运动到 %.1f° 失败：%s\n",
                      joint, angle_deg, err_str(rc));
        } else {
            printf("关节%d 自动运动到 %.1f° ...\n", joint, angle_deg);
            {
                const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
                int32_t target = DEG2STEPS(angle_deg, reductions[joint - 1]);
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

    /* 恢复原屏蔽状态 */
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
