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
#include "kinematics/joint_zero.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>      /* isfinite/isnan：robot_movej 的目标角有限性检查 */

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
    /* 1=该轴 0x0024（每转脉冲数）已被写入【并读回确认】等于 ENCODER_STEPS_PER_REV。
     * 由 robot_apply_subdivision 维护；motor_move_abs 出口据此拦截运动。
     * 【为什么单独立一个状态】写成功≠写对了。0x0024 出厂是 4000、断电即回 4000，
     * 而 DEG2STEPS 用的是 ENCODER_STEPS_PER_REV=10000；没对齐就下发，同样一个
     * "90°"会被驱动器理解成 2.5 倍（2026-09-19 实测：250000 步在 4000 下 = 225°）。
     * 更阴的是：编码器在电机侧，下发和读回用的是同一个常量 ⇒ 程序永远自洽、
     * getpos 永远显示"正确"，只有真机转过头才看得出来。⇒ 只能靠"写后读回比对"证明。 */
    int subdiv_ok[ROBOT_JOINT_COUNT];
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

/* robot_request_noread：与 robot_request 同一把锁，但不等从站响应。
 *
 * 【为什么"不等响应"也必须持锁】这里曾经是个真实的坑：
 * motor_write_i32_noread 原先以"不读响应就不需要加锁"为由直接调
 * modbus_transact_noread，绕过了本文件的总线锁。后果有两层：
 *   1) 物理层撞车：RS485 半双工单主站，两个线程同时驱动总线发送，
 *      两帧在线上叠加，双方都收到畸形帧，CRC 校验失败；
 *   2) 冲掉对方数据：modbus_transact_noread 第一件事是 flush
 *      （PurgeComm PURGE_RXCLEAR），会把【另一线程已经收到、还没读走】
 *      的响应字节直接从驱动缓冲里清掉，对方必然读到超时。
 * 现场症状（2026-09-18）：跑 diag 时后台巡检线程被 noread 帧踩踏，
 * 刷出"关节2~6 掉线（状态读取无响应）"，diag 一结束立刻"恢复在线"。
 * 结论：只要往总线上发字节，就必须持这把锁——与是否等响应无关。 */
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

/* robot_apply_subdivision：按从站 ID 逐轴写入细分，使驱动器 0x0024
 * 对齐 ENCODER_STEPS_PER_REV（RAM 生效断电丢失，每次上电写入）。
 * 逐轴写+读回校验；屏蔽/离线轴跳过，失败不阻断启动。 */
void robot_apply_subdivision(Robot *robot)
{
    int i, ok_cnt = 0, fail_cnt = 0;

    if (robot == NULL) {
        return;
    }
    /* 先全部清零：本函数可能被重复调用（如重新上电后），不能让上一次的
     * "成功"残留下来骗过闸门。subdiv_ok 只在【本次】读回比对通过后才置 1。 */
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
            robot->subdiv_ok[i] = 1;      /* 唯一置 1 的地方：写后读回确实等于配置值 */
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
        /* 成功也要留痕：这道闸门平时一次都不会触发，若完全静默就无法区分
         * "对齐成功"和"根本没跑"。启动时看到这一行才算本次对齐确实生效。 */
        printf("[细分对齐] %d 轴已确认：0x0024 = %d 脉冲/转（出厂 4000，写后读回比对通过）\n",
               ok_cnt, (int)ENCODER_STEPS_PER_REV);
    }
}

/* robot_subdivision_ok：该关节的每转脉冲数是否已被写入【并读回确认】。
 * 返回 1 = 可以信任 DEG2STEPS/STEPS2DEG；0 = 不可信，禁止下发运动。
 *
 * 离线 / 被屏蔽的关节一律返回 1：它们本来就不会收到运动指令，
 * 不能因为一台坏电机就把其余五轴也一起锁死。
 *
 * 【为什么必须显式问一次】见 struct Robot 里 subdiv_ok 的注释 —— 这类错误
 * 软件内部永远自洽（下发与读回用同一个常量），只有外部物理测量才看得出来，
 * 所以"看起来一切正常"不构成证据，必须有读回比对留下的记录。 */
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

    /* 启动时查询各电机在线状态（读 0x0066 设备地址） */
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

    /* 按从站 ID 把细分对齐到 ENCODER_STEPS_PER_REV（RAM 生效、断电丢失，
     * 每次启动必须写；失败不阻断启动，后续角度换算可能失真） */
    robot_apply_subdivision(r);

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
    printf("关节%d 已屏蔽\n", joint);
    return ERR_NONE;
}

ErrCode robot_unmask(Robot *robot, int joint)
{
    if (robot == NULL || joint < 1 || joint > ROBOT_JOINT_COUNT) {
        return ERR_ARG;
    }
    robot->masked[joint - 1] = 0;
    printf("关节%d 已恢复\n", joint);
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

/* robot_disable：失能指定关节（写 0x00D4 = 1，释放马达），返回 ErrCode */
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

/* robot_angle_in_soft_limit：目标机械角是否落在软限位内（纯函数，可离线单测）。
 *
 * 【为什么不放进 robot_movej】回零（home.c）也走 robot_movej，而回零的原理
 * 就是"朝一个方向顶到堵转"—— 起点已经贴着限位、过程中必然越限。把这条
 * 检查塞进 robot_movej 会让六轴全部回不了零。所以它只用于【用户显式指定的
 * 目标角】（CLI 的 MoveJ / MoveL 目标），放在下发之前由调用方决定要不要拦。
 *
 * 【返回值】1 = 在限位内（合法）；0 = 越限；-1 = 关节号非法。
 *   关节号非法与"越限"分开：前者是调用方的 bug，后者是操作员的输入问题，
 *   两种错误的处置完全不同，混成同一个 0 会让报错文案说谎。
 * out_min / out_max 可为 NULL；非 NULL 时写出该轴限位区间，供报错打印。 */
int robot_angle_in_soft_limit(int joint, double deg, double *out_min, double *out_max)
{
    const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;

    if (joint < 1 || joint > ROBOT_JOINT_COUNT) return -1;
    if (out_min != NULL) *out_min = lmin[joint - 1];
    if (out_max != NULL) *out_max = lmax[joint - 1];
    return (deg >= lmin[joint - 1] && deg <= lmax[joint - 1]) ? 1 : 0;
}

/* robot_readback_anomaly：读回的六轴机械角是否"不可能为真"（纯函数，可单测）。
 *
 * 【由来 2026-09-19】一次 MoveL 途中六轴读回同时变垃圾：
 *     J1=176.66 J2=-74.58 J3=180.03 J4=-4.27 J5=-115.44 J6=-84.89
 * 其中 J2/J3/J5 越软限位（分别超 2.58° / 0.03° / 20.44°），末端偏差读数
 * 冻结在 258.92mm 一动不动。当时"等到位"循环只能干等到 60s 超时，
 * 而真超时后急停指令还发不出去 —— 整整一分钟完全失控。
 *
 * 【判据】两条，任一命中即异常。都刻意保守：宁可漏报，不可误报
 * （误报会把一次完全正常的运动打断，那是更常见的工况）。
 *   ① 任一轴越软限位幅度 > big_margin（默认 15°）
 *      规划出来的目标角全在限位内，真的越限 15° 只可能是跑飞或读回是假的。
 *   ② 越限超过 0.5° 的轴数 >= 3
 *      机械臂不可能真的三根轴同时越软限位还毫无报警。
 *      0.5° 的死区是为了容忍到位 overshoot —— 限位边界上的正常抖动不能算。
 *
 * 【为什么必须传 ok[]】读失败的轴在调用方常被填成 0，而 0 对 J3（限位 30~180）
 * 来说是"越限 30°"—— 不区分就会把【单次读失败】误判成总线异常。
 * 只统计本轮真正读到的轴。
 *
 * 返回 1=异常 / 0=正常。bad_joint、bad_excess 可为 NULL，非 NULL 时写出
 * 超得最狠的那一根轴(1..6)与超出量(度，>0)。 */
int robot_readback_anomaly(const double deg[6], const int ok[6],
                           double big_margin_deg,
                           int *bad_joint, double *bad_excess)
{
    const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
    const double DEADZONE = 0.5;   /* 容忍到位 overshoot 的死区(度) */
    int n_over = 0, worst_j = 0;
    double worst_ex = 0.0;
    int i;

    if (deg == NULL || ok == NULL) return 0;

    for (i = 0; i < 6; i++) {
        double ex = 0.0;
        if (!ok[i]) continue;                 /* 没读到就别拿来判 */
        if (deg[i] < lmin[i]) ex = lmin[i] - deg[i];
        else if (deg[i] > lmax[i]) ex = deg[i] - lmax[i];
        else continue;
        if (ex > DEADZONE) n_over++;
        if (ex > worst_ex) { worst_ex = ex; worst_j = i + 1; }
    }
    if (bad_joint  != NULL) *bad_joint  = worst_j;
    if (bad_excess != NULL) *bad_excess = worst_ex;

    if (worst_ex > big_margin_deg) return 1;  /* 判据①：单轴离谱地越限 */
    if (n_over >= 3) return 1;                /* 判据②：多轴同时越限 */
    return 0;
}

/* robot_movej：关节绝对运动到指定【机械角】（立三 0x00E8~0x00E9 绝对位置，
 * 速度写 0x00D8~0x00D9，单位 0.01 rpm），返回 ErrCode。
 * 入参为机械角（机械零位为 0，与 status/fk/ik 同一口径），
 * 内部经零点标定换算为上位机电机角后再转脉冲下发。 */
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
    /* 【NaN/Inf 检查】单轴 MoveJ 此前完全没有这道检查，直接 DEG2STEPS 就写
     * 0x00E8 了 —— NaN 转 int32 会变成 ±2147483647 附近的垃圾，驱动器收到
     * 天文数字目标就猛冲、永不到位、超时急停（2026-09-18 实测甩出 34mm）。
     * isfinite 用 <math.h>，本文件已包含。 */
    if (!isfinite(angle_deg)) {
        printf("[错误] 关节%d 目标角不是有限数（%s），已拒绝下发\n",
               joint, isnan(angle_deg) ? "NaN" : "Inf");
        return ERR_ARG;
    }
    /* 机械角 -> 电机角（仅本关节有效，其余位置零不影响单轴换算） */
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
    return robot_read_status(robot, joint, status);
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

/* robot_read_position_deg：读取关节实时位置并换算为【机械角】（度）。
 * 脉冲 -> 电机角 ->（减零点）-> 机械角，与 status/fk/ik/robot_movej 同一口径。
 * ok 指示读取是否成功；失败返回 0.0。 */
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

/* robot_read_current_ma：读取关节实时电流（mA，0x001A），失败返回 -1 */
int robot_read_current_ma(Robot *robot, int joint)
{
    return motor_read_current(robot, joint);
}

/* robot_read_speed_rpm：读取关节实时速度（rpm，寄存器 0x00D6~0x00D7，分辨率 0.01rpm），失败返回 -1
 * 【修正】底层 motor_read_speed 读的是 0x00D6，不是 0x0019：0x0019 为 INT16，
 * 且 SV118+ 固件语义已变为"实际给定电流"（详见 robot_internal.h 定义处说明）。 */
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
