/*
 * home.c —— 堵转模式回零实现（LEESN 立三体系）
 * ------------------------------------------------------------
 * 所属模块：控制层（control）
 * 对外接口：robot_home
 * 依赖模块：control/robot（内部接口）、comm/modbus_rtu、config/robot_config、utils/logger
 *
 * 回零策略：立三电机无"堵转电流/堵转逻辑"专用寄存器，改为
 * 速度模式顶硬限位 → 电流超阈值判定堵转 → 急停(0x00C8=0x0100) →
 * 清零位置(0x00D2~0x00D3=0) → 运动到机械原点。
 */

#include "control/home.h"
#include "control/robot_internal.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "utils/logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* ================= 内部辅助函数 ================= */

/* 写单个寄存器并检查响应：返回 ErrCode（ERR_NONE 成功 / 对应错误码） */
static ErrCode home_write_reg(Robot *robot, int joint, uint16_t reg, uint16_t val)
{
    uint8_t frame[16];
    ModbusFrame resp;
    size_t len = modbus_build_write_single(joint_slave(joint), reg, val, frame);
    return robot_request(robot, frame, len, &resp);
}

/* 回零启动前握手：使能 → 关限位 → 设运行速度 → 启动正/反向运行。
 * 立三无堵转电流/堵转逻辑专用寄存器，碰撞判定全部由 PC 侧读 0x001A
 * 实时电流完成（见 home_grp_stall_scan）。 */
static ErrCode home_arm_motor(Robot *robot, int joint)
{
    static const int home_dir[ROBOT_JOINT_COUNT] = HOME_DIR;
    int dir = home_dir[joint - 1];
    ErrCode rc;

    /* ① 使能电机（0x00D4 写 0 = 马达使能） */
    rc = home_write_reg(robot, joint, LEESN_REG_ENABLE, LEESN_CMD_ENABLE);
    if (rc != ERR_NONE) { LOG_WARN("关节%d 使能失败：%s", joint, err_str(rc)); return rc; }
    Sleep(50);

    /* ② 关限位（纯靠电流检测碰撞，避免限位开关提前截停） */
    rc = home_write_reg(robot, joint, LEESN_REG_LIMIT, 0x0000);
    if (rc != ERR_NONE) { LOG_WARN("关节%d 关限位失败：%s", joint, err_str(rc)); return rc; }
    Sleep(20);

    /* ③ 设运行速度 0x00D8~0x00D9（INT32，0.01 rpm；立三 V126 目标速度寄存器，
     *    与 robot_movej 一致。0x009A 为 SV113 以下固件 UINT16 速度寄存器，回零不再使用） */
    {
        int32_t vel = LEESN_RPM_TO_VELREG((double)HOME_SPEED_RPM);
        uint16_t vals[2];
        uint8_t frame[32];
        ModbusFrame resp;
        size_t len;
        vals[0] = (uint16_t)((uint32_t)vel & 0xFFFFu);
        vals[1] = (uint16_t)(((uint32_t)vel >> 16) & 0xFFFFu);
        len = modbus_build_write_multi(joint_slave(joint), LEESN_REG_VEL_RUN, vals, 2, frame);
        rc = robot_request(robot, frame, len, &resp);
    }
    if (rc != ERR_NONE) { LOG_WARN("关节%d 设速度失败：%s", joint, err_str(rc)); return rc; }
    Sleep(20);

    /* ④ 启动运行：0x00C8 写 1=正转 / 257=反转 */
    rc = home_write_reg(robot, joint, LEESN_REG_RUN_CTRL,
                        (dir > 0) ? LEESN_CMD_RUN_CW : LEESN_CMD_RUN_CCW);
    if (rc != ERR_NONE) { LOG_WARN("关节%d 启动运行失败：%s", joint, err_str(rc)); return rc; }

    LOG_INFO("关节%d 堵转回零已启动 (%+d rpm)", joint, dir * (int)HOME_SPEED_RPM);
    return ERR_NONE;
}

/* 急停电机（写 0x00C8 = 0x0100） */
static ErrCode home_stop_motor(Robot *robot, int joint)
{
    return home_write_reg(robot, joint, LEESN_REG_RUN_CTRL, LEESN_CMD_ESTOP);
}

/* 清零位置：写 0x00D2~0x00D3 = 0（设定当前电机绝对位置为 0） */
static ErrCode home_clear_position(Robot *robot, int joint)
{
    uint16_t vals[2] = {0, 0};
    uint8_t frame[32];
    ModbusFrame resp;
    size_t len = modbus_build_write_multi(joint_slave(joint), LEESN_REG_SET_POS, vals, 2, frame);
    return robot_request(robot, frame, len, &resp);
}

/* 恢复电机正常工作状态：使能 + 开限位（立三无堵转逻辑寄存器，无需恢复） */
static ErrCode home_restore_motor(Robot *robot, int joint)
{
    ErrCode rc;
    rc = home_write_reg(robot, joint, LEESN_REG_ENABLE, LEESN_CMD_ENABLE);
    if (rc != ERR_NONE) { LOG_WARN("关节%d 恢复使能失败：%s", joint, err_str(rc)); }
    Sleep(30);
    rc = home_write_reg(robot, joint, LEESN_REG_LIMIT, 0x0001);
    if (rc != ERR_NONE) { LOG_WARN("关节%d 恢复限位失败：%s", joint, err_str(rc)); }
    return rc;
}

/* 扫描一组关节堵转状态：电流超阈值→急停清零标记到位，返回1=全员到位。
 * 立三无碰撞停状态位，堵转判定完全依赖 0x001A 实时电流；
 * 运行状态取 0x0006 状态字 bit8~9（11=正在运行）。 */
static int home_grp_stall_scan(Robot *robot, const int *joints, int cnt,
                               uint8_t *done_bits, uint32_t grp_start_ms)
{
    static const uint16_t stall_cur[ROBOT_JOINT_COUNT + 1] = HOME_STALL_CURRENT_MA;
    static uint32_t stuck_since[ROBOT_JOINT_COUNT];
    static int stuck_init = 0;
    int all_done = 1;
    int j;
    uint32_t now = GetTickCount();

    if (!stuck_init) {
        for (j = 0; j < ROBOT_JOINT_COUNT; j++) stuck_since[j] = 0;
        stuck_init = 1;
    }

    for (j = 0; j < cnt; j++) {
        int joint = joints[j];
        uint16_t st;
        ErrCode rc;
        int cur, pos_ok = 0;
        int32_t pos;

        if (robot_is_masked(robot, joint)) continue;
        if (*done_bits & (1u << (joint - 1))) continue;  /* 已到位或已跳过 */

        all_done = 0;

        rc = robot_read_status(robot, joint, &st);
        if (rc != ERR_NONE) continue;  /* 通信失败，跳过本轮 */

        cur = robot_read_current_ma(robot, joint);
        if (cur < 0) cur = 0;
        /* 电流读数有效性检查：超过 3000mA 视为通信异常值（保守上限，
         * 实机按电机额定电流校准），不用于堵转判定 */
        if (cur > 3000) {
            LOG_WARN("关节%d 电流读数异常 %dmA（>3000），本次忽略电流判定", joint, cur);
            cur = 0;
        }

        pos = robot_read_position_steps(robot, joint, &pos_ok);

        LOG_INFO("关节%d 状态=0x%04X 电流=%dmA 位置=%d脉冲 (阈值=%dmA)",
                 joint, st, cur, pos_ok ? (int)pos : -99999, stall_cur[joint] + HOME_STALL_MARGIN_MA);

        /* 到位判定：电流 > 阈值+余量（堵转/碰撞） */
        if (stall_cur[joint] > 0 && cur > (int)(stall_cur[joint] + HOME_STALL_MARGIN_MA)) {
            int clear_ok;
            home_stop_motor(robot, joint);
            Sleep(50);
            clear_ok = (home_clear_position(robot, joint) == ERR_NONE);
            Sleep(30);
            pos = robot_read_position_steps(robot, joint, &pos_ok);
            *done_bits |= (1u << (joint - 1));
            stuck_since[joint - 1] = 0;
            LOG_INFO("关节%d 堵转到位 (状态=0x%04X 电流=%dmA)，清零%s，清零后位置=%d脉冲",
                     joint, st, cur, clear_ok ? "成功" : "失败",
                     pos_ok ? (int)pos : -99999);
            continue;
        }

        /* 正在运行：正常，复位保活计时 */
        if ((st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) {
            stuck_since[joint - 1] = 0;
            continue;
        }

        /* 非运行非堵转：超最小运行窗口后累计计时，超时重新下发运行命令 */
        if ((now - grp_start_ms) >= HOME_MIN_RUN_MS) {
            if (stuck_since[joint - 1] == 0) {
                stuck_since[joint - 1] = now;
            } else if ((now - stuck_since[joint - 1]) >= HOME_STUCK_MS) {
                static const int home_dir[ROBOT_JOINT_COUNT] = HOME_DIR;
                int dir = home_dir[joint - 1];
                home_write_reg(robot, joint, LEESN_REG_RUN_CTRL,
                               (dir > 0) ? LEESN_CMD_RUN_CW : LEESN_CMD_RUN_CCW);
                LOG_WARN("关节%d 非运行态(状态=0x%04X)超%ums，重新下发运行命令",
                         joint, st, (unsigned)HOME_STUCK_MS);
                stuck_since[joint - 1] = 0;
            }
        }
    }
    return all_done;
}

/* 轮询一组关节直到全部堵转到位或超时，返回0成功 / -1超时 */
static int wait_group_stall(Robot *robot, const int *joints, int cnt, uint32_t timeout_ms)
{
    uint8_t done_bits = 0;
    uint32_t start_ms = GetTickCount();
    int j;

    /* 先 arm 所有电机，arm 失败的标记为跳过（计入 done_bits） */
    for (j = 0; j < cnt; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        if (home_arm_motor(robot, joints[j]) != ERR_NONE) {
            LOG_WARN("关节%d arm 失败，跳过", joints[j]);
            done_bits |= (1u << (joints[j] - 1));  /* 标记为已完成，不等待 */
        }
        Sleep(10);  /* 台间间隔，防帧粘连 */
    }

    /* 轮询堵转到位 */
    while ((GetTickCount() - start_ms) < timeout_ms) {
        if (home_grp_stall_scan(robot, joints, cnt, &done_bits, start_ms)) {
            return 0;
        }
        Sleep(200);  /* 200ms 轮询间隔 */
    }
    return -1;
}

/* 轮询一组关节直到全部非运行态（到位或停止），超时返回 -1 */
static int wait_group_idle(Robot *robot, const int *joints, int cnt)
{
    uint32_t elapsed = 0;

    LOG_INFO("等待运动到位...");
    while (elapsed < HOME_GRP_TIMEOUT_MS) {
        int all_done = 1;
        int j;
        for (j = 0; j < cnt; j++) {
            int joint = joints[j];
            uint16_t st;
            ErrCode rc;
            if (robot_is_masked(robot, joint)) {
                continue;
            }
            rc = robot_read_status(robot, joint, &st);
            if (rc != ERR_NONE) {
                /* 通信失败的关节也视为"已到位"，避免一直等 */
                continue;
            }
            if ((st & LEESN_STAT_RUN_MASK) == LEESN_STAT_RUN_ACTIVE) {
                all_done = 0;
            }
        }
        if (all_done) {
            LOG_INFO("运动到位");
            return 0;
        }
        Sleep(200);
        elapsed += 200;
    }
    LOG_WARN("等待运动到位超时");
    return -1;
}

/* ================= 公开接口 ================= */

ErrCode robot_home(Robot *robot)
{
    static const double home_pose[ROBOT_JOINT_COUNT] = HOME_POSE_DEG;
    const int group0[] = {1, 2, 3, 5, 6};
    const int group1[] = {4};
    const int all[] = {1, 2, 3, 4, 5, 6};
    int j;
    ErrCode rc = ERR_NONE;

    if (robot == NULL) {
        return ERR_ARG;
    }

    /* 组0：{1,2,3,5,6} 并行堵转回零 */
    LOG_INFO("回零：组0 {1,2,3,5,6} 并行堵转回零...");
    if (wait_group_stall(robot, group0, 5, HOME_GRP_TIMEOUT_MS) != 0) {
        LOG_WARN("组0 堵转回零超时");
        rc = ERR_TIMEOUT;
    }
    LOG_INFO("组0 堵转回零完成");

    /* 组1：{4} 堵转回零 */
    LOG_INFO("回零：组1 {4} 堵转回零...");
    if (wait_group_stall(robot, group1, 1, HOME_GRP_TIMEOUT_MS) != 0) {
        LOG_WARN("组1 堵转回零超时");
        rc = ERR_TIMEOUT;
    }
    LOG_INFO("组1 堵转回零完成");

    /* 运动到机械原点位姿（电机保持使能状态，防止松力后重力漂移） */
    LOG_INFO("回零完成，运动到机械原点位姿...");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        if (robot_is_masked(robot, j)) continue;
        if (robot_movej(robot, j, home_pose[j - 1], (double)HOME_SPEED_RPM) != ERR_NONE) {
            rc = ERR_TIMEOUT;
        }
    }
    wait_group_idle(robot, all, 6);

    /* 运动完成后恢复正常工作状态（使能+限位） */
    LOG_INFO("运动完成，恢复电机正常模式...");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        if (robot_is_masked(robot, j)) continue;
        home_restore_motor(robot, j);
        Sleep(20);
    }

    LOG_INFO("回零流程结束");
    return rc;
}
