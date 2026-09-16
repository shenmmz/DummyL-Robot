/*
 * movl_stream_check.c —— MoveL step / stream 两种下发模式的离线对比（注入假总线，不接硬件）
 * ------------------------------------------------------------
 * 真机不在手时验证 cmd_movel 的 stream 分支：
 *   ① 段间确实不等待到位（位置寄存器 0x0004 读事务显著更少）
 *   ② 总事务数显著更少（无逐段轮询、加减速只写一次）
 *   ③ 末段目标脉冲与 step 逐轴一致（不改变轨迹终点）
 *   ④ 末位姿 FK 等于指令位姿（IK/FK 自洽）
 *   ⑤ 直线度代价：用【真实下发的 move_abs 目标序列】反算末端轨迹，
 *      量出到起终点连线的最大垂直偏差 —— 步长放大的代价在此暴露
 *
 * 假从站带极简电机模型：每次被读位置时朝当前目标爬 SIM_RATE 步，
 * 从而可区分"等到位"与"不等到位"；并记录每次 move_abs 下发的六轴目标。
 *
 * 编译：gcc tests/movl_stream_check.c -Isrc -Lbuild/lib \
 *            -lcli -lcontrol -ltrajectory -lkinematics -lapi -lcomm -lutils -lm \
 *            -o tests/movl_stream_check.exe
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "cli/commands.h"
#include "control/robot.h"
#include "comm/comm_if.h"
#include "comm/modbus_rtu.h"
#include "config/robot_config.h"
#include "kinematics/dh.h"
#include "kinematics/joint_zero.h"

#define RAD2DEG (180.0 / 3.14159265358979323846)
#define SIM_RATE 500          /* 每次被读位置时朝目标爬的脉冲数 */
#define LOG_MAX  4096

static int32_t g_pos[7];
static int32_t g_target[7];
static int32_t g_last_tgt[7];
static int32_t g_cmd_log[LOG_MAX][7];
static int     g_cmd_n;
static int     g_txn;
static int     g_pos_reads;

static uint16_t crc16(const uint8_t *d, int n)
{
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
    }
    return c;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFF); }

static void push_u32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)(v & 0xFFFF));
    put16(p + 2, (uint16_t)(v >> 16));
}

static int build_resp(const uint8_t *req, int len, uint8_t *resp)
{
    uint8_t slave = req[0], func = req[1];
    uint16_t addr = (uint16_t)((req[2] << 8) | req[3]);
    uint16_t cnt  = (uint16_t)((req[4] << 8) | req[5]);
    int n = 0;

    (void)len;
    if (slave < 1 || slave > 6) return 0;

    if (func == 0x03) {
        if (addr == 0x0004) g_pos_reads++;
        if (addr == 0x0004 || addr == 0x0006) {
            int32_t d = g_target[slave] - g_pos[slave];
            if (d > SIM_RATE) d = SIM_RATE;
            if (d < -SIM_RATE) d = -SIM_RATE;
            g_pos[slave] += d;
        }
        resp[0] = slave; resp[1] = func;
        if (addr == 0x0004 && cnt == 4) {
            resp[2] = 8;
            push_u32(resp + 3, (uint32_t)g_pos[slave]);
            push_u32(resp + 7, 0u);
            n = 11;
        } else {
            resp[2] = (uint8_t)(cnt * 2);
            for (uint16_t i = 0; i < cnt; i++) {
                uint16_t v = 0;
                uint16_t a = (uint16_t)(addr + i);
                if (a == 0x0004)      v = (uint16_t)((uint32_t)g_pos[slave] & 0xFFFF);
                else if (a == 0x0005) v = (uint16_t)(((uint32_t)g_pos[slave] >> 16) & 0xFFFF);
                else if (a == 0x001A) v = 200;
                else if (a == 0x0024) v = (uint16_t)(ENCODER_STEPS_PER_REV & 0xFFFF);
                else if (a == 0x0025) v = (uint16_t)((ENCODER_STEPS_PER_REV >> 16) & 0xFFFF);
                else if (a == 0x0066) v = slave;
                else if (a == 0x00A3) v = 0;
                put16(resp + 3 + i * 2, v);
            }
            n = 3 + cnt * 2;
        }
    } else if (func == 0x06) {
        memcpy(resp, req, 8);
        n = 6;
    } else if (func == 0x10) {
        if (addr == 0x00E8) {
            int32_t v = (int32_t)(((uint32_t)((req[7] << 8) | req[8])) |
                                  ((uint32_t)((req[9] << 8) | req[10]) << 16));
            g_target[slave] = v;
            g_last_tgt[slave] = v;
            if (slave == 6 && g_cmd_n < LOG_MAX) {   /* 6 号轴写完 = 一个节拍收齐 */
                for (int j = 1; j <= 6; j++) g_cmd_log[g_cmd_n][j] = g_target[j];
                g_cmd_n++;
            }
        }
        memcpy(resp, req, 6);
        n = 6;
    } else {
        return 0;
    }
    {
        uint16_t c = crc16(resp, n);
        resp[n] = (uint8_t)(c & 0xFF);
        resp[n + 1] = (uint8_t)(c >> 8);
    }
    return n + 2;
}

static uint8_t g_resp[300];
static int g_resp_len;

static int  fake_open(const char *p, uint32_t b) { (void)p; (void)b; return 0; }
static void fake_close(void) {}
static void fake_flush(void) {}
static int  fake_write(const uint8_t *buf, int len)
{
    g_txn++;
    g_resp_len = build_resp(buf, len, g_resp);
    return len;
}
static int  fake_read(uint8_t *buf, int cap, int to)
{
    (void)to;
    int n = g_resp_len < cap ? g_resp_len : cap;
    memcpy(buf, g_resp, (size_t)n);
    return n;
}
static const CommOps g_fake_ops = { fake_open, fake_close, fake_read, fake_write, fake_flush };

/* 用真实下发序列反算末端轨迹到起终点连线的最大垂直偏差。
 * 关键点：段内是关节空间插补（各轴按距离比例同步 ≈ 关节线性），末端并不走直线，
 * 故必须在相邻两个下发点之间【关节空间再采样】才能量出段内弓高。
 * 路径起点取 q_start（回零姿态），其后依次是每次下发的六轴目标。 */
static double cmd_path_deviation(const double p0[3], const double p1[3],
                                 const double q_start[6])
{
    const uint16_t red[6] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
    double L = sqrt(ux * ux + uy * uy + uz * uz);
    double prev[6], next[6];
    double maxd = 0.0;
    const int K = 20;

    if (L < 1e-9) return 0.0;
    ux /= L; uy /= L; uz /= L;

    for (int j = 0; j < 6; j++) prev[j] = q_start[j];

    for (int i = 0; i < g_cmd_n; i++) {
        for (int j = 1; j <= 6; j++)
            next[j - 1] = STEPS2DEG(g_cmd_log[i][j], red[j - 1]) - zero[j - 1];
        for (int k = 0; k <= K; k++) {
            double t = (double)k / (double)K;
            double q[6], m[4][4], xyz[3], rpy[3], vx, vy, vz, proj, d;
            for (int j = 0; j < 6; j++) q[j] = prev[j] + (next[j] - prev[j]) * t;
            dh_forward(DH_TABLE, q, m);
            dh_pose_to_xyz_rpy(m, xyz, rpy);
            vx = xyz[0] - p0[0]; vy = xyz[1] - p0[1]; vz = xyz[2] - p0[2];
            proj = vx * ux + vy * uy + vz * uz;
            vx -= proj * ux; vy -= proj * uy; vz -= proj * uz;
            d = sqrt(vx * vx + vy * vy + vz * vz);
            if (d > maxd) maxd = d;
        }
        for (int j = 0; j < 6; j++) prev[j] = next[j];
    }
    return maxd;
}

static int run_case(int stream, double speed_rpm, const double end_pose[6],
                    const double start_xyz[3],
                    int32_t final_tgt[7], int *txn, int *pos_reads,
                    double end_fk[3], double *dev, int *cmd_n)
{
    const uint16_t red[6] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    double q_home[6] = {0, 0, 90, 0, 0, 0};
    ParsedCmd cmd;
    Robot *robot;
    int j;

    for (j = 1; j <= 6; j++) {
        g_pos[j] = DEG2STEPS(q_home[j - 1] + zero[j - 1], red[j - 1]);
        g_target[j] = g_pos[j];
        g_last_tgt[j] = g_pos[j];
    }
    g_txn = 0; g_pos_reads = 0; g_cmd_n = 0;

    modbus_comm_set(&g_fake_ops);
    robot = robot_init("FAKE", 115200);
    if (robot == NULL) { printf("[FAIL] robot_init 失败\n"); return -1; }

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = CMD_MOVEL;
    for (j = 0; j < 6; j++) cmd.cartesian[j] = end_pose[j];
    cmd.speeds[0] = speed_rpm;
    cmd.accel_ms[0] = 80;
    cmd.decel_ms[0] = 90;
    cmd.stream = stream;

    cmd_movel(robot, &cmd);

    for (j = 1; j <= 6; j++) final_tgt[j] = g_last_tgt[j];
    {
        double mech[6], m[4][4], xyz[3], rpy[3];
        for (j = 1; j <= 6; j++)
            mech[j - 1] = STEPS2DEG(final_tgt[j], red[j - 1]) - zero[j - 1];
        dh_forward(DH_TABLE, mech, m);
        dh_pose_to_xyz_rpy(m, xyz, rpy);
        end_fk[0] = xyz[0]; end_fk[1] = xyz[1]; end_fk[2] = xyz[2];
    }
    *dev = cmd_path_deviation(start_xyz, end_pose, q_home);
    *cmd_n = g_cmd_n;
    *txn = g_txn;
    *pos_reads = g_pos_reads;
    robot_close(robot);
    return 0;
}

static void scenario(const char *name, double drop_mm, double speed_rpm)
{
    double q_home[6] = {0, 0, 90, 0, 0, 0};
    double m[4][4], xyz[3], rpy[3], end_pose[6];
    int32_t ts[7] = {0}, tv[7] = {0};
    int xs = 0, xv = 0, ps = 0, pv = 0, ns = 0, nv = 0;
    double fks[3] = {0}, fkv[3] = {0}, ds = 0, dv = 0;
    int fail = 0, j;

    dh_forward(DH_TABLE, q_home, m);
    dh_pose_to_xyz_rpy(m, xyz, rpy);
    end_pose[0] = xyz[0];
    end_pose[1] = xyz[1];
    end_pose[2] = xyz[2] - drop_mm;
    end_pose[3] = rpy[0] * RAD2DEG;
    end_pose[4] = rpy[1] * RAD2DEG;
    end_pose[5] = rpy[2] * RAD2DEG;

    printf("\n========== %s：Z 下移 %.0f mm @ %.0f rpm ==========\n", name, drop_mm, speed_rpm);
    printf("起点 (%.2f, %.2f, %.2f) → 终点 (%.2f, %.2f, %.2f)\n",
           xyz[0], xyz[1], xyz[2], end_pose[0], end_pose[1], end_pose[2]);

    run_case(0, speed_rpm, end_pose, xyz, ts, &xs, &ps, fks, &ds, &ns);
    run_case(1, speed_rpm, end_pose, xyz, tv, &xv, &pv, fkv, &dv, &nv);

    printf("  step  : 总事务=%-5d 位置读=%-5d 下发段数=%-4d 末端偏差=%.4f mm 直线度=%.4f mm\n",
           xs, ps, ns, fabs(fks[2] - end_pose[2]), ds);
    printf("  stream: 总事务=%-5d 位置读=%-5d 下发段数=%-4d 末端偏差=%.4f mm 直线度=%.4f mm\n",
           xv, pv, nv, fabs(fkv[2] - end_pose[2]), dv);

    if (pv >= ps) { printf("  ① [FAIL] stream 位置读未减少\n"); fail = 1; }
    else printf("  ① stream 段间不等到位 PASS (%d < %d)\n", pv, ps);

    if (xv >= xs) { printf("  ② [FAIL] stream 总事务未减少\n"); fail = 1; }
    else printf("  ② stream 总事务更少 PASS (%d < %d)\n", xv, xs);

    {
        int same = 1;
        for (j = 1; j <= 6; j++) if (ts[j] != tv[j]) same = 0;
        if (!same) { printf("  ③ [FAIL] 末段目标不一致\n"); fail = 1; }
        else printf("  ③ 末段目标逐轴一致 PASS\n");
    }
    {
        double e = fmax(fabs(fks[0] - end_pose[0]), fabs(fks[1] - end_pose[1]));
        e = fmax(e, fabs(fks[2] - end_pose[2]));
        if (e >= 0.1) { printf("  ④ [FAIL] 末位姿偏差 %.4f mm\n", e); fail = 1; }
        else printf("  ④ 末位姿 FK 与指令一致 PASS (%.4f mm)\n", e);
    }
    printf("  ⑤ 直线度：step %.4f mm vs stream %.4f mm（stream 步长放大后的代价）\n", ds, dv);
    printf("  => %s\n", fail ? "存在 FAIL" : "PASS");
}

int main(void)
{
    printf("MoveL step / stream 下发模式离线对比（假总线，不接硬件）\n");
    scenario("短程", 8.0, 60.0);
    scenario("长程", 120.0, 60.0);
    scenario("长程慢速", 120.0, 10.0);
    return 0;
}
