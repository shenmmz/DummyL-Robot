/*
 * test_joint_limits.c —— 关节软限位离线回归测试
 * ------------------------------------------------------------
 * 所属：tests（CTest）
 * 仅链接 kinematics 静态库（无串口/总线依赖，可在无硬件环境运行）。
 * 覆盖：
 *   ① 配置合法性：ROBOT_JOINT_LIMIT_MIN/MAX_DEG 满足 min<max 且范围非空；
 *   ② IK 限位过滤 ik_filter_by_limits 使用真实软限位表时的行为：
 *      - 全轴立正姿态 (0,0,0,0,0,0) 因 J3=0<30 必须被过滤掉（结构不可达）；
 *      - 回零机械姿态 (0,0,90,0,0,0) 必须在限位内保留；
 *      - 单轴越界（J2=100>90）必须被过滤；
 *      - 可整圈回转（J4=360 在 [-360,360]）必须保留。
 * 固定判据，结果可复现。
 */

#include "config/robot_config.h"
#include "kinematics/ik.h"

#include <stdio.h>
#include <string.h>

#define N ROBOT_JOINT_COUNT

static int g_fail = 0;
#define CHECK(cond, msg)                                      \
    do {                                                      \
        if (!(cond)) {                                        \
            printf("  FAIL: %s\n", msg);                      \
            g_fail++;                                         \
        }                                                     \
    } while (0)

int main(void)
{
    const double lim_min[N] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lim_max[N] = ROBOT_JOINT_LIMIT_MAX_DEG;
    JointLimit limits[N];
    double in[IK_MAX_SOLUTIONS][6];
    double out[IK_MAX_SOLUTIONS][6];
    int cnt, j;

    for (j = 0; j < N; j++) {
        limits[j].min_deg = lim_min[j];
        limits[j].max_deg = lim_max[j];
    }

    /* ① 配置合法性 */
    for (j = 0; j < N; j++) {
        CHECK(lim_min[j] < lim_max[j], "限位 min<max");
        CHECK(lim_min[j] > -1000.0 && lim_max[j] < 1000.0, "限位范围合理");
    }

    /* ② 全轴立正 (0,0,0,0,0,0)：J3=0 < 30，应被过滤 */
    memset(in, 0, sizeof(in));
    cnt = ik_filter_by_limits(in, 1, limits, out);
    CHECK(cnt == 0, "立正姿态(全0)应被软限位过滤(J3<30)");

    /* ③ 回零机械姿态 (0,0,90,0,0,0)：在限位内，保留 */
    memset(in, 0, sizeof(in));
    in[0][2] = 90.0;
    cnt = ik_filter_by_limits(in, 1, limits, out);
    CHECK(cnt == 1, "回零机械姿态(0,0,90,0,0,0)应在软限位内保留");

    /* ④ 单轴越界 J2=100 (>90)：过滤 */
    memset(in, 0, sizeof(in));
    in[0][2] = 90.0;
    in[0][1] = 100.0;
    cnt = ik_filter_by_limits(in, 1, limits, out);
    CHECK(cnt == 0, "J2=100° 超出上限90°应被过滤");

    /* ⑤ 整圈回转 J4=360 (在 [-360,360])：保留 */
    memset(in, 0, sizeof(in));
    in[0][2] = 90.0;
    in[0][3] = 360.0;
    cnt = ik_filter_by_limits(in, 1, limits, out);
    CHECK(cnt == 1, "J4=360° 在[-360,360]内应保留");

    if (g_fail == 0) {
        printf("joint_limits: 全部通过 (5 项)\n");
        return 0;
    }
    printf("joint_limits: %d 项失败\n", g_fail);
    return 1;
}
