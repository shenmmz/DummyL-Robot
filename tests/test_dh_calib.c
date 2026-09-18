/*
 * test_dh_calib —— 零点标定层的闭环与守恒性（离线）
 *
 * 【为什么单独测它】零点标定是"上位机下发的电机角"与"用户看到的机械角"之间
 * 唯一的转换层。它要是错了，表现是"回零后角度看着对、但画出来的线整体偏移"，
 * 极难从现象定位。而它是纯算术，完全可以在离线测死。
 *
 * 测什么：
 *   1) 双向闭环：mech → motor → mech 必须原样回来
 *   2) 与 joint_zero_get() 一致：转换量必须正好等于当前标定值
 *   3) reset/save 语义：save 后 get 变，reset 后回到编译期默认
 *   4) 单调性：机械角整体 +10°，电机角也必须整体 +10°（只是平移，不改比例）
 *
 * 运行：cmake --build build --target test_dh_calib && build/bin/test_dh_calib.exe
 */

#include <stdio.h>
#include <math.h>
#include "kinematics/joint_zero.h"

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-44s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

int main(void)
{
    double mech[6]  = { 12.5, -30.0,  95.0,  45.0, -60.0, 33.0 };
    double motor[6], back[6], tmp[6];
    const double *z;
    char buf[128];
    int i, ok;

    printf("=== 1. 双向闭环 mech → motor → mech ===\n");
    joint_zero_mech_to_motor(mech, motor);
    joint_zero_motor_to_mech(motor, back);
    ok = 1;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        if (fabs(back[i] - mech[i]) > 1e-9) ok = 0;
    }
    snprintf(buf, sizeof buf, "最大偏差 %.3e°", fabs(back[0] - mech[0]));
    check("往返还原", ok, buf);

    printf("\n=== 2. 转换量 == 当前标定值 joint_zero_get() ===\n");
    z = joint_zero_get();
    ok = 1;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        double expect = mech[i] + z[i];     /* 通用口径：电机角 = 机械角 + q0 */
        if (fabs(motor[i] - expect) > 1e-9) ok = 0;
    }
    {
        char t[160];
        snprintf(t, sizeof t, "q0 = {%.2f, %.2f, %.2f, %.2f, %.2f, %.2f}",
                 z[0], z[1], z[2], z[3], z[4], z[5]);
        check("motor = mech + q0", ok, t);
    }

    printf("\n=== 3. 平移不改比例（整体 +10° 仍差 10°）===\n");
    for (i = 0; i < JOINT_ZERO_COUNT; i++) tmp[i] = mech[i] + 10.0;
    joint_zero_mech_to_motor(tmp, back);
    ok = 1;
    for (i = 0; i < JOINT_ZERO_COUNT; i++) {
        if (fabs((back[i] - motor[i]) - 10.0) > 1e-9) ok = 0;
    }
    check("机械角 +10° ⇒ 电机角 +10°", ok, NULL);

    printf("\n=== 4. save / reset 语义 ===\n");
    {
        double def[6], mine[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
        const double *after;
        int i2, same;

        joint_zero_reset();
        after = joint_zero_get();
        for (i2 = 0; i2 < JOINT_ZERO_COUNT; i2++) def[i2] = after[i2];

        joint_zero_save(mine);
        after = joint_zero_get();
        same = 1;
        for (i2 = 0; i2 < JOINT_ZERO_COUNT; i2++)
            if (fabs(after[i2] - mine[i2]) > 1e-9) same = 0;
        check("save 后 get() 返回新值", same, NULL);

        joint_zero_reset();
        after = joint_zero_get();
        same = 1;
        for (i2 = 0; i2 < JOINT_ZERO_COUNT; i2++)
            if (fabs(after[i2] - def[i2]) > 1e-9) same = 0;
        check("reset 后回到编译期默认", same, NULL);
    }

    printf("\n%s（失败项 %d）\n", g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
