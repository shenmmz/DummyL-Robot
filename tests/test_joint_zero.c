/*
 * test_joint_zero —— 关节零点标定换算的离线校验
 *
 * 校验两点：
 *   1) 往返一致：mech = motor - q0，motor = mech + q0 必须严格互逆；
 *   2) 标定基准：机械姿态 (0, 0, 90, 0, 0, 0)（大臂竖直、前臂水平）
 *      换算回上位机电机角必须等于回零实测值
 *      (-176.85, 72.87, -85.46, 7.38, 118.08, 262.27)。
 *      注意 J6=262.27：传感器回零清零点(电机角0)不是机械零点，回零后须再正转 +262.27°，
 *      机械角才归 0（home.c home_goto_pose 的 6 轴零点偏置）。
 * 该基准一旦对不上，说明 config/robot_config.h 的 ROBOT_JOINT_ZERO_DEG 被改坏。
 */

#include "kinematics/joint_zero.h"

#include <math.h>
#include <stdio.h>

static int g_fail = 0;

static void check_close(const char *tag, double got, double want, double tol)
{
    if (fabs(got - want) > tol) {
        printf("  [FAIL] %s: got %.6f, want %.6f (tol %g)\n", tag, got, want, tol);
        g_fail = 1;
    }
}

int main(void)
{
    double motor[6], mech[6], back[6];
    /* 回零后实测上位机角（对应机械姿态 0,0,90,0,0,0） */
     const double motor_ref[6] = { -176.85, 72.87, -85.46, 7.38, 118.08, 262.27 };
    const double mech_ref[6]  = { 0.0, 0.0, 90.0, 0.0, 0.0, 0.0 };
    int i;

    printf("test_joint_zero: 零点标定换算\n");

    /* 1) 电机角 -> 机械角 必须等于标定姿态 */
    joint_zero_motor_to_mech(motor_ref, mech);
    for (i = 0; i < 6; i++) {
        check_close("motor_to_mech", mech[i], mech_ref[i], 1e-9);
    }

    /* 2) 机械角 -> 电机角 必须还原实测值 */
    joint_zero_mech_to_motor(mech_ref, motor);
    for (i = 0; i < 6; i++) {
        check_close("mech_to_motor", motor[i], motor_ref[i], 1e-9);
    }

    /* 3) 往返一致：任意角先正后逆应还原 */
    {
        const double any[6] = { -12.5, 33.25, -7.5, 91.0, -180.0, 45.0 };
        joint_zero_motor_to_mech(any, mech);
        joint_zero_mech_to_motor(mech, back);
        for (i = 0; i < 6; i++) {
            check_close("roundtrip", back[i], any[i], 1e-9);
        }
    }

    if (g_fail) {
        printf("test_joint_zero: FAIL\n");
        return 1;
    }
    printf("test_joint_zero: PASS\n");
    return 0;
}
