/*
 * test_dh_fk.c —— 正运动学（FK）离线单元测试
 * ------------------------------------------------------------
 * 所属：tests（CTest）
 * 校验点：
 *   1) 位姿矩阵合法性：末行 [0 0 0 1]、旋转块正交（R·R^T = I）、det(R) = +1、数值有限；
 *   2) 结构对称性：仅 J1 转动时末端 Z 不变、XY 半径不变，且 XY 按 +90° 旋转；
 *   3) 零位可达性：全轴 0（home）末端位置有限且落在结构包络内。
 * 纯算法测试，不依赖串口 / 总线 / 硬件。
 */

#include "kinematics/dh.h"
#include "kinematics/mat3.h"

#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

static int g_fail = 0;

#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            printf("  [FAIL] %s:%d ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                           \
            printf("\n");                                  \
            g_fail++;                                      \
        }                                                  \
    } while (0)

/* 位姿矩阵合法性检查：末行、正交性、行列式、有限性 */
static void check_pose_valid(const char *tag, const double pose[4][4])
{
    Mat3 r, rt, prod;
    double det;
    int i, j;

    CHECK(fabs(pose[3][0]) < 1e-9 && fabs(pose[3][1]) < 1e-9 &&
              fabs(pose[3][2]) < 1e-9 && fabs(pose[3][3] - 1.0) < 1e-9,
          "%s: 末行应为 [0 0 0 1]", tag);

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            r[i][j] = pose[i][j];
            CHECK(isfinite(pose[i][j]), "%s: pose[%d][%d] 非有限值", tag, i, j);
        }
        CHECK(isfinite(pose[i][3]), "%s: 平移分量 [%d][3] 非有限值", tag, i);
    }

    mat3_transpose(r, rt);
    mat3_mul(r, rt, prod);
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            double expect = (i == j) ? 1.0 : 0.0;
            CHECK(fabs(prod[i][j] - expect) < 1e-9,
                  "%s: R·R^T 非单位阵，[%d][%d]=%.12f", tag, i, j, prod[i][j]);
        }
    }

    det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    CHECK(fabs(det - 1.0) < 1e-9, "%s: det(R)=%.12f，应为 +1", tag, det);
}

int main(void)
{
    /* 覆盖零位、单轴大角度、多轴一般姿态 */
    static const double samples[][6] = {
        {   0.0,   0.0,   0.0,   0.0,   0.0,    0.0 },
        {  90.0,   0.0,   0.0,   0.0,   0.0,    0.0 },
        {  45.0,  30.0, -20.0,  10.0,  25.0,  -15.0 },
        { -60.0,  75.0,  40.0, -35.0,  60.0,  120.0 },
        { 180.0, -90.0, -90.0,   0.0, -90.0,    0.0 },
    };
    const int sample_cnt = (int)(sizeof(samples) / sizeof(samples[0]));
    double pose[4][4];
    double p0[4][4], p1[4][4];
    double xyz0[3], rpy0[3], xyz1[3], rpy1[3];
    double j0[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double j1[6] = { 90.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    char tag[16];
    int k;

    printf("test_dh_fk: FK 位姿合法性与结构对称性\n");

    /* 1) 位姿矩阵合法性 */
    for (k = 0; k < sample_cnt; k++) {
        snprintf(tag, sizeof(tag), "sample%d", k + 1);
        dh_forward(DH_TABLE, samples[k], pose);
        check_pose_valid(tag, pose);
    }

    /* 2) J1 绕基座 z 轴旋转：末端 Z 与 XY 半径不变，XY 按 +90° 旋转 */
    dh_forward(DH_TABLE, j0, p0);
    dh_forward(DH_TABLE, j1, p1);
    dh_pose_to_xyz_rpy(p0, xyz0, rpy0);
    dh_pose_to_xyz_rpy(p1, xyz1, rpy1);

    CHECK(fabs(xyz0[2] - xyz1[2]) < 1e-9,
          "J1 转动不应改变末端 Z：%.6f vs %.6f", xyz0[2], xyz1[2]);
    CHECK(fabs(hypot(xyz0[0], xyz0[1]) - hypot(xyz1[0], xyz1[1])) < 1e-9,
          "J1 转动不应改变 XY 半径：%.6f vs %.6f",
          hypot(xyz0[0], xyz0[1]), hypot(xyz1[0], xyz1[1]));
    CHECK(fabs(xyz1[0] + xyz0[1]) < 1e-9 && fabs(xyz1[1] - xyz0[0]) < 1e-9,
          "J1=+90° 时末端 XY 应为 (x,y)->(-y,x)：得到 (%.6f, %.6f)", xyz1[0], xyz1[1]);

    /* 3) 零位（home）末端位置应在结构包络内（基座高度 140，全伸展约 444mm） */
    CHECK(fabs(xyz0[0]) < 500.0 && fabs(xyz0[1]) < 500.0 &&
              xyz0[2] > 0.0 && xyz0[2] < 1000.0,
          "零位末端超出预期包络：X=%.3f Y=%.3f Z=%.3f", xyz0[0], xyz0[1], xyz0[2]);

    printf("  零位(home)末端: X=%.3f Y=%.3f Z=%.3f mm，RPY=%.2f %.2f %.2f deg\n",
           xyz0[0], xyz0[1], xyz0[2],
           rpy0[0] * 180.0 / PI, rpy0[1] * 180.0 / PI, rpy0[2] * 180.0 / PI);
    printf("  仅 J1=+90°:  X=%.3f Y=%.3f Z=%.3f mm\n", xyz1[0], xyz1[1], xyz1[2]);

    if (g_fail == 0) {
        printf("test_dh_fk: PASS\n");
        return 0;
    }
    printf("test_dh_fk: FAIL（%d 处断言未通过）\n", g_fail);
    return 1;
}
