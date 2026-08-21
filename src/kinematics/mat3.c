/*
 * mat3.c —— 3x3 旋转矩阵与三维向量基础运算
 * ------------------------------------------------------------
 * 所属模块：运动学基础库（kinematics）
 * 对外接口：mat3_identity、mat3_rotx、mat3_roty、mat3_rotz、
 *           mat3_mul、mat3_transpose、mat3_apply
 * 依赖模块：无
 */

#include "kinematics/mat3.h"

#include <math.h>

/* mat3_identity：将矩阵初始化为单位阵 */
void mat3_identity(Mat3 m)
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            m[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

/* mat3_rotx：生成绕 X 轴旋转 ang 弧度的旋转矩阵 */
void mat3_rotx(double ang, Mat3 m)
{
    double c = cos(ang), s = sin(ang);
    mat3_identity(m);
    m[1][1] = c;  m[1][2] = -s;
    m[2][1] = s;  m[2][2] = c;
}

/* mat3_roty：生成绕 Y 轴旋转 ang 弧度的旋转矩阵 */
void mat3_roty(double ang, Mat3 m)
{
    double c = cos(ang), s = sin(ang);
    mat3_identity(m);
    m[0][0] = c;  m[0][2] = s;
    m[2][0] = -s; m[2][2] = c;
}

/* mat3_rotz：生成绕 Z 轴旋转 ang 弧度的旋转矩阵 */
void mat3_rotz(double ang, Mat3 m)
{
    double c = cos(ang), s = sin(ang);
    mat3_identity(m);
    m[0][0] = c;  m[0][1] = -s;
    m[1][0] = s;  m[1][1] = c;
}

/* mat3_mul：3x3 矩阵乘法，out = a * b */
void mat3_mul(const Mat3 a, const Mat3 b, Mat3 out)
{
    int i, j, k;
    Mat3 r;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            r[i][j] = 0.0;
            for (k = 0; k < 3; k++) {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[i][j] = r[i][j];
        }
    }
}

/* mat3_transpose：3x3 矩阵转置，out = a^T */
void mat3_transpose(const Mat3 a, Mat3 out)
{
    int i, j;
    Mat3 r;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            r[i][j] = a[j][i];
        }
    }
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[i][j] = r[i][j];
        }
    }
}

/* mat3_apply：矩阵作用于向量，out = m * v */
void mat3_apply(const Mat3 m, const Vec3 v, Vec3 out)
{
    int i, j;
    Vec3 r;
    for (i = 0; i < 3; i++) {
        r[i] = 0.0;
        for (j = 0; j < 3; j++) {
            r[i] += m[i][j] * v[j];
        }
    }
    for (i = 0; i < 3; i++) {
        out[i] = r[i];
    }
}
