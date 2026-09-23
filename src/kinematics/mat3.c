
#include "kinematics/mat3.h"

#include <math.h>

/* 3x3 单位阵。 */
void mat3_identity(Mat3 m)
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            m[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

/* 绕 X 轴旋转 ang 弧度。 */
void mat3_rotx(double ang, Mat3 m)
{
    double c = cos(ang), s = sin(ang);
    mat3_identity(m);
    m[1][1] = c;  m[1][2] = -s;
    m[2][1] = s;  m[2][2] = c;
}

/* 绕 Y 轴旋转 ang 弧度。 */
void mat3_roty(double ang, Mat3 m)
{
    double c = cos(ang), s = sin(ang);
    mat3_identity(m);
    m[0][0] = c;  m[0][2] = s;
    m[2][0] = -s; m[2][2] = c;
}

/* 绕 Z 轴旋转 ang 弧度。 */
void mat3_rotz(double ang, Mat3 m)
{
    double c = cos(ang), s = sin(ang);
    mat3_identity(m);
    m[0][0] = c;  m[0][1] = -s;
    m[1][0] = s;  m[1][1] = c;
}

/* 3x3 矩阵相乘 out = a*b。 */
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

/* 3x3 转置。 */
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

/* 3x3 矩阵作用于三维向量 out = m*v。 */
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
