#ifndef MAT3_H
#define MAT3_H

/*
 * 3x3 矩阵运算（纯 C，行主序）
 * 用于旋转矩阵构造 / 乘法 / 转置 / 向量变换。
 */

#include <stddef.h>

typedef double Mat3[3][3];
typedef double Vec3[3];

/* 单位矩阵 */
void mat3_identity(Mat3 m);

/* 绕 X/Y/Z 轴旋转（rad） */
void mat3_rotx(double ang, Mat3 m);
void mat3_roty(double ang, Mat3 m);
void mat3_rotz(double ang, Mat3 m);

/* 矩阵乘法: out = a * b（a/b/out 不允许别名） */
void mat3_mul(const Mat3 a, const Mat3 b, Mat3 out);

/* 转置: out = a^T */
void mat3_transpose(const Mat3 a, Mat3 out);

/* 向量变换: out = m * v */
void mat3_apply(const Mat3 m, const Vec3 v, Vec3 out);

#endif /* MAT3_H */
