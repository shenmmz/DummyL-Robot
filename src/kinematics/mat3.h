#ifndef MAT3_H
#define MAT3_H


#include <stddef.h>

typedef double Mat3[3][3];
typedef double Vec3[3];

/* m = 单位阵。 */
void mat3_identity(Mat3 m);

/* m = 绕 X 轴旋转 ang（弧度）。 */
void mat3_rotx(double ang, Mat3 m);
/* m = 绕 Y 轴旋转 ang（弧度）。 */
void mat3_roty(double ang, Mat3 m);
/* m = 绕 Z 轴旋转 ang（弧度）。 */
void mat3_rotz(double ang, Mat3 m);

/* out = a * b。允许 out 与 a/b 同一块内存。 */
void mat3_mul(const Mat3 a, const Mat3 b, Mat3 out);

/* out = a 的转置。旋转矩阵正交，转置即求逆。 */
void mat3_transpose(const Mat3 a, Mat3 out);

/* out = m * v。 */
void mat3_apply(const Mat3 m, const Vec3 v, Vec3 out);

#endif
