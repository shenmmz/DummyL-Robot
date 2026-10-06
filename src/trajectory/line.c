
#include "trajectory/line.h"
#include "kinematics/ik.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define LINE_PI     3.14159265358979323846
#define LINE_DEG2RAD (LINE_PI / 180.0)
#define LINE_EPS    1e-12


/* 四元数相乘 out = a*b。 */
static void quat_mul(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* RPY（度）→ 四元数。 */
static void euler_to_quat(double roll_deg, double pitch_deg, double yaw_deg, double q[4])
{
    double roll = roll_deg * LINE_DEG2RAD;
    double pitch = pitch_deg * LINE_DEG2RAD;
    double yaw = yaw_deg * LINE_DEG2RAD;
    double cr = cos(roll/2), sr = sin(roll/2);
    double cp = cos(pitch/2), sp = sin(pitch/2);
    double cy = cos(yaw/2), sy = sin(yaw/2);

    double q_roll[4] = {cr, sr, 0, 0};
    double q_pitch[4] = {cp, 0, sp, 0};
    double q_yaw[4]  = {cy, 0, 0, sy};
    double tmp[4];
    quat_mul(q_pitch, q_roll, tmp);
    quat_mul(q_yaw, tmp, q);
}

/* 把 x 夹进 [-1, 1]，防 acos/asin 定义域越界。 */
static double clamp_pm1(double x)
{
    if (x > 1.0) return 1.0;
    if (x < -1.0) return -1.0;
    return x;
}

/* 四元数 → RPY（度）。 */
static void quat_to_euler(const double q[4], double *roll_deg, double *pitch_deg, double *yaw_deg)
{
    double roll = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
    double pitch = asin(clamp_pm1(2*(q[0]*q[2] - q[1]*q[3])));
    double yaw = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
    *roll_deg = roll / LINE_DEG2RAD;
    *pitch_deg = pitch / LINE_DEG2RAD;
    *yaw_deg = yaw / LINE_DEG2RAD;
}

/* 四元数球面线性插值：t=0 得 q1，t=1 得 q2。 */
static void slerp(const double q1[4], const double q2[4], double t, double out[4])
{
    double qq2[4] = {q2[0], q2[1], q2[2], q2[3]};
    double cosom = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    if (cosom < 0) {
        cosom = -cosom;
        qq2[0] = -qq2[0]; qq2[1] = -qq2[1]; qq2[2] = -qq2[2]; qq2[3] = -qq2[3];
    }
    if (cosom > 0.9995) {
        out[0] = q1[0] + t*(qq2[0]-q1[0]);
        out[1] = q1[1] + t*(qq2[1]-q1[1]);
        out[2] = q1[2] + t*(qq2[2]-q1[2]);
        out[3] = q1[3] + t*(qq2[3]-q1[3]);
        double len = sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3]);
        out[0]/=len; out[1]/=len; out[2]/=len; out[3]/=len;
        return;
    }
    double omega = acos(cosom);
    double sinom = sin(omega);
    double s1 = sin((1-t)*omega) / sinom;
    double s2 = sin(t*omega) / sinom;
    out[0] = s1*q1[0] + s2*qq2[0];
    out[1] = s1*q1[1] + s2*qq2[1];
    out[2] = s1*q1[2] + s2*qq2[2];
    out[3] = s1*q1[3] + s2*qq2[3];
}


/* 位姿 [x,y,z,rx,ry,rz]（mm/度）→ 4x4 齐次矩阵。 */
void line_pose_to_matrix(const double pose6[6], double m[4][4])
{
    double rx = pose6[3] * LINE_DEG2RAD;
    double ry = pose6[4] * LINE_DEG2RAD;
    double rz = pose6[5] * LINE_DEG2RAD;
    double crx = cos(rx), srx = sin(rx);
    double cry = cos(ry), sry = sin(ry);
    double crz = cos(rz), srz = sin(rz);

    m[0][0] = crz * cry;
    m[0][1] = crz * sry * srx - srz * crx;
    m[0][2] = crz * sry * crx + srz * srx;
    m[1][0] = srz * cry;
    m[1][1] = srz * sry * srx + crz * crx;
    m[1][2] = srz * sry * crx - crz * srx;
    m[2][0] = -sry;
    m[2][1] = cry * srx;
    m[2][2] = cry * crx;
    m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0;
    m[0][3] = pose6[0];
    m[1][3] = pose6[1];
    m[2][3] = pose6[2];
    m[3][3] = 1.0;
}

/* 按步长把总距离切成若干段，返回插补点数（含首尾，段数 = 点数-1，至少 2 个点）。 */
int line_count_for_distance(double dist_mm, double step_mm)
{
    int count;

    if (step_mm <= LINE_EPS || dist_mm <= LINE_EPS) {
        return 2;
    }
    count = (int)(dist_mm / step_mm);
    if ((double)count * step_mm < dist_mm - 1e-9) {
        count++;
    }
    count += 1;
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;
    return count;
}

/* 笛卡尔直线插补：位置线性插值 + 姿态四元数 SLERP，产出 count 个位姿点。 */
int line_plan(const double start_pose[6], const double end_pose[6],
               int count, LinePath *path)
{
    double q_start[4], q_end[4], q_interp[4];
    double pose_interp[6];
    double t;
    int i, j;

    if (start_pose == NULL || end_pose == NULL || path == NULL) {
        return -1;
    }
    if (count < 2) count = 2;
    if (count > LINE_MAX_POINTS) count = LINE_MAX_POINTS;

    euler_to_quat(start_pose[3], start_pose[4], start_pose[5], q_start);
    euler_to_quat(end_pose[3], end_pose[4], end_pose[5], q_end);

    path->count = count;
    for (i = 0; i < count; i++) {
        t = (double)i / (double)(count - 1);
        for (j = 0; j < 3; j++) {
            pose_interp[j] = start_pose[j] + (end_pose[j] - start_pose[j]) * t;
        }
        slerp(q_start, q_end, t, q_interp);
        quat_to_euler(q_interp, &pose_interp[3], &pose_interp[4], &pose_interp[5]);
        for (j = 0; j < 6; j++) {
            if (!isfinite(pose_interp[j])) {
                return -1;
            }
            path->pose[i][j] = pose_interp[j];
        }
    }
    return 0;
}

/* 逐点 IK 解算整条直线：每点以【上一点解】为参考，解算 → 限位过滤 → unwrap 连续选解。
 * 失败返回 -1，并通过 fail_idx / fail_reason 说清是第几个点、哪个部位无解
 * （"肩部/肘部/腕部 + 状态" 或 "候选解全部越软限位"）。 */
int line_solve(const LinePath *path, const DhParam *dh, const JointLimit *limits,
                const double *start_joints, double (*q_out)[6],
                int *fail_idx, char *fail_reason)
{
    double prev[6];
    double sols[IK_MAX_SOLUTIONS][6];
    double filtered[IK_MAX_SOLUTIONS][6];
    IkSolInfo info[IK_MAX_SOLUTIONS];
    int i, j, cnt, n;

    if (path == NULL || dh == NULL || q_out == NULL || path->count < 2) {
        if (fail_reason) snprintf(fail_reason, 64, "参数非法");
        return -1;
    }

    for (j = 0; j < 6; j++) {
        prev[j] = (start_joints != NULL) ? start_joints[j] : 0.0;
    }

    for (i = 0; i < path->count; i++) {
        double m[4][4];

        line_pose_to_matrix(path->pose[i], m);
        cnt = ik_solve_ref(dh, m, prev, sols, info);
        if (cnt <= 0) {
            int s, found = 0;
            for (s = 0; s < IK_MAX_SOLUTIONS && !found; s++) {
                if (info[s].shoulder != IK_SOL_VALID) {
                    if (fail_reason) snprintf(fail_reason, 64,
                             "IK 无解：肩部%s（点%d）",
                             ik_sol_status_str(info[s].shoulder), i);
                    found = 1;
                } else if (info[s].elbow != IK_SOL_VALID) {
                    if (fail_reason) snprintf(fail_reason, 64,
                             "IK 无解：肘部%s（点%d）",
                             ik_sol_status_str(info[s].elbow), i);
                    found = 1;
                } else if (info[s].wrist != IK_SOL_VALID) {
                    if (fail_reason) snprintf(fail_reason, 64,
                             "IK 无解：腕部%s（点%d）",
                             ik_sol_status_str(info[s].wrist), i);
                    found = 1;
                }
            }
            if (!found && fail_reason) {
                snprintf(fail_reason, 64, "IK 无解（点%d，原因不明）", i);
            }
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        /* 限位判定必须在【物理角】（IK 输出的 (-180,180]）上做，不能放在 unwrap 之后：
         * ik_unwrap_solutions 为求连续会把解平移到贴近 prev 的 ±360k 分支，在 ±180°
         * 软限位边缘会把本来可达的点平移成 190°/-190° 之类而被误杀（旧实现先 unwrap
         * 再过滤，报"候选解全部越软限位"是假的不可达）。连续性交给下面的
         * ik_select_best_continuous —— 它内部对已过滤的物理解再 unwrap 选最近支。 */
        n = ik_filter_by_limits(sols, cnt, limits, filtered);
        if (n <= 0) {
            if (fail_reason)
                snprintf(fail_reason, 64, "候选解全部越软限位（点%d，IK 产出%d组）", i, cnt);
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        if (ik_select_best_continuous(filtered, n, prev, NULL, q_out[i]) != 0) {
            if (fail_reason)
                snprintf(fail_reason, 64, "分支连续选解失败（点%d）", i);
            if (fail_idx) *fail_idx = i;
            return -1;
        }
        for (j = 0; j < 6; j++) {
            prev[j] = q_out[i][j];
        }
    }
    return 0;
}

/* 由各关节最大速度反算每段耗时。
 * ★ 每段各自取【该段】最慢轴所需时间 ⇒ 段内各轴同时到达（同步），段间按各自
 *   行程分配时长：小的段快走完，不被全路径最慢那一段拖着整体爬。
 *   （旧实现把 dt 取成全路径统一最大值，只要有一段擦过奇异大跳，其余所有段都被
 *     拉到同样慢，dt_total 被放大成 worst_dt × 段数。）
 * dt_total = Σ seg_dt。返回 -1 表示参数非法或某轴 vmax<=0。 */
int line_time_table(const double (*q_seq)[6], int count, const double vmax_joint[6],
                     double *seg_dt, double *dt_total)
{
    double total = 0.0;
    int i, j;

    if (q_seq == NULL || seg_dt == NULL || count < 2) {
        return -1;
    }
    for (j = 0; j < 6; j++) {
        if (vmax_joint == NULL || vmax_joint[j] <= 0.0) {
            return -1;
        }
    }

    for (i = 0; i < count - 1; i++) {
        double dt = 0.0;
        for (j = 0; j < 6; j++) {
            double need = fabs(q_seq[i + 1][j] - q_seq[i][j]) / vmax_joint[j];
            if (need > dt) dt = need;
        }
        seg_dt[i] = dt;
        total += dt;
    }
    if (dt_total != NULL) {
        *dt_total = total;
    }
    return 0;
}

/* 找出相邻插补点之间最大的【单关节跳变】(度)，用于预警某段线会让某轴猛甩。
 * out_joint / out_seg 传出是哪一轴、哪一段。 */
double line_max_joint_jump(const double (*q_seq)[6], int count,
                           int *out_joint, int *out_seg)
{
    double worst = 0.0;
    int worst_j = 0, worst_seg = 0;
    int i, j;

    if (q_seq == NULL || count < 2) return 0.0;

    for (i = 1; i < count; i++) {
        for (j = 0; j < 6; j++) {
            double dj = fabs(q_seq[i][j] - q_seq[i - 1][j]);
            if (dj > worst) { worst = dj; worst_j = j + 1; worst_seg = i; }
        }
    }
    if (out_joint != NULL) *out_joint = worst_j;
    if (out_seg   != NULL) *out_seg   = worst_seg;
    return worst;
}
