/*
 * test_wrist_geom —— 球腕几何自洽性 + 关节符号/零点验证方案的量化依据
 * ------------------------------------------------------------
 * 【为什么要有这个测试】
 * 清单 #5（J4/J6 方向符号）与 #6（J4/J5 零点）一直没闭环，因为**光靠软件
 * 查不出来**：getpos 显示的是 FK(编码器读数)，读数 + 模型永远自洽 ——
 * 模型参数错了，读回来的数会跟着一起错，看不出破绽。要发现符号/零点错误，
 * 必须有**独立物理参照**（肉眼、尺子、触碰固定物）。
 *
 * 所以本测试做两件事：
 *   ① 锁住 DH 表的球腕几何性质（这些是 IK 算法成立的前提，改坏了要能立刻发现）
 *   ② 把"真机上该怎么验、验出来长什么样"所需的预测值算出来，
 *      供 docs/关节符号与零点验证.md 使用 —— 让用户做最少的观察就能定论。
 *
 * 【轴序约定（踩过的坑）】标准 DH 里 **关节 i 的转轴是 z_{i-1}**，不是 z_i。
 *   T_{i-1→i} = Rz(θ_i)Tz(d_i)Tx(a_i)Rx(α_i)，其中 Rz(θ_i) 是绕 z_{i-1} 转。
 *   我第一版按 "J4 轴 = z4" 算，得到 z5·z6 = 1.0（"J5、J6 同轴"，不像球腕），
 *   一度以为 DH 表有问题 —— 其实是自己轴序搞错了。
 *
 * 运行：cmake --build build --target test_wrist_geom && build/bin/test_wrist_geom.exe
 */

#include <stdio.h>
#include <math.h>
#include "kinematics/dh_params.h"
#include "kinematics/dh.h"

#define RAD2DEG (180.0 / 3.14159265358979323846)
#define DEG2RAD (3.14159265358979323846 / 180.0)

static int g_fail = 0;

static void check(const char *tag, int ok, const char *detail)
{
    printf("  %-46s %s%s%s\n", tag, ok ? "OK" : "[FAIL]",
           detail ? "  " : "", detail ? detail : "");
    if (!ok) g_fail++;
}

/* acc = T_0_k（前 k 个关节变换之积），k=0 时为单位阵。
 * 【踩过的坑】theta 必须带上 theta_offset（即各轴 home 角），与 dh_forward
 * 的口径保持一致。第一版漏了这一项 ⇒ J2 的 home 偏移 -90° 被丢掉，连杆 a2
 * 指向 +X 而不是 +Z，整条链错位，导致第 ④ 项误报 FAIL（|TCP-腕心|=155.84）。
 * 教训：凡是与 FK 对拍的辅助函数，都要跟 dh_forward 用同一个 theta 口径。 */
static void accum(const double q[6], int k, double out[4][4])
{
    int i;
    for (i = 0; i < 4; i++) {
        int jj;
        for (jj = 0; jj < 4; jj++) out[i][jj] = (i == jj) ? 1.0 : 0.0;
    }
    for (i = 0; i < k; i++) {
        double t[4][4], r[4][4];
        int a, b, m;
        dh_transform(&DH_TABLE[i], q[i] * DEG2RAD + DH_TABLE[i].theta_offset, t);
        for (a = 0; a < 4; a++)
            for (b = 0; b < 4; b++) {
                double s = 0.0;
                for (m = 0; m < 4; m++) s += out[a][m] * t[m][b];
                r[a][b] = s;
            }
        for (a = 0; a < 4; a++)
            for (b = 0; b < 4; b++) out[a][b] = r[a][b];
    }
}

/* 关节 joint(1..6) 的转轴方向（基座系）= z_{joint-1} */
static void joint_axis(const double q[6], int joint, double z[3])
{
    double T[4][4];
    accum(q, joint - 1, T);
    z[0] = T[0][2]; z[1] = T[1][2]; z[2] = T[2][2];
}

/* frame k 的原点（基座系坐标），k=0 为基座原点 */
static void frame_origin(const double q[6], int k, double o[3])
{
    double T[4][4];
    accum(q, k, T);
    o[0] = T[0][3]; o[1] = T[1][3]; o[2] = T[2][3];
}

static void tcp(const double q[6], double p[3])
{
    double T[4][4];
    dh_forward(DH_TABLE, q, T);
    p[0] = T[0][3]; p[1] = T[1][3]; p[2] = T[2][3];
}

/* 法兰坐标系里的点 (lx,ly,lz) 变换到基座系 */
static void flange_point(const double q[6], double lx, double ly, double lz, double w[3])
{
    double T[4][4];
    int i;
    dh_forward(DH_TABLE, q, T);
    for (i = 0; i < 3; i++)
        w[i] = T[i][0] * lx + T[i][1] * ly + T[i][2] * lz + T[i][3];
}

static double dist3(const double a[3], const double b[3])
{
    return sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2) + pow(a[2]-b[2],2));
}

static double dot3(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

int main(void)
{
    double q[6] = {0, 0, 90, 0, 0, 0};
    double z4[3], z5[3], z6[3], wc[3], p[3], p0[3], p1[3];
    char buf[192];

    dh_set_tool_length(0.0);

    printf("=== 1. 球腕基本性质（IK 解析算法成立的前提）===\n");

    /* ⓪ 先对拍：本文件的 accum() 必须与 dh_forward 完全同口径。
     *   第一版就是漏了 theta_offset 在这里翻的车，加这条能立刻抓住同类错误。 */
    {
        double T[4][4], A[4][4], worst = 0.0;
        int i, jj;
        dh_forward(DH_TABLE, q, T);
        accum(q, 6, A);
        for (i = 0; i < 4; i++)
            for (jj = 0; jj < 4; jj++)
                if (fabs(T[i][jj] - A[i][jj]) > worst) worst = fabs(T[i][jj] - A[i][jj]);
        snprintf(buf, sizeof buf, "accum(T1..T6) 与 dh_forward 最大差 %.2e", worst);
        check("辅助函数 accum() 与 dh_forward 同口径", worst < 1e-9, buf);
    }
    /* home 位姿【回归】锚点：真机回零后 getpos 打印 X=333.00 Y=52.00 Z=286.00
 *   （d6=183 口径；d6 还是 91.5 时同一姿态读作 241.55/51.85/286.05 —— 见 dh_params.h）
     * （回零把编码器带到 ROBOT_HOME_MECH_DEG={0,0,90,0,0,0}）。
     *
     * ⚠️ 这不是"物理验证"，别被它骗了：getpos 打印的就是 FK(编码器读数)，
     *    读数 + 模型天然自洽 —— 这个数吻合【证明不了 DH 模型符合真实机械】。
     *    它唯一的作用是【回归】：改坏 DH_TABLE 或 theta_offset 时立刻报红。
     *    真正能证伪模型的只有外部物理参照，见 docs/关节符号与零点验证.md。
     *
     * 把这段想清楚很值钱：我第一版把它当成了"物理锚点"，等于自己骗自己 ——
     * 这正是不做外部参照就永远发现不了符号/零点错误的根本原因。 */
    {
        double home[3] = {333.00, 52.00, 286.00};
        tcp(q, p);
        snprintf(buf, sizeof buf,
                 "FK=(%.2f,%.2f,%.2f) getpos=(%.2f,%.2f,%.2f) 差 %.3f mm",
                 p[0], p[1], p[2], home[0], home[1], home[2], dist3(p, home));
        check("FK home 位姿回归（非物理验证，见注释）", dist3(p, home) < 0.5, buf);
    }

    /* ① J4/J5/J6 三轴必须交于一点（腕心）。d5 = 0 ⇒ o5 == o4，
     *   而 J4 轴过 o4、J5 轴过 o4、J6 轴过 o5 ⇒ 三线共点。 */
    {
        double o4[3], o5[3];
        frame_origin(q, 4, o4);
        frame_origin(q, 5, o5);
        snprintf(buf, sizeof buf, "o4=(%.2f,%.2f,%.2f) o5=(%.2f,%.2f,%.2f) 相距 %.6f mm",
                 o4[0], o4[1], o4[2], o5[0], o5[1], o5[2], dist3(o4, o5));
        check("J4/J5/J6 三轴交于一点（腕心）", dist3(o4, o5) < 1e-6, buf);
        wc[0] = o4[0]; wc[1] = o4[1]; wc[2] = o4[2];
    }

    /* ② 相邻轴垂直：J4⊥J5、J5⊥J6 */
    joint_axis(q, 4, z4);
    joint_axis(q, 5, z5);
    joint_axis(q, 6, z6);
    snprintf(buf, sizeof buf, "J4·J5 = %+.6f, J5·J6 = %+.6f", dot3(z4, z5), dot3(z5, z6));
    check("相邻轴垂直（J4⊥J5, J5⊥J6）",
          fabs(dot3(z4, z5)) < 1e-9 && fabs(dot3(z5, z6)) < 1e-9, buf);

    /* ③ θ5 = 0 是腕部奇异点：此时 J4 轴与 J6 轴共线 */
    snprintf(buf, sizeof buf, "J4·J6 = %+.6f（1.0 = 共线 = 奇异）", dot3(z4, z6));
    check("θ5=0 时 J4 与 J6 共线（腕部奇异点）", fabs(dot3(z4, z6) - 1.0) < 1e-9, buf);

    /* ④ TCP 在 J6 轴上，距腕心 d6 */
    tcp(q, p);
    snprintf(buf, sizeof buf, "|TCP-腕心| = %.3f mm（d6 应为 183.00）", dist3(p, wc));
    check("TCP 在 J6 轴上、距腕心 = d6", fabs(dist3(p, wc) - 183.0) < 1e-6, buf);

    printf("\n=== 2. 转动效应对 TCP 位置的影响 ===\n");

    /* ⑤ 转 J6 永远不改 TCP 位置（TCP 在 J6 轴上） */
    {
        double qq[6] = {0,0,90,0,0,90};
        tcp(q, p0); tcp(qq, p1);
        snprintf(buf, sizeof buf, "J6: 0→+90°，TCP 位移 %.6f mm", dist3(p0, p1));
        check("转 J6 不改 TCP 位置（任意位形）", dist3(p0, p1) < 1e-6, buf);
    }
    /* θ5=45 时再验一次 */
    {
        double a[6] = {0,0,90,20,45,0}, b[6] = {0,0,90,20,45,75};
        tcp(a, p0); tcp(b, p1);
        snprintf(buf, sizeof buf, "J5=45 时 J6: 0→+75°，TCP 位移 %.6f mm", dist3(p0, p1));
        check("转 J6 不改 TCP 位置（J5=45 复核）", dist3(p0, p1) < 1e-6, buf);
    }

    /* ⑥ θ5=0（奇异）时转 J4 也不改 TCP；θ5≠0 时必须改 */
    {
        double qq[6] = {0,0,90,90,0,0};
        tcp(q, p0); tcp(qq, p1);
        snprintf(buf, sizeof buf, "J5=0 时 J4: 0→+90°，TCP 位移 %.6f mm", dist3(p0, p1));
        check("θ5=0 时转 J4 不改 TCP（奇异的几何特征）", dist3(p0, p1) < 1e-6, buf);
    }

    /* ⑦ wrist-flip 等价性：(θ4,θ5,θ6) 与 (θ4+180,-θ5,θ6+180) 位姿相同 */
    {
        double a[6] = {10,-20,95,30,40,-15};
        double b[6] = {10,-20,95,210,-40,165};
        double Ta[4][4], Tb[4][4];
        double worst = 0.0;
        int i, jj;
        dh_forward(DH_TABLE, a, Ta);
        dh_forward(DH_TABLE, b, Tb);
        for (i = 0; i < 4; i++)
            for (jj = 0; jj < 4; jj++)
                if (fabs(Ta[i][jj] - Tb[i][jj]) > worst) worst = fabs(Ta[i][jj] - Tb[i][jj]);
        snprintf(buf, sizeof buf, "两解位姿矩阵最大差 %.2e", worst);
        check("wrist-flip 等价：(θ4,θ5,θ6)≡(θ4+180,-θ5,θ6+180)", worst < 1e-8, buf);
    }

    printf("\n=== 3. 验证方案所需的预测值（供 docs/关节符号与零点验证.md）===\n");

    printf("  【通用放大技巧】在法兰上贴一根长 L 的硬指针（沿法兰 y 向）。\n");
    printf("     转动 θ ⇒ 指针尖端位移 ≈ 2·L·sin(θ/2)。L=200 时 1°≈3.5mm、\n");
    printf("     5°≈17mm、30°≈104mm。肉眼门槛约 3mm ⇒ 裸眼只看得出 ≥2°，\n");
    printf("     加 200mm 指针能看出 0.5°。下面所有预测都给出「指针尖端位移」。\n\n");

    printf("  【A. J4 符号】J5 取 45°（离开奇异），J4 分别取 +90 / -90：\n");
    {
        double a[6] = {0,0,90,90,45,0};
        double b[6] = {0,0,90,-90,45,0};
        tcp(a, p0); tcp(b, p1);
        printf("      J4=+90 ⇒ TCP = (%.1f, %.1f, %.1f)\n", p0[0], p0[1], p0[2]);
        printf("      J4=-90 ⇒ TCP = (%.1f, %.1f, %.1f)\n", p1[0], p1[1], p1[2]);
        printf("      两者相距 %.1f mm ⇒ 真机上 TCP 落在哪边，就说明 J4 符号是哪边\n",
               dist3(p0, p1));
    }

    printf("  【B. J6 符号】J6 转不影响位置，只能靠法兰上的标记。\n");
    printf("     在法兰上做个偏离轴心 10mm 的标记（法兰局部坐标 (0,10,0)）：\n");
    {
        double a[6] = {0,0,90,0,45,0};      /* J5=45 让法兰轴不水平，便于描述 */
        double b[6] = {0,0,90,0,45,90};
        double w0[3], w1[3];
        flange_point(a, 0, 10, 0, w0);
        flange_point(b, 0, 10, 0, w1);
        printf("      J6=0   ⇒ 标记世界坐标 (%.1f, %.1f, %.1f)\n", w0[0], w0[1], w0[2]);
        printf("      J6=+90 ⇒ 标记世界坐标 (%.1f, %.1f, %.1f)\n", w1[0], w1[1], w1[2]);
        printf("      标记位移 %.1f mm（沿哪个方向动，就对照上面的世界坐标判断）\n",
               dist3(w0, w1));
    }

    printf("  【C. J5 零点】★方向无关★ —— 只需看末端【动没动、动了多少】，\n");
    printf("     完全不需要知道世界坐标系朝哪边。\n");
    printf("     θ5=0 是奇异点 ⇒ J4 轴与 J6 轴共线 ⇒ 同时下发 J4=+A、J6=-A\n");
    printf("     时两个电机的效应应当【完全抵消】⇒ **末端纹丝不动**（电机空转）。\n");
    printf("     J5 零点偏 e ⇒ 两轴不再共线 ⇒ 抵消不掉 ⇒ 末端摆动：\n\n");
    {
        int A, ei;
        double e, L = 200.0;
        /* 显式列出而不是等差递增：文档要逐格照抄，e=1° 这一档是判断
         * "到底能看出多小的偏差"的关键，不能靠相邻的 2° 外推。 */
        const double es[] = {0.0, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0};
        for (A = 60; A <= 120; A += 60) {
            printf("      A=%d° │ 零点偏 e │ TCP 位移 │ 法兰轴倾角 │ 指针尖端位移(L=200)\n", A);
            for (ei = 0; ei < (int)(sizeof es / sizeof es[0]); ei++) {
                e = es[ei];
                double a[6] = {0,0,90,0,e,0};                    /* 起点：J5 物理角 = e */
                double b[6] = {0,0,90,(double)A,e,-(double)A};   /* 终点：J4=+A, J6=-A */
                double Ta[4][4], Tb[4][4], w0[3], w1[3];
                double ax[3], bx[3], d;
                tcp(a, p0); tcp(b, p1);
                dh_forward(DH_TABLE, a, Ta);
                dh_forward(DH_TABLE, b, Tb);
                ax[0]=Ta[0][2]; ax[1]=Ta[1][2]; ax[2]=Ta[2][2];
                bx[0]=Tb[0][2]; bx[1]=Tb[1][2]; bx[2]=Tb[2][2];
                d = dot3(ax, bx);
                if (d > 1.0) d = 1.0;
                if (d < -1.0) d = -1.0;
                flange_point(a, 0, L, 0, w0);
                flange_point(b, 0, L, 0, w1);
                printf("             │ %7.0f°  │ %7.2f mm │ %8.2f°   │ %8.1f mm\n",
                       e, dist3(p0, p1), acos(d) * RAD2DEG, dist3(w0, w1));
            }
            printf("\n");
        }
    }

    printf("  【D. J4 与 J6 的【相对】符号】★方向无关、最灵敏★\n");
    printf("     同样在 J5=0 下发 J4=+A、J6=-A：\n");
    printf("       · 两轴符号约定【一致】（都对 / 都反）⇒ 抵消 ⇒ 末端不动\n");
    printf("       · 两轴符号约定【相反】（其中恰好一个反了）⇒ 实际变成\n");
    printf("         (-A) 与 (-A) 叠加 ⇒ **法兰自转 2A**，量级大到不可能看错\n");
    printf("     （注：两轴【都】反了这条测不出来，得靠 A 节的外部方向参照）\n");
    {
        int A;
        double L = 200.0;
        printf("      A │ TCP 位移 │ 法兰自转 │ 指针尖端位移(L=200)\n");
        for (A = 30; A <= 90; A += 30) {
            double a[6] = {0,0,90,0,0,0};                        /* 起点 home */
            double b[6] = {0,0,90,-(double)A,0,-(double)A};      /* s4≠s6 时的物理角 */
            double Ta[4][4], Tb[4][4], w0[3], w1[3];
            double ax[3], bx[3], d;
            tcp(a, p0); tcp(b, p1);
            dh_forward(DH_TABLE, a, Ta);
            dh_forward(DH_TABLE, b, Tb);
            /* 自转是绕法兰【自身】轴转 ⇒ 不能量 z 轴方向的夹角（绕自身轴转，
             * 自身轴方向当然不变，第一版就在这里量出了 0.0° 的假象）。
             * 必须量垂直于轴的方向，这里取法兰 y 轴（也就是指针的方向）。 */
            ax[0]=Ta[0][1]; ax[1]=Ta[1][1]; ax[2]=Ta[2][1];
            bx[0]=Tb[0][1]; bx[1]=Tb[1][1]; bx[2]=Tb[2][1];
            d = dot3(ax, bx);
            if (d > 1.0) d = 1.0;
            if (d < -1.0) d = -1.0;
            flange_point(a, 0, L, 0, w0);
            flange_point(b, 0, L, 0, w1);
            printf("    %3d° │ %7.2f mm │ %7.1f° │ %8.0f mm\n",
                   A, dist3(p0, p1), acos(d) * RAD2DEG, dist3(w0, w1));
        }
        printf("      （对照：符号一致时这三项全为 0，即 C 节 e=0 那一行）\n");
    }

    printf("\n%s（失败项 %d）\n",
           g_fail ? "*** 有失败 ***" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
