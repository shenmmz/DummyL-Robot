/*
 * commands.c —— CLI 命令实现与分发
 * ------------------------------------------------------------
 * 所属模块：应用命令层（cli）
 * 对外接口：cmd_dispatch
 * 支持命令：home、MoveJ、MoveL、disable、enable、motor、getpos、zero、
 *   zero_save、help、exit
 * main.c 仅负责初始化与命令循环。
 */

#include "cli/commands.h"
#include "control/robot.h"
#include "control/home.h"
#include "control/monitor.h"
#include "control/robot_internal.h"
#include "api/motor_reg.h"
#include "utils/err.h"
#include "utils/ini_rw.h"
#include "utils/telemetry.h"
#include "kinematics/joint_zero.h"
#include "comm/modbus_rtu.h"
#include "kinematics/dh.h"
#include "kinematics/ik.h"
#include "trajectory/line.h"
#include "config/robot_config.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <windows.h>

/* ====================== 内部辅助 ====================== */

#define MOVEJ_POLL_MS      10
#define MOVEJ_INPOS_TOL   100
#define MOVEJ_TIMEOUT_MS  60000
/* 实时状态行的最小刷新间隔(ms)：轮询周期本身被总线读占用（6 事务 ≈160ms），
 * 每次循环都 printf 会让日志被 \r 行刷爆。200ms 一刷足够看趋势。 */
#define MOVEJ_STATUS_MS   200
/* 状态行固定宽度：\r 覆盖时若新行比旧行短，行尾会残留上一轮字符
 * （曾出现 "偏差 0.00mmmm" —— spd 从 -17 变 0 短了 2 字符，前一轮的 "mm" 留了下来）。
 * 故统一按固定宽度输出，结束时整行擦除。 */
#define MOVEJ_STATUS_W    78
/* 单轴最低转速(rpm)：movej_issue 靠"转速按行程比例分配"让六轴同起同停，
 * 行程小的轴分到的转速是个位甚至小数，这里是防止它小到驱动器执行不了。
 * 注意两点：
 *   1) 绝不能抬高——一旦某轴被下限截断，它就提前到位，最后一段只剩其余几轴在动，
 *      笛卡尔末端就是一段弧（"逼近目标点画弧"的根因）。比例分配保证下限只在
 *      "该轴行程 < 最大轴的 最低转速/指令转速"（10rpm 时即 10%）时才生效，
 *      此时提前到位的偏差以该轴自身那点行程为界，可忽略。
 *   2) 依赖 motor_set_speed 传小数：寄存器单位 0.01rpm，若形参是 int 会把
 *      0.28rpm 截成 0 ⇒ 该轴完全不动。曾因此把 0.05 写成 0 导致 J2 前两拍不动、
 *      J3 最终超时急停。 */
#define MOVEJ_MIN_RPM       1.0
/* 加减速安全下限（ms）。【必须强制取 max，不能只在 <=0 时兜底】
 * 现场踩到（2026-09-18）：movel 的 dec 手滑输成 9（本意 90），旧写法
 * `x > 0 ? x : FLOOR` 只对非正数兜底，9 被原样下发——9ms 内从运行速度减速到
 * 停止相当于急停，步进必然丢步，位置永远进不了到位容差 ⇒ movej_wait 一直
 * 等到 60s 超时，表现为"卡住不动"。过短的加减速还会触发动态误差报警。
 * 所以下限要对所有过小值生效，而不是只兜非正数。 */
#define MOVL_ACC_FLOOR_MS   60
/* sync 航点弦长：由【弓高预算】反推，不再写死一个步长。
 *
 * 为什么改：弓高 ∝ 弦长²，而"这一段有多弯"取决于方向——同一条 50mm 直线，
 * 沿某些方向关节空间几乎是直线（根本不用分段），沿另一些方向弯得厉害。
 * 写死 10mm 对所有方向一刀切：对直的方向分了多余的段（每段一停 ⇒ 白白卡顿），
 * 对弯的方向又可能不够密。
 *
 * 做法：先算"整段不分段"的弓高 bow_full，按 bow ∝ L² 反推满足预算的弦长
 *     L = 位移 × sqrt(MOVL_SYNC_BOW_MM / bow_full)
 * 特别地，bow_full ≤ 预算时【一段走完】，中间零停顿——这是短线段最大的收益
 * （旧逻辑 20mm 线也要切成 2 段，白停一次）。
 *
 * 调大预算 = 段少、顺、但弯；调小 = 直、但段多、卡。
 *
 * 【默认值 1.0 的依据（2026-09-18 实测）】
 * 偏差 vs 段数是【U 形】，不是单调的——这点曾被我搞错，别再写"越少越直"：
 *     段数   几何弓高     实测最大偏差    放大倍数   （50mm 直线，整段弓高 2.08mm）
 *      1      2.08 mm    **2.11 mm**     1.0×   无段间启动差，偏差≈纯弧高
 *      2      0.33~0.40  **1.15~1.37**   3.4×   ← 最优
 *      3      0.24 mm    **1.44~1.74**   6~7×
 *      5      0.39 mm    **4.59 mm**    11.9×
 * 两个误差源此消彼长：段数多 ⇒ 段间启动差累积（每段"减速到0 → 空等约90ms
 * 等轮询发现到位 → 再起步"，首末轴相隔约 75ms）；段数少 ⇒ 纯几何弧高全部保留。
 *
 * 1.0 让 50mm 自动分 2 段（n = ceil(位移/L)，L = 位移×sqrt(budget/整段弓高)
 * ⇒ L=34.7 ⇒ n=ceil(1.44)=2），落在最优点；长距离自动多分以压住弧高。
 * 注意本值只是 ini 读不到时的兜底，实际以 [movel] bow_mm 为准，两者要同步改。
 *
 * 用户的取向仍是"宁可有点弧也不要一卡一卡"，但实测证明 2 段比 1 段【更直】
 * 且只停 1 次（3 段要停 2 次）⇒ 2 段同时优于 1 段和 3 段，不必牺牲精度换流畅。
 * 要完全不分段改 3.0 或用 smooth；0.10 实测【更歪】，不要用。
 *
 * 【⚠️ 2026-09-19：上面这套"U 形 / 2 段最优"不能直接推广】
 *   ① 它是【50mm、特定方向】的数据，而弓高强烈依赖方向（沿某些方向关节空间
 *      几乎是直线）。方向一换，最优段数就变了。
 *   ② 更要命的是程序打印的【几何弓高预测】本身不可信：100mm Y 向线预测"弯
 *      9.55mm"，用户实测却【非常直】。之前还发生过低估 11.6 倍（4.55 vs 0.39）。
 *   ⇒ **别再拿预测值劝用户放弃某个方案**，也别把这里的 50mm 数当成通用规律。
 *   ⇒ 当前结论：画线直接用 smooth（单段），既不卡也不弯。 */
#define MOVL_SYNC_BOW_MM   2.0     /* 仅 ini 读不到时的兜底，与 [movel] bow_mm=2.0 保持一致 */
#define MOVL_MIN_STEP_MM   2.0    /* 弦长下限(mm)，防止反推出过密弦把段数撑爆 */

/* MOVL_TIP_WARN_DEG：MoveL 目标姿态与【当前】姿态相差多少度就告警。
 *
 * 【为什么必须有这道告警 —— 2026-09-19 真机事故】
 * 用户报"画的是斜线、落点不对"，但离线复算【法兰】轨迹是完美的直线
 * （80mm 线直线度 0.0000mm）。真因是 moveL 的 Rx,Ry,Rz 抄错了：
 * 抄成了别的姿态下的值（home 姿态 getpos 打印 Ry=89.99，绘图位姿实际是 0）。
 * moveL 用 SLERP 把姿态从起点【一路拧到】目标 ⇒ 中途姿态一直在变 ⇒
 * 装在法兰下方 L mm 处的笔尖绕法兰摆 **L×2sin(θ/2)**（θ = 法兰倾角变化）。
 * 【⚠️ 公式别写成 L×sin(θ)】θ→180° 时 sin(θ)→0，会算出"几乎不动"的荒谬结论
 * （90° 时前者给 41.17、后者给 58.2；179° 时前者给 0.72、后者给 82.3，完全反过来）。
 * 正确式子的几何意义就是两个笔尖位置的距离 |n1−n0|×L。
 *
 * 【离线实测（80mm Y 向线，起点 J=0,0,110,0,70,0，笔长 41.17）】
 *   姿态填 -180,0,-180（= 起点）⇒ 倾角变化 0.000°  笔尖纸面偏离 0.0000 mm
 *   姿态填 115,90,115（home 抄的）⇒ 倾角变化 88.9°  笔尖纸面偏离 7.71 mm
 *   姿态填 0,90,0 / 180,90,180   ⇒ 倾角变化 88.9°  笔尖纸面偏离 7.71 mm
 * ⇒ 法兰 0mm、笔尖 7.7mm：只看法兰坐标【永远发现不了】这个问题。
 * ⇒ 注意 88.9° 的空间摆幅是 41.17×2sin(44.45°)=57.6mm，但【纸面上只表现为 7.71mm】：
 *    那一摆主要是把笔"从朝下抬到朝水平"，竖直分量占大头，水平投影才 7.71mm。
 *    所以告警里的毫米数是空间摆幅上界，别当成纸上偏差。
 *
 * 默认 5° 的依据：41.17mm 的笔在 5° 下摆 41.17×2sin(2.5°)=3.6mm，
 * 在 100mm 的线上已经肉眼可见地斜了。装短工具时可以调大。 */
#define MOVL_TIP_WARN_DEG  5.0
/* 实际发生的总线【写】事务计数（movej_issue 里累加），stream 用它折算单事务耗时。
 * 常量估算在"跳过重复速度写"之后会虚报，改为实点。 */
static int    g_tx_written = 0;
#define MOVL_STEP_MM        1.0     /* 直线插补弦步长(mm)：越小越直、点越密（封顶 LINE_MAX_POINTS） */
#define MOVL_STREAM_MIN_MS  20      /* stream 单节拍下限(ms)：防 seg_dt 过小打爆总线；实际节拍由总线耗时与 seg_dt 取大者 */
/* stream 目标节拍(s)：一个节拍要写 6 轴速度+绝对位置(12 事务)，115200 下实测约 8ms/事务
 * ⇒ 单节拍约 100ms。这是 PC 端周期下发的物理下限，弦步长必须按它放大，
 * 否则段行程 1.4ms 就走完、剩余 95ms 干等，退化成比 step 更差的"走一步停一下"。
 * 换更快的总线或更少的轴时按实测定值调小本常量。 */
#define MOVL_STREAM_BEAT_S  0.10    /* stream 目标节拍(s)：段行程应≈一个节拍走完，见上方说明 */
#define MOVL_SEG_RAMP_RATIO 3.0     /* 段时长下限 = 本值 ×(ACC+DEC)：段比斜坡短 ⇒ 驱动器反复刹车/爬坡，又慢又顿。
                                     * 现场标定：每段都是"从 0 加速→巡航→减速到 0"（实测 50 段 ×(99+115)ms ≈ 实跑时长），
                                     * 所以段数越少越顺；取 3 时巡航占比 ≈63%。
                                     * 【已删掉的一句】原注释写"弓高仍 <0.15mm"是纯几何推算值，
                                     * 2026-09-18 实测打脸：分段实测偏差是几何弓高的 3~12 倍
                                     * （2 段 1.15~1.37mm / 3 段 1.44~1.74mm / 5 段 4.59mm）。
                                     * 几何弓高只能用来比较"两种分法的相对优劣"，
                                     * 不能当成实际偏差上界，别再拿它做验收标准。 */
#define MOVL_BOW_SAMPLES    24      /* 估算弓高的关节空间采样数（每段） */
/* 弓高警告阈值：不再用固定 mm 值——"多少弧算大"是用户用 bow_mm 表达的取舍，
 * 写死一个值会在 bow_mm 调大后把用户已接受的弧当成错误反复报警。
 * 现改为与 bow_budget 相对比较（见下处用法），本常量仅作"预算读不到"时的兜底。 */
#define MOVL_BOW_WARN_MM    1.0
#define MOVL_STREAM_PROBE_MS 150    /* stream 段内偏差采样所需空闲时间(ms)：约 6 个读事务(6×20ms)+余量。
                                     * 节拍剩余不足这个数就不采样（宁可不采，也不能挤占本段下发） */
/* 腕部奇异预警阈值(度)：规划路径上出现 |J5| 小于此值的点就告警。
 * θ5≈0 时 θ4 与 θ6 同轴、自由度退化，见 movl_plan 里的说明。 */
#define MOVL_WRIST_SINGULAR_DEG 5.0
/* 甩臂预警阈值(度)：相邻插补点之间单个关节的转角超过此值就告警。
 * 弦步长默认 0.5mm，正常路径上单关节单段变化只有零点几度（实测干净路径
 * 最大 0.56°），因此 15° 已经是很宽松的界 —— 真超了基本可以断定是擦过
 * 奇异位形或逆解分支翻转。详见 movl_plan 体检①。 */
#define MOVL_JUMP_WARN_DEG     15.0
/* 单次下发的位移上限(机械角度)，ini [safety] max_step_deg 可覆盖。
 * 依据与取值理由见 movl_max_step_deg 的注释（默认 = J4/J6 软限位全宽 720°）。 */
#define MOVEJ_MAX_STEP_DEG     720.0
/* MoveL 规划层"单段跳变"的【拒绝】阈值(度)，ini [safety] max_jump_deg 可覆盖。
 * 与上面 MOVL_JUMP_WARN_DEG 是两档：15° 只告警、30° 直接拒绝。
 *
 * 【为什么是 30】实测两端差两个数量级：
 *   正常路径单段 <1°（home 沿 -Z 走 50mm，1mm 弦 ⇒ J3 全程 21° 分 50 段）；
 *   病态路径第 1 段就要 J4 转 89.98°（home 沿 +Y 走，起点 J5=0 正踩腕奇异）。
 * 取 30° 两边都留了极大余量：既不误伤正常作业，也绝不会放过 90° 的甩臂。
 * 判据本体 line_max_joint_jump() 在 trajectory/line.c（纯函数，已单测）。 */
#define MOVL_JUMP_MAX_DEG      30.0

/* 读回体检阈值（movej_wait 的"读回不可能是真的"检测）：
 * MARGIN  单轴越软限位超过这么多度就判异常。取 15° —— 规划目标全在限位内，
 *         正常 overshoot 是零点几度，15° 只有"跑飞/读回假"才够得着；
 *         2026-09-19 事故里 J5 越了 20.44°，正好被这条逮住。
 * POLLS   连续这么多轮都异常才认定，滤掉单次干扰（一轮约 160ms ⇒ 约 0.5s 内收敛）。 */
#define MOVEJ_ANOM_MARGIN_DEG  15.0
#define MOVEJ_ANOM_MIN_POLLS   3

/* 前向声明：定义在下方（与 movl_bow_budget 等 ini 读取函数放一起），
 * 但 movej_issue 在它之前就要用。 */
static double movl_max_step_deg(void);
static double movl_max_jump_deg(void);
#define MOVL_EPS_MM         0.05    /* "已在目标位姿"判据：位置位移小于此值(mm)认为没动 */
#define MOVL_EPS_DEG        0.05    /* 同上，姿态角(deg)；两者同时满足才早退 */

/* 六轴位置连发时的帧间延迟(ms) —— 由 `nrtest` 实测定出，不要拍脑袋改：
 *   nrtest 结果（反复下发当前位置、原地不动，看读回是否出错）：
 *       0 ms → 一轮 6.30 ms，读失败 1/6   ← 撞车
 *       1 ms → 一轮 10.90 ms，读失败 1/6  ← 撞车
 *       2 ms → 一轮 17.10 ms，读失败 1/6  ← 撞车
 *       3 ms → 一轮 21.80 ms，读失败 0/6  ← 干净
 *       5 ms → 一轮 34.40 ms，读失败 0/6  ← 干净
 * 取 4 ms = 干净阈值 3ms + 1ms 余量（运动时从站响应可能比静止时略慢）。
 * 收益：一轮六轴从 92ms(10.8Hz) 降到约 26ms(38Hz)，
 *       六轴启动差从约 75ms 降到约 26ms —— 这是画直线画歪的主因。 */
#define MOVEJ_NR_GAP_MS    4

/* 前向声明：line_a/line_b 非空时，等待循环中顺便打印末端相对理想直线的实时偏差 */
static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                          const double angles[6], double speed,
                          int accel_ms, int decel_ms,
                          const double *line_a, const double *line_b);
static void movej_multi(Robot *robot, const ParsedCmd *cmd);

/* movl_point_dev_mm：点 x 到直线段 AB 的距离(mm)，用于实测轨迹偏差读数 */
static double movl_point_dev_mm(const double x[3], const double a[3], const double b[3])
{
    double ab[3], ab2 = 0.0, t = 0.0, d2 = 0.0;
    int i;
    for (i = 0; i < 3; i++) { ab[i] = b[i] - a[i]; ab2 += ab[i] * ab[i]; }
    if (ab2 < 1e-9) return 0.0;
    for (i = 0; i < 3; i++) t += (x[i] - a[i]) * ab[i];
    t /= ab2;
    if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    for (i = 0; i < 3; i++) {
        double e = (x[i] - a[i]) - t * ab[i];
        d2 += e * e;
    }
    return sqrt(d2);
}

/* ====================== 电机实时监控线程 ====================== */

typedef struct {
    Robot *robot;
    volatile LONG running;
    HANDLE thread;
} MotorMonitor;

static MotorMonitor *g_motor_mon = NULL;

/* 当前监控器（cmd_dispatch 入口处记下）。MoveL 的过流保护阈值从它读，
 * 这样 stall:N:MA 的运行时改动能立刻生效 —— 见 movl_stall_thresholds。 */
static Monitor *g_mon = NULL;

/* motor_monitor_thread：后台线程主循环，每1秒打印一次全部电机信息 */
static DWORD WINAPI motor_monitor_thread(LPVOID arg)
{
    MotorMonitor *mm = (MotorMonitor *)arg;

    /* 使用 InterlockedCompareExchange 原子读 running，与写入侧
     * InterlockedExchange 配对，避免非 x86 平台上的撕裂读。 */
    while (InterlockedCompareExchange(&mm->running, 1, 1) != 0) {
        char ts[32];
        {
            SYSTEMTIME lt;
            GetLocalTime(&lt);
            snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
                     lt.wYear, lt.wMonth, lt.wDay,
                     lt.wHour, lt.wMinute, lt.wSecond);
        }

        for (int j = 1; j <= 6; j++) {
            int pos, cur, spd;
            double deg;
            int ok = 0;
            pos = motor_read_position(mm->robot, j, NULL);
            cur = motor_read_current(mm->robot, j);
            spd = motor_read_speed(mm->robot, j);
            deg = robot_read_position_deg(mm->robot, j, &ok);
            printf("%s  关节%d: 脉冲=%d  机械角=%.2f°  速度=%d rpm  电流=%d mA\n",
                   ts, j, pos, deg, spd, cur);
        }
        printf("\n");
        fflush(stdout);

        /* 等待1秒，期间每500ms检查一次 running 是否被置0 */
        for (int i = 0; i < 2; i++) {
            if (mm->running == 0) break;
            Sleep(500);
        }
    }
    return 0;
}

static MotorMonitor *motor_monitor_create(Robot *robot)
{
    MotorMonitor *mm = (MotorMonitor *)calloc(1, sizeof(MotorMonitor));
    if (mm == NULL) return NULL;
    mm->robot = robot;
    mm->running = 0;
    mm->thread = NULL;
    return mm;
}

static int motor_monitor_start(MotorMonitor *mm)
{
    if (mm == NULL || mm->thread != NULL) return 0;
    InterlockedExchange(&mm->running, 1);   /* 与线程内原子读配对 */
    mm->thread = CreateThread(NULL, 0, motor_monitor_thread, mm, 0, NULL);
    return mm->thread != NULL;
}

static void motor_monitor_stop(MotorMonitor *mm)
{
    if (mm == NULL || mm->thread == NULL) return;
    InterlockedExchange(&mm->running, 0);
    WaitForSingleObject(mm->thread, 2000);
    CloseHandle(mm->thread);
    mm->thread = NULL;
}

static void motor_monitor_destroy(MotorMonitor *mm)
{
    if (mm == NULL) return;
    motor_monitor_stop(mm);
    free(mm);
}

static int motor_monitor_is_running(MotorMonitor *mm)
{
    return (mm != NULL && mm->thread != NULL) ? 1 : 0;
}

/* cmd_motor_running：查询电机监控线程是否在运行 */
int cmd_motor_running(void)
{
    return motor_monitor_is_running(g_motor_mon);
}

/* ====================== 堵转保护：电流实测与逐轴阈值 ======================
 *
 * 立三没有堵转状态寄存器，0x001A 实时电流是唯一判据。阈值必须【逐轴】：
 * 六轴机座大小不同、额定电流能差好几倍，一个全局值必然要么大轴一动就误报、
 * 要么小轴撞死都不报（旧代码正是这个设计，且 main.c 传 0 = 检测从没开过）。
 *
 * 阈值属于"用户决策"，代码不自作主张填数字，只负责搭框架 + 提供测量工具：
 *   curtest  量出各轴 保持电流 / 运动峰值，打印建议区间（只打印，不写配置）
 *   stall    查看与运行时设置；持久化写 ini [stall] j1..j6
 */

typedef struct {
    int    n;      /* 有效样本数 */
    int    min;
    int    max;
    double sum;
} CurStat;

static void curstat_reset(CurStat *s)
{
    s->n = 0; s->min = 0; s->max = 0; s->sum = 0.0;
}

/* 读数 <0（读失败）直接丢弃：拿失败值当电流会算出假的均值/峰值 */
static void curstat_add(CurStat *s, int v)
{
    if (v < 0) return;
    if (s->n == 0) {
        s->min = v; s->max = v;
    } else {
        if (v < s->min) s->min = v;
        if (v > s->max) s->max = v;
    }
    s->sum += (double)v;
    s->n++;
}

static double curstat_mean(const CurStat *s)
{
    return (s->n > 0) ? (s->sum / (double)s->n) : 0.0;
}

#define CURTEST_IDLE_ROUNDS   20      /* 静止采样轮数（每轮 6 轴 ≈90ms ⇒ 约 1.8s） */
#define CURTEST_IDLE_BASE     12      /* 单轴模式下"静止基线"的采样点数 */
/* 起步屏蔽窗(ms)：①躲过发指令瞬间的浪涌电流；②刚下发时状态字可能还残留
 * 上一轮的到位标志(INPOS)，立刻判到位会一帧样本都采不到。 */
#define CURTEST_START_MASK_MS 250
#define CURTEST_MIN_SAMPLES   8       /* 运动段最少样本数，太少说明根本没动起来 */
#define CURTEST_MOVE_TIMEOUT  15000u  /* 单段运动采样超时 ms */

/* curtest_sample_move：下发 target_deg 并全程采样该轴电流（累加进 mv）。
 * 返回 0=正常到位 / 1=超时（已采样本保留） / -1=下发失败。 */
static int curtest_sample_move(Robot *robot, int j, double target_deg,
                               double rpm, CurStat *mv)
{
    uint32_t t0;
    int k = 0;

    if (robot_movej(robot, j, target_deg, rpm) != ERR_NONE) return -1;
    t0 = GetTickCount();
    while ((GetTickCount() - t0) < CURTEST_MOVE_TIMEOUT) {
        curstat_add(mv, robot_read_current_ma(robot, j));
        /* 每 3 个电流样本查一次状态：电流采样优先，状态只用来判断"走完了没" */
        if ((++k % 3) == 0 && (GetTickCount() - t0) > CURTEST_START_MASK_MS) {
            uint32_t st = 0;
            if (robot_read_status(robot, j, &st) == ERR_NONE &&
                (st & LEESN_STAT_INPOS) && mv->n >= CURTEST_MIN_SAMPLES) {
                return 0;
            }
        }
    }
    return 1;
}

/* cmd_curtest：电流实测。joint=0 静止六轴；joint=N 单轴小幅摆动。 */
static void cmd_curtest(Robot *robot, const ParsedCmd *cmd)
{
    /* 巡检会让出总线：采样要独占总线才能拿满采样率，
     * 否则读到的是"排队等锁"的电流，噪声被人为放大。 */
    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    if (cmd->joint == 0) {
        CurStat st[ROBOT_JOINT_COUNT];
        int j, r;

        for (j = 0; j < ROBOT_JOINT_COUNT; j++) curstat_reset(&st[j]);
        printf("curtest 静止采样：六轴【使能保持电流】（%d 轮 ≈ %d ms，一动不动）\n",
               CURTEST_IDLE_ROUNDS, CURTEST_IDLE_ROUNDS * 90);
        for (r = 0; r < CURTEST_IDLE_ROUNDS; r++) {
            for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
                if (robot_is_masked(robot, j)) continue;
                curstat_add(&st[j - 1], robot_read_current_ma(robot, j));
            }
        }
        printf("\n%-8s %10s %10s %10s %8s\n", "关节", "最小mA", "均值mA", "最大mA", "样本");
        for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
            if (robot_is_masked(robot, j)) {
                printf("关节%-4d （已屏蔽，跳过）\n", j);
                continue;
            }
            if (st[j - 1].n == 0) {
                printf("关节%-4d 读取全部失败（离线？）\n", j);
                continue;
            }
            printf("关节%-4d %10d %10.0f %10d %8d\n", j, st[j - 1].min,
                   curstat_mean(&st[j - 1]), st[j - 1].max, st[j - 1].n);
        }
        /* 报警回读：巡检被挂起期间没人报报警，这里必须自己看一眼。
         * 保持电流异常低（几十 mA 甚至 0）通常就是"报警已切断输出"。 */
        {
            int any = 0;
            for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
                int code;
                if (robot_is_masked(robot, j)) continue;
                code = motor_read_alarm(robot, j);
                if (code > 0) {
                    printf("[警告] 关节%d 报警代码 %d = %s\n",
                           j, code, leesn_alarm_text(code));
                    any++;
                }
            }
            if (any == 0) printf("（六轴当前均无报警）\n");
        }
        printf("\n下一步：curtest:N 逐轴量【运动峰值】（会小幅摆动，先确认末端已抬起）\n");
        monitor_pause_active(0);
        return;
    }

    {
        int j = cmd->joint;
        int ok = 0, rc;
        int moved_ok = 0, alarm_after = -1;
        double q0, amp = cmd->angle_deg, rpm = cmd->speed_rpm, target, dir = 1.0;
        const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
        const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
        CurStat base, mv;

        if (robot_is_masked(robot, j)) {
            printf("关节%d 已被屏蔽，无法测量\n", j);
            monitor_pause_active(0);
            return;
        }
        q0 = robot_read_position_deg(robot, j, &ok);
        if (!ok) {
            printf("[错误] 关节%d 读取当前位置失败，无法做摆动测量\n", j);
            monitor_pause_active(0);
            return;
        }
        /* 摆的方向自动挑：朝限位外的那一侧翻过来，绝不往软限位上撞 */
        if (q0 + amp > lmax[j - 1]) dir = -1.0;
        else if (q0 - amp < lmin[j - 1]) dir = 1.0;
        target = q0 + dir * amp;
        if (target > lmax[j - 1] || target < lmin[j - 1]) {
            printf("[错误] 关节%d 当前 %.2f°，两个方向摆 %.2f° 都会出软限位"
                   "（%.1f~%.1f），请减小摆幅\n", j, q0, amp, lmin[j - 1], lmax[j - 1]);
            monitor_pause_active(0);
            return;
        }

        curstat_reset(&base);
        curstat_reset(&mv);
        printf("curtest 关节%d：%.2f° → %.2f° → 回到 %.2f°，转速 %.0f rpm\n",
               j, q0, target, q0, rpm);
        printf("（摆幅上限 10° 由解析器强制；越软限位会自动反向）\n");

        for (int i = 0; i < CURTEST_IDLE_BASE; i++)
            curstat_add(&base, robot_read_current_ma(robot, j));

        rc = curtest_sample_move(robot, j, target, rpm, &mv);
        if (rc < 0) {
            printf("[错误] 关节%d 运动指令下发失败\n", j);
            monitor_pause_active(0);
            return;
        }
        /* 【到位核查 —— 2026-09-18 血的教训】
         * 采样期间巡检被挂起，驱动器报警没人报。曾出现 J5 一动就"相位过流"
         * 报警、驱动器切断输出、电机纹丝不动，而电流读数掉到 0~20mA，
         * 被当成"这轴电流真小"打印出来 —— 完全是假数据。
         * 所以必须回读位置确认它【真的走到了目标】，否则这次测量作废。 */
        {
            int okr = 0;
            double reached = robot_read_position_deg(robot, j, &okr);
            moved_ok = (okr && fabs(reached - target) <= 0.05) ? 1 : 0;
            if (!moved_ok) {
                printf("[错误] 关节%d 没有走到目标：目标 %.2f°，实际 %.2f°"
                       "（指令未被执行）\n", j, target, reached);
            }
            alarm_after = motor_read_alarm(robot, j);
        }
        rc = curtest_sample_move(robot, j, q0, rpm, &mv);   /* 回到原位 */

        printf("\n关节%d 电流实测（0x001A）：\n", j);
        printf("  静止保持：最小 %d / 均值 %.0f / 最大 %d mA（%d 样本）\n",
               base.min, curstat_mean(&base), base.max, base.n);
        printf("  运动中  ：最小 %d / 均值 %.0f / 最大 %d mA（%d 样本）%s\n",
               mv.min, curstat_mean(&mv), mv.max, mv.n,
               (rc == 1) ? "  ← 有段没判到到位（超时），峰值可能被截断" : "");
        if (!moved_ok) {
            printf("  ⚠ 本次测量【作废】：轴根本没动，上面的电流是驱动器切断输出后的读数。\n");
            if (alarm_after > 0)
                printf("  ⚠ 关节%d 报警代码 %d = %s（先排除这个再看电流）\n",
                       j, alarm_after, leesn_alarm_text(alarm_after));
            else
                printf("  ⚠ 无报警代码，检查使能/屏蔽/软限位\n");
            monitor_pause_active(0);
            return;
        }
        if (alarm_after > 0) {
            printf("  [警告] 关节%d 报警代码 %d = %s\n",
                   j, alarm_after, leesn_alarm_text(alarm_after));
        }
        if (mv.n > 0) {
            printf("  建议阈值区间：运动峰值 %d × 1.5 ~ × 2.0 = %d ~ %d mA\n",
                   mv.max, (int)(mv.max * 1.5), (int)(mv.max * 2.0));
            printf("  【只是参考，最终由你定】填到 ini [stall] j%d 并重启；0 = 该轴不启用保护。\n", j);
        } else {
            printf("  [警告] 没采到任何运动样本，检查该轴是否使能/在线\n");
        }
    }
    monitor_pause_active(0);
}

/* cmd_stall：查看/运行时设置逐轴堵转阈值。不下发运动、不读总线（电流取快照）。 */
static void cmd_stall(Monitor *mon, const ParsedCmd *cmd)
{
    int j;

    if (mon == NULL) {
        printf("[错误] 监控器未创建，无法查看/设置堵转阈值\n");
        return;
    }

    if (cmd->joint >= 1) {
        int jt = cmd->joint;
        if (cmd->param >= 0.0) {
            int ma = (int)(cmd->param + 0.5);
            monitor_set_stall_threshold(mon, jt, ma);
            printf("关节%d 堵转阈值 = %d mA%s\n", jt, ma,
                   (ma <= 0) ? "（该轴检测已关闭）" : "");
            printf("（只改本次运行；要持久化请写 ini [stall] j%d 后重启）\n", jt);
        }
        {
            MonitorSnapshot snap;
            int th = monitor_get_stall_threshold(mon, jt);
            printf("关节%d：阈值 %d mA（%s）", jt, th, (th > 0) ? "启用" : "关闭");
            if (monitor_snapshot(mon, jt, &snap) == ERR_NONE && snap.current_ma >= 0)
                printf("，最新电流 %d mA\n", snap.current_ma);
            else
                printf("，最新电流未知（尚未巡检/离线）\n");
        }
        return;
    }

    printf("逐轴堵转阈值（mA，0 = 该轴不检测）：\n");
    printf("%-8s %10s %12s %10s\n", "关节", "阈值mA", "最新电流mA", "状态");
    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        MonitorSnapshot snap;
        int th = monitor_get_stall_threshold(mon, j);
        char curbuf[24];
        if (monitor_snapshot(mon, j, &snap) == ERR_NONE && snap.current_ma >= 0)
            snprintf(curbuf, sizeof(curbuf), "%d", snap.current_ma);
        else
            snprintf(curbuf, sizeof(curbuf), "-");
        printf("关节%-4d %10d %12s %10s\n", j, th, curbuf, (th > 0) ? "启用" : "关闭");
    }
    if (monitor_get_stall_threshold(mon, 1) <= 0 &&
        monitor_get_stall_threshold(mon, 2) <= 0 &&
        monitor_get_stall_threshold(mon, 3) <= 0 &&
        monitor_get_stall_threshold(mon, 4) <= 0 &&
        monitor_get_stall_threshold(mon, 5) <= 0 &&
        monitor_get_stall_threshold(mon, 6) <= 0) {
        printf("\n[提示] 六轴阈值全为 0 ⇒ 碰撞保护【未启用】。\n"
               "       先 curtest 量出各轴电流，再 stall:N:MA 试填，稳定后写进 ini [stall]。\n");
    }
}

/* ====================== 位姿合法性闸门 ======================
 *
 * 【为什么需要 —— 2026-09-18 事故催生】
 * 驱动器掉电重启后 0x00D2 零点（RAM，无记忆）被清零，位置计数从 0 重新开始，
 * 但 ini 里存的 q0..q5 标定还在 ⇒ 程序算出来的【机械角全是假值】。
 * 事故当时读出来 J1=180.46 / J2=-73.71 / J5=-113.87，**全部越软限位**，
 * 而程序照样拿这些垃圾去做 FK、照样会下发 MoveL —— 没被拦住的话，
 * 下一步就是按假坐标画图形，直接撞。
 *
 * 判据：任一轴机械角越软限位 ⇒ 要么零点丢了，要么真撞出限位了。
 * 两种情况都必须先回零，不能继续基于位姿发运动命令。
 */
#define POSE_LIMIT_TOL_DEG  1.0   /* 容差：躲过回零后的正常轻微越界 */

static int g_pose_invalid = 0;   /* >0 = 越限位的关节号；0 = 位姿可信 */

/* pose_scan_bad：扫六轴机械角，返回第一个越软限位的关节号（1..6），全正常返回 0 */
static int pose_scan_bad(Robot *robot)
{
    const double lmin[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MIN_DEG;
    const double lmax[ROBOT_JOINT_COUNT] = ROBOT_JOINT_LIMIT_MAX_DEG;
    int j;

    for (j = 1; j <= ROBOT_JOINT_COUNT; j++) {
        int ok = 0;
        double q;
        if (robot_is_masked(robot, j)) continue;
        q = robot_read_position_deg(robot, j, &ok);
        if (!ok) continue;              /* 读不到不判定，交给掉线/报警去报 */
        if (q < lmin[j - 1] - POSE_LIMIT_TOL_DEG ||
            q > lmax[j - 1] + POSE_LIMIT_TOL_DEG) {
            return j;
        }
    }
    return 0;
}

/* cmd_pose_check：扫一遍并在越限时打印告警，返回越限关节号（0=正常）。
 * main.c 启动后调用一次；命令分发层用它锁运动命令。 */
int cmd_pose_check(Robot *robot)
{
    int bad = pose_scan_bad(robot);

    g_pose_invalid = bad;
    if (bad > 0) {
        printf("[警告] 关节%d 当前机械角越软限位 —— 零点【很可能已丢失】\n"
               "       （0x00D2 是 RAM 无记忆寄存器，驱动器掉电即清零，须重新回零）。\n"
               "       位姿读数不可信，MoveL / MoveJ / curtest 已锁定。请先执行 home。\n",
               bad);
    }
    return bad;
}

/* cmd_pose_unlock：人工解锁闸门（poseok）。仅在你确认是误判时用。
 * 解锁前会再扫一次并如实打印，不做"静默放行"。 */
void cmd_pose_unlock(Robot *robot)
{
    int bad = pose_scan_bad(robot);

    if (bad > 0) {
        printf("[警告] 关节%d 仍然越软限位，但按你的要求强制解锁运动命令。\n"
               "       如果零点真的丢了，接下来所有位姿与运动都是错的 —— 后果自负。\n",
               bad);
    } else {
        printf("六轴机械角均在软限位内，闸门解锁。\n");
    }
    g_pose_invalid = 0;
}

/* ====================== 命令分发 ====================== */

/* cmd_dispatch：命令分发总入口
 * 返回 1 表示用户请求退出（exit/quit），否则 0。 */
int cmd_dispatch(Robot *robot, Monitor *mon, const ParsedCmd *cmd)
{
    int j;

    /* 记下监控器，让 MoveL 能读到【运行时】的堵转阈值。
     * 不能让 movl_stall_thresholds 自己去读 ini —— 那样用 stall:N:MA
     * 临时改的阈值对 MoveL 完全无效，而且打印还会谎报"过流保护 开启"。 */
    g_mon = mon;

    /* 【位姿闸门】零点丢失时机械角全是假值，基于位姿的运动命令一律拒绝。
     * 放行 home（它就是要重建零点）、只读命令、以及 enable/disable
     * （手动处理机械臂时要能泄力）。 */
    if (g_pose_invalid &&
        (cmd->type == CMD_MOVEJ ||
         cmd->type == CMD_MOVEL ||
         (cmd->type == CMD_CURTEST && cmd->joint != 0))) {
        printf("[错误] 位姿不可信：关节%d 机械角越软限位，零点可能已丢失 ⇒ 命令已拒绝。\n"
               "       先执行 home 回零；确认是误判可用 poseok 解锁（不推荐）。\n",
               g_pose_invalid);
        return 0;
    }

    switch (cmd->type) {
    case CMD_HOME: {
        motor_monitor_stop(g_motor_mon);
        monitor_stop(mon);
        ErrCode rc = (cmd->joint >= 1) ? robot_home_single(robot, cmd->joint)
                                       : robot_home(robot);
        if (rc != ERR_NONE) printf("[错误] 回零失败：%s\n", err_str(rc));
        /* 回零重建了零点，重新扫一次：正常了就自动解锁闸门 */
        if (cmd_pose_check(robot) == 0) {
            printf("回零后六轴机械角均在软限位内，位姿闸门已解除。\n");
        }
        if (!monitor_start(mon, (int)MONITOR_DEFAULT_INTERVAL_MS))
            printf("[警告] 回零后监控线程重启失败\n");
        break;
    }
    case CMD_MOVEJ: {
        /* 运动期间请后台巡检让出总线。放在【派发层】而不是 cmd_movel 内部：
         * 内部有多个提前 return 分支，在那里挂起极易漏恢复、把巡检永久停住。
         * 安全性：main.c 用 monitor_create(robot, 0)，堵转检测本就没开，
         * 巡检只做状态日志，运动期间暂停没有任何安全损失。 */
        monitor_pause_active(1);
        Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);   /* 等巡检线程走到下一个暂停点 */
        if (cmd->num_joints > 1) {
            movej_multi(robot, cmd);
        } else {
            int can_move = 1;
            double target = cmd->angle_deg;
            if (cmd->rel) {
                int ok = 0;
                double cur = robot_read_position_deg(robot, cmd->joint, &ok);
                if (!ok) {
                    printf("[错误] 关节%d 读取当前位置失败，相对运动无法执行\n", cmd->joint);
                    can_move = 0;
                } else {
                    target = cur + cmd->angle_deg;   /* 相对当前实际位置 */
                }
            }
            /* 越软限位拦截（含相对运动换算出来的目标 —— 当前位置 + 位移，
             * 一样可能越过限位）。判据与 movej_issue 里那条完全同源。 */
            if (can_move) {
                double lmin = 0.0, lmax = 0.0;
                int r = robot_angle_in_soft_limit(cmd->joint, target, &lmin, &lmax);
                if (r < 0) {
                    printf("[错误] 关节号 %d 非法（应为 1..6），已拒绝下发\n", cmd->joint);
                    can_move = 0;
                } else if (r == 0) {
                    printf("[错误] 关节%d 目标 %.2f° 超出软限位 [%.0f, %.0f]°，已拒绝下发。\n"
                           "       越限位的目标必然让电机一路顶到机械极限并触发堵转/过流，\n"
                           "       是一次纯粹的无效冲撞。请改到限位内的角度。\n",
                           cmd->joint, target, lmin, lmax);
                    can_move = 0;
                }
            }
            if (can_move) {
                ErrCode rc = robot_movej(robot, cmd->joint, target, cmd->speed_rpm);
                if (rc != ERR_NONE) printf("[错误] 运动指令失败：%s\n", err_str(rc));
            }
        }
        monitor_pause_active(0);
        break;
    }
    case CMD_MOVEL: {
        /* 同 CMD_MOVEJ：运动期间挂起巡检，把总线完整让给运动指令。
         * 量化依据：巡检一轮 12 笔、周期 300ms ⇒ 约 40 事务/秒，
         * 而单总线实测总吞吐约 65 事务/秒 ⇒ 巡检吃掉约六成带宽，
         * 直接放大"六轴启动差"（画直线画歪的主因）。
         *
         * 【⚠️ 这句话曾被写错过，别再写回去】旧注释说"堵转检测本就没开，
         * 挂起没有安全损失"。给 ini [stall] 配上阈值之后这个前提就不成立了：
         * 运动期间恰恰最需要碰撞保护，而巡检此时一个字节都不上总线。
         * 所以运动期间的保护【由 MoveL 循环内部的 movl_stall_guard 负责】
         * （每段下发前逐轴查电流，超阈即急停），静止时再由巡检兜底。
         * 覆盖不到的只有"整段一次下发"的短线段（没有分段点可插检查）。 */
        monitor_pause_active(1);
        Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);
        cmd_movel(robot, cmd);
        monitor_pause_active(0);
        break;
    }
    case CMD_DISABLE: {
        motor_monitor_stop(g_motor_mon);
        if (cmd->joint >= 1) {
            ErrCode rc = robot_disable(robot, cmd->joint);
            if (rc != ERR_NONE) printf("[错误] 关节%d 失能失败：%s\n", cmd->joint, err_str(rc));
            else printf("关节%d 已泄力(失能)\n", cmd->joint);
        } else {
            for (j = 1; j <= 6; j++) {
                ErrCode rc = robot_disable(robot, j);
                if (rc != ERR_NONE) printf("[错误] 关节%d 失能失败：%s\n", j, err_str(rc));
            }
        }
        break;
    }
    case CMD_ENABLE: {
        int failed = 0;
        int first = (cmd->joint >= 1) ? cmd->joint : 1;
        int last = (cmd->joint >= 1) ? cmd->joint : 6;
        for (j = first; j <= last; j++) {
            ErrCode rc = robot_enable(robot, j);
            if (rc != ERR_NONE) {
                printf("[错误] 关节%d 使能失败：%s\n", j, err_str(rc));
                failed++;
            }
        }
        /* 只有目标轴全部成功才提示成功，避免"最后一轴成功"掩盖前面轴的失败 */
        if (failed == 0) {
            if (cmd->joint >= 1) printf("关节%d 已使能\n", cmd->joint);
            else printf("全部关节已恢复使能\n");
        } else {
            printf("[错误] 有 %d 个关节使能失败\n", failed);
        }
        break;
    }
    case CMD_MOTOR: {
        if (g_motor_mon == NULL) {
            g_motor_mon = motor_monitor_create(robot);
            if (g_motor_mon == NULL || !motor_monitor_start(g_motor_mon)) {
                printf("[错误] 电机监控启动失败\n");
                motor_monitor_destroy(g_motor_mon);
                g_motor_mon = NULL;
            } else {
               printf("电机监控已启动（每1秒刷新，按 q 停止）\n");
            }
        } else {
            motor_monitor_stop(g_motor_mon);
            motor_monitor_destroy(g_motor_mon);
            g_motor_mon = NULL;
            printf("电机监控已停止\n");
        }
        break;
    }
    case CMD_ZERO:
        cmd_zero(robot);
        break;
    case CMD_ZERO_SAVE:
        cmd_zero_save(robot, cmd->zero_vals);
        break;
    case CMD_GETPOS:
        cmd_getpos(robot);
        break;
    case CMD_FK:
        cmd_fk(cmd);
        break;
    case CMD_DIAG:
        cmd_diag(robot, cmd);
        break;
    case CMD_BCAST:
        cmd_bcast(robot);
        break;
    case CMD_NRTEST:
        cmd_nrtest(robot);
        break;
    case CMD_CURTEST:
        cmd_curtest(robot, cmd);
        break;
    case CMD_STALL:
        cmd_stall(mon, cmd);
        break;
    case CMD_POSEOK:
        cmd_pose_unlock(robot);
        break;
    case CMD_HELP:
        cmd_print_help();
        break;
    case CMD_EXIT:
        motor_monitor_stop(g_motor_mon);
        motor_monitor_destroy(g_motor_mon);
        g_motor_mon = NULL;
        return 1;
    case CMD_EMPTY:
        break;
    default:
        printf("[警告] 未知命令，输入 help 查看帮助\n");
        break;
    }
    return 0;
}

/* cmd_zero：显示零点标定数据和当前机械角 */
void cmd_zero(Robot *robot)
{
    const double *zero = joint_zero_get();
    const double target[6] = {0, 0, 90, 0, 0, 0};
    double reading[6];
    double corrected[6];
    int ok[6];
    int all_ok = 1;

    for (int i = 0; i < 6; i++) {
        reading[i] = robot_read_position_deg(robot, i + 1, &ok[i]);
        if (!ok[i]) all_ok = 0;
        corrected[i] = zero[i] + (reading[i] - target[i]);
    }

    printf("当前零点:     {");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", zero[i]);
    printf("}\n");
    printf("当前角度:     {");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", reading[i]);
    printf("}°\n");
    printf("目前零点:     {");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", corrected[i]);
    printf("}\n");

    if (!all_ok) printf("[警告] 部分关节读取失败，显示值仅供参考\n");
}

/* cmd_zero_save：保存指定的零点标定值，内存 + ini 持久化 */
void cmd_zero_save(Robot *robot, const double vals[6])
{
    (void)robot;
    joint_zero_save(vals);
    if (ini_write_joint_zero(INI_PATH, vals)) {
        printf("零点标定已保存（内存 + ini：%s）：\n", INI_PATH);
    } else {
        printf("零点标定已保存(内存)，但写入 ini 失败\n");
    }
    printf("zero_save:");
    for (int i = 0; i < 6; i++) printf(i ? ", %.2f" : "%.2f", vals[i]);
    printf("\n");
}

/* movej_issue：按关节行程比例分配速度并下发绝对位置，【不等待到位】。
 * ref_angles 为行程参考点：
 *   NULL    -> 读各轴当前实际位置算行程（逐段到位模式，多 6 次总线读，最准）；
 *   非 NULL -> 用上一插补点角度算行程（stream 模式，省掉读位置，压单节拍事务数）。
 * set_profile=0 时跳过加减速写入（stream 只在首段写一次）。
 * 速度写：与上次写入值相差 <2% 就跳过（见函数内说明）——单节拍事务数 12 → 6。
 * 输出 tgt/pend（各轴目标脉冲与待到位标记），返回待到位轴数。
 *
 * g_tx_written 累计"实际发生的总线写事务数"，供 stream 诊断折算单事务耗时。
 * 此前用常量估算（首段 18 / 其余 12），跳过速度写后会虚报，改为实点。 */
/* 遥测用的"最近一次完整六轴角"。movej_issue 存目标角当基准，movej_wait 每轮把
 * 读回的轴更新进去、没读到的沿用上次 —— 保证发出去的永远是六个完整值，
 * 不会因为某一轴没参与运动就摆出不存在的姿态（详见 movej_wait 里的注释）。 */
static double g_tlm_mech[6] = {0};
static int    g_tlm_mech_ok = 0;

static int movej_issue(Robot *robot, int num_joints, const int joints[6],
                       const double angles[6], const double *ref_angles,
                       double speed, int accel_ms, int decel_ms, int set_profile,
                       int32_t tgt[7], uint8_t pend[7])
{
    static double last_spd[7] = {0};    /* 上次真正写入的转速(rpm)，用于跳过重复写 */
    static int    last_spd_ok[7] = {0};
    const uint16_t reductions[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    const double *zero = joint_zero_get();
    double dist[7] = {0};
    double max_dist = 0;
    int i, j, remain = 0;

    /* 速度必须为正：speed<=0 时按距离比例分配会得到非法转速，直接拒绝执行
     * （movel 与多关节 movej 均经本函数下发，统一在此兜底校验） */
    if (speed <= 0.0) {
        printf("[警告] 多关节运动速度须大于 0 rpm，已拒绝执行\n");
        return 0;
    }

    /* 【防线三 · 最后一道闸】目标角里只要有一个是 NaN/Inf，就地拒绝下发。
     *
     * 为什么必须在这里拦：DEG2STEPS 把 double 转 int32，NaN 转出来是
     * ±2147483647 附近的垃圾，写进 0x00E8 后驱动器就会朝一个天文数字目标
     * 猛冲、永远到不了位 —— 真机实测：臂乱甩、末端偏差 34mm、最后超时急停
     * （日志里表现为「还差 ±2147228222 步」）。
     * 那次的源头是 line_plan 的 quat_to_euler 在万向锁处 asin 越界返回 NaN
     * （已在 line.c 修根因 + 加了位姿有限性检查），但 NaN 还可能从别的路径
     * 冒出来，而【任何一路 NaN 的代价都是撞机】，所以下发前必须无条件拦。
     * 宁可明确报错不动，也绝不能把垃圾写给驱动器。 */
    for (i = 0; i < num_joints; i++) {
        if (!isfinite(angles[i])) {
            printf("[错误] 关节%d 的目标角不是有限数（%s），已拒绝下发。\n"
                   "       下发 NaN/Inf 会被转成 ±2147483647 附近的步数写进驱动器，\n"
                   "       电机会朝天文数字目标猛冲并超时急停（曾实测甩出 34mm）。\n"
                   "       请检查目标位姿是否落在万向锁（Ry≈±90°）等退化姿态上。\n",
                   joints[i], isnan(angles[i]) ? "NaN" : "Inf");
            return 0;
        }
    }

    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        tgt[j] = DEG2STEPS(angles[i] + zero[j - 1], reductions[j - 1]);
        if (ref_angles != NULL) {
            int32_t ref_steps = DEG2STEPS(ref_angles[i] + zero[j - 1], reductions[j - 1]);
            dist[j] = (double)abs(tgt[j] - ref_steps);
        } else {
            int pos_ok = 0;
            int32_t pos = motor_read_position(robot, j, &pos_ok);
            dist[j] = pos_ok ? (double)abs(tgt[j] - pos) : 0.0;
        }
        if (dist[j] > max_dist) max_dist = dist[j];
    }

    /* 【防线四 · 相对位移闸门】单次下发的位移不得离谱。
     *
     * 出口层（motor_move_abs）那条只挡"绝对值大到不合物理"的垃圾，
     * 挡不住"数值在合理量级内、但离当前位置十万八千里"的逻辑错误 ——
     * 比如单位搞错（度当弧度）、符号搞反、坐标系弄错。这类错误算出来的
     * 目标完全是个"正常的 int32"，只有"相对当前位置太远"能识别。
     *
     * 【为什么零成本】dist[j] 在上面已经算好了（算转速本来就要它），
     * 这里只是加一次比较，不额外占用总线。MoveL 每段都要下发，
     * 多一笔读事务就是 15ms × 6 轴，能省则省。
     *
     * 【阈值】ini [safety] max_step_deg，默认 MOVEJ_MAX_STEP_DEG。
     * 取该关节软限位全宽 + 余量：既然起点终点都在限位内（合法运动的
     * 必要条件），单次位移就不该超过全宽。这跟"用户能不能走出限位"
     * 是两件事 —— 这里只拦明显离谱的，限位本身由 pose_scan_bad 与
     * line_solve 的限位过滤负责。 */
    /* 【防线五 · 越软限位拦截】目标角必须落在该轴的软限位区间内。
     *
     * 【2026-09-18 用户拍板：拦截】此前单轴 MoveJ 完全不查限位（movej:1:500
     * 会照发，而 J1 限位是 -170~179）。理由：目标越限位意味着电机必然一路
     * 顶到机械极限、触发堵转/过流，是一次纯粹的无效冲撞。
     *
     * 【为什么不放进 robot_movej】回零（home.c）也走 robot_movej，而回零的
     * 原理就是朝一个方向顶到堵转 —— 起点已贴着限位、过程中必然越限。塞进
     * robot_movej 会让六轴全部回不了零。所以只拦【用户显式指定的目标角】。
     *
     * 【不影响谁】J4/J6 的限位是 ±360°，整圈转动完全在范围内，不受影响；
     * MoveL 与回零不走这条路径（MoveL 的限位由 line_solve 的限位过滤负责）。 */
    for (i = 0; i < num_joints; i++) {
        double lmin = 0.0, lmax = 0.0;
        int r = robot_angle_in_soft_limit(joints[i], angles[i], &lmin, &lmax);
        if (r < 0) {
            printf("[错误] 关节号 %d 非法（应为 1..6），已拒绝下发\n", joints[i]);
            return 0;
        }
        if (r == 0) {
            printf("[错误] 关节%d 目标 %.2f° 超出软限位 [%.0f, %.0f]°，已拒绝下发。\n"
                   "       越限位的目标必然让电机一路顶到机械极限并触发堵转/过流，\n"
                   "       是一次纯粹的无效冲撞。请改到限位内的角度。\n"
                   "       （限位值在 config/robot_config.h 的 ROBOT_JOINT_LIMIT_MIN/MAX_DEG）\n",
                   joints[i], angles[i], lmin, lmax);
            return 0;
        }
    }

    for (i = 0; i < num_joints; i++) {
        double max_steps;
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        max_steps = (double)DEG2STEPS(movl_max_step_deg(), reductions[j - 1]);
        if (max_steps < 0) max_steps = -max_steps;
        if (dist[j] > max_steps) {
            printf("[错误] 关节%d 本次位移 %.1f° 超过单步上限 %.0f°，已拒绝下发。\n"
                   "       目标离当前位置这么远，通常是单位/符号/坐标系搞错了，\n"
                   "       真发出去就是电机猛冲 + 超时急停。确认无误就调大\n"
                   "       ini [safety] max_step_deg（当前 %.0f）。\n",
                   j, dist[j] / (double)DEG2STEPS(1.0, reductions[j - 1]),
                   movl_max_step_deg(), movl_max_step_deg());
            return 0;
        }
    }

    /* 先只算不写：把六轴转速定下来 */
    double spd[7] = {0};
    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        if (max_dist > 0) {
            spd[j] = dist[j] / max_dist * speed;
            if (spd[j] < MOVEJ_MIN_RPM) spd[j] = MOVEJ_MIN_RPM;  /* 见 MOVEJ_MIN_RPM 说明，勿抬高 */
        } else {
            spd[j] = speed;
        }
    }

    /* 分遍下发，而不是"每轴 profile+speed+pos 一把梭"。
     * 六轴只能顺序下发，真正让电机起步的是最后那次 0x00E8 写，
     * 所以【六轴的启动时刻间隔】= 相邻两次 0x00E8 之间的帧数 × 单帧耗时：
     *   一把梭（p,s,pos 紧跟）  ：轴 j 的 0x00E8 在第 3j-1 帧（首段）/ 2j-1 帧
     *                            ⇒ 首末轴相隔 15 帧 / 10 帧
     *   分遍（profile 遍→speed 遍→pos 遍）：0x00E8 集中在最后 6 帧
     *                            ⇒ 首末轴相隔 5 帧，同步性提升 3 倍 / 2 倍
     * 现场依据：末端偏差与"节拍占段时长的比例"强相关（48%→0.10mm，72%→2.58mm），
     * 而这个比例的分子就是首末轴启动间隔。改动零成本、不改变任何运动学语义。 */
    if (set_profile) {
        for (i = 0; i < num_joints; i++) {
            j = joints[i];
            if (robot_is_masked(robot, j)) continue;
            g_tx_written++;
            if (motor_set_profile(robot, j, accel_ms, decel_ms) != ERR_NONE)
                printf("[警告] 关节%d 加减速设置失败\n", j);
        }
    }
    /* 速度寄存器每个节拍都写是很贵的一笔：一轮 6 轴要 6 个事务，和位置写一样多。
     * 但同一条 MoveL 里相邻节拍的指令转速几乎不变（同一段时间表、同样比例分配），
     * 变化 <2% 就跳过不写 ⇒ stream 单节拍事务数 12 → 6，节拍耗时和"首末轴启动
     * 间隔"都减半。set_profile=1（本段首拍）时强制重写：此时驱动器可能刚被
     * 重新上电/使能，缓存值不可信。 */
    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        {
            double s = spd[j];
            double ref = fabs(last_spd[j]);
            if (!set_profile && last_spd_ok[j] &&
                fabs(s - last_spd[j]) <= 0.02 * ((ref > 1e-9) ? ref : 1.0))
                continue;                       /* 变化 <2%，跳过本轴速度写 */
            g_tx_written++;
            if (motor_set_speed(robot, j, s) != ERR_NONE)
                printf("[警告] 关节%d 速度设置失败\n", j);
            else {
                last_spd[j] = s;
                last_spd_ok[j] = 1;
            }
        }
    }
    for (i = 0; i < num_joints; i++) {
        j = joints[i];
        if (robot_is_masked(robot, j)) continue;
        g_tx_written++;
        if (motor_move_abs(robot, j, tgt[j]) != ERR_NONE) {
            printf("[警告] 关节%d 多关节运动发指令失败\n", j);
            continue;
        }
        pend[j] = 1;
        remain++;
    }

    /* UDP 遥测：把本次下发的目标角发给本机 3D 镜像，同时作为"实际角"的基准。
     * 【只在六轴齐全时发】num_joints<6 时 angles 里没被指定的轴是上一次的残值，
     * 发出去会让镜像摆出一个根本不存在的姿态。六轴下发（movel 各段、多轴 movej）
     * 才是完整位姿，其余情况宁可不发。
     * 这一句在最热的路径上，但 sendto 回环是微秒级且失败静默，不影响控制。 */
    if (num_joints == 6) {
        for (i = 0; i < 6; i++) g_tlm_mech[i] = angles[i];
        g_tlm_mech_ok = 1;
        telemetry_send(angles, "cmd");
    }

    return remain;
}

/* 全程偏差峰值累加器：movej_wait 只更新、不打印；由 cmd_movel 在整条 MoveL
 * 结束时汇总成一行。此前是"每段打印一行本段实测最大偏差"，50 段的 step 会刷屏。 */
static double g_dev_peak = 0.0;
static int    g_dev_seen = 0;

static void dev_reset(void) { g_dev_peak = 0.0; g_dev_seen = 0; }

static void dev_report(const char *tag)
{
    if (!g_dev_seen) return;
    printf("  %s实测最大偏差 %.2f mm（全程峰值）\n",
           (tag != NULL) ? tag : "", g_dev_peak);
}

/* status_line：\r 刷新的实时状态行。固定宽度 + 尾部填充，避免短行残留上一轮字符。 */
static void status_line(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("\r%-*s", MOVEJ_STATUS_W, buf);
    fflush(stdout);
}

static void status_clear(void)
{
    printf("\r%*s\r", MOVEJ_STATUS_W, "");
    fflush(stdout);
}

/* movej_wait：轮询等待 pend 中轴到位（位置进容差即算到位），超时急停。
 * line_a/line_b 非空时顺便统计末端相对理想直线的实时偏差(mm)，只累加进 g_dev_peak，
 * 不在此处打印。
 *
 * 总线开销：每个轮询只读【一次】六轴位置（同一 0x00?? 位置寄存器既用于到位判定、
 * 又用于偏差统计），共 6 个事务。此前"显示用"读一遍(6) +"到位判定用"再读一遍(6)
 * + 读一次 J1 速度(1) = 13 事务，轮询周期被拖到 ~340ms（现场实测"每节拍 347ms"）。
 * 合并后 ≈160ms，六轴起步不同步的程度直接减半。 */
static void movej_wait(Robot *robot, const int joints[6], const int32_t tgt[7],
                       uint8_t pend[7], int remain,
                       const double *line_a, const double *line_b)
{
    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    uint32_t start_ms = GetTickCount();
    uint32_t last_print = 0;
    int j;

    /* 位置读数跨轮保留：见下方"只读没到位的轴"说明。 */
    int32_t pos[7] = {0};
    int      ok[7] = {0};
    int      anom_n = 0;    /* 连续"读回异常"轮数，见下方读回体检 */

    (void)joints;
    while (remain > 0) {
        if ((GetTickCount() - start_ms) >= MOVEJ_TIMEOUT_MS) {
            const uint16_t red_to[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
            status_clear();   /* 别让超时警告接在 \r 状态行屁股后面 */
            for (j = 1; j <= 6; j++) {
                int pos_ok = 0;
                int32_t pos;
                ErrCode rc;
                if (!pend[j]) continue;
                pos = motor_read_position(robot, j, &pos_ok);
                rc = motor_estop(robot, j);
                printf("[警告] 关节%d 多关节运动超时，已急停%s", j,
                       (rc == ERR_NONE) ? "" : "（急停指令下发失败）");
                if (pos_ok) {
                    double per_deg = (double)DEG2STEPS(1.0, red_to[j - 1]);
                    printf("（还差 %.2f° = %ld 步）",
                           (double)(tgt[j] - pos) / per_deg, (long)(tgt[j] - pos));
                }
                printf("\n");
                pend[j] = 0; remain--;
            }
            break;
        }
        double   motor[6], mech[6];
        double   dev = 0.0;
        /* 只读【还没到位】的轴，已到位的沿用上一轮读数。
         * 旧实现每轮固定读满 6 轴，一轮 6×15ms≈90ms——这个延迟本身就是段间
         * 停顿的一部分（臂早停了，还在等轮询发现它停了）。随着轴陆续到位，
         * 这里从 6 笔递减到 1 笔，轮询周期大幅缩短。
         * 注意偏差统计要用到全部六轴做 FK，所以不能直接置 0，必须保留旧值。 */
        for (j = 1; j <= 6; j++) {
            if (!pend[j]) continue;
            pos[j] = motor_read_position(robot, j, &ok[j]);
        }
        for (j = 0; j < 6; j++)
            motor[j] = ok[j + 1] ? STEPS2DEG(pos[j + 1], red[j]) : 0.0;
        joint_zero_motor_to_mech(motor, mech);

        /* UDP 遥测：发【实际读回】的六轴角，3D 镜像据此连续刷新。
         * 与 movej_issue 里那包的区别：那里发的是【目标角】，一段只有一包，
         * 镜像会一跳一跳；这里跟着轮询走（一轮约 90ms，轴陆续到位后更快），
         * 动作是连续的，而且看到的是真机【真实走到哪了】，包含滞后/跟不上。
         *
         * 【别要求"六轴都在动"】第一版写成 pend[1..6] 全为真才发，结果只要有一
         * 轴已到位/被 mask 就一包都不发（实测整条 movej 只收到 2 包，全是 cmd）。
         * 现在改成：以 movej_issue 存下的目标角为基准，本轮【读到】的轴就更新成
         * 实际值，没读到的沿用上次 —— 这样六角永远是完整的，姿态不会凭空乱摆，
         * 而且部分轴不动时也能连续发。ok[j]==0 的轴不能用（motor=0 ⇒ mech 是
         * 0 机械角，是假值），所以必须逐个判断 ok。 */
        if (g_tlm_mech_ok) {
            for (j = 0; j < 6; j++)
                if (ok[j + 1]) g_tlm_mech[j] = mech[j];
            telemetry_send(g_tlm_mech, "act");
        }

        /* 【读回体检：读回不可能是真的 ⇒ 立刻停，别干等 60s】
         *
         * 2026-09-19 事故：六轴读回同时变垃圾（J5 读成 -115.44，越软限位 20°），
         * 到位判定永远不满足，于是循环干等到 MOVEJ_TIMEOUT_MS 才罢手，而真超时后
         * 急停指令还发不出去 —— 整整一分钟完全失控（清单 #14）。
         *
         * 现在每轮都查一次"这组读数有没有可能是真的"，连续 MOVEJ_ANOM_MIN_POLLS
         * 轮都为异常就立刻中止并尽力急停。要求连续多轮是为了滤掉单次干扰。
         *
         * 【为什么用 ok[] 过滤】读失败的轴上面被填成 0，而 0 对 J3（限位 30~180）
         * 等于"越限 30°"—— 不过滤会把一次普通读失败当成总线异常。 */
        {
            int ok6[6], bad_j = 0;
            double bad_ex = 0.0;
            for (j = 0; j < 6; j++) ok6[j] = ok[j + 1];
            if (robot_readback_anomaly(mech, ok6, MOVEJ_ANOM_MARGIN_DEG,
                                       &bad_j, &bad_ex)) {
                if (++anom_n >= MOVEJ_ANOM_MIN_POLLS) {
                    status_clear();
                    printf("[危险] 关节%d 读回 %.2f° 越软限位 %.1f°，且连续 %d 轮如此 ——\n"
                           "       这不是真实位置，是总线/读回异常（2026-09-19 实测：六轴\n"
                           "       读回同时变成越限值，末端偏差冻结在 258.92mm 不动）。\n"
                           "       不再等 %d ms 超时，立即中止并尽力急停。\n"
                           "       【请检查】USB-RS485 适配器与驱动器供电，别急着改软件。\n",
                           bad_j, mech[bad_j - 1], bad_ex, anom_n, MOVEJ_TIMEOUT_MS);
                    for (j = 1; j <= 6; j++) {
                        ErrCode rc;
                        if (!pend[j]) continue;
                        rc = motor_estop(robot, j);
                        if (rc != ERR_NONE) {
                            printf("[警告] 关节%d 急停指令下发失败（总线确实不通了）\n", j);
                        }
                        pend[j] = 0; remain--;
                    }
                    break;
                }
            } else {
                anom_n = 0;
            }
        }

        if (line_a != NULL && line_b != NULL) {
            double m[4][4], xyz[3];
            dh_forward(DH_TABLE, mech, m);
            for (j = 0; j < 3; j++) xyz[j] = m[j][3];
            dev = movl_point_dev_mm(xyz, line_a, line_b);
            if (!g_dev_seen || dev > g_dev_peak) g_dev_peak = dev;
            g_dev_seen = 1;
        }
        for (j = 1; j <= 6; j++) {
            if (!pend[j]) continue;
            if (ok[j] && pos[j] >= tgt[j] - MOVEJ_INPOS_TOL &&
                pos[j] <= tgt[j] + MOVEJ_INPOS_TOL) {
                pend[j] = 0; remain--;
            }
        }
        if ((GetTickCount() - last_print) >= MOVEJ_STATUS_MS || remain == 0) {
            last_print = GetTickCount();
            if (line_a != NULL && line_b != NULL)
                status_line("J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f  偏差%6.2f mm",
                            mech[0], mech[1], mech[2], mech[3], mech[4], mech[5], dev);
            else
                status_line("J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f",
                            mech[0], mech[1], mech[2], mech[3], mech[4], mech[5]);
        }
        if (remain > 0) Sleep(MOVEJ_POLL_MS);
    }
    status_clear();
}

/* movl_dev_sample：读一轮六轴位置 → 正解 → 累加全程偏差峰值。
 *
 * 为什么单独抽出来：stream 模式是"下发完所有段才统一等到位"，它的 movej_wait
 * 只在【最后一段下发之后】跑一次，采样窗口只覆盖末尾那一小截。结果就是
 * 打出"实测最大偏差 0.00 mm"——不是真的直，是几乎没量东西，而且和 sync 的
 * "全程峰值"根本不可比。所以要在每个节拍下发完的空闲窗口里补采样。
 *
 * 读失败就整轮放弃，不拿残缺的六轴角去算正解（算出来的点是假的）。 */
static void movl_dev_sample(Robot *robot, const double *line_a, const double *line_b)
{
    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
    double motor[6], mech[6], m[4][4], xyz[3];
    int32_t pos[7];
    int ok[7], j;

    if (line_a == NULL || line_b == NULL) return;
    for (j = 1; j <= 6; j++) {
        pos[j] = motor_read_position(robot, j, &ok[j]);
        if (!ok[j]) return;
    }
    for (j = 0; j < 6; j++) motor[j] = STEPS2DEG(pos[j + 1], red[j]);
    joint_zero_motor_to_mech(motor, mech);
    dh_forward(DH_TABLE, mech, m);
    for (j = 0; j < 3; j++) xyz[j] = m[j][3];
    {
        double dev = movl_point_dev_mm(xyz, line_a, line_b);
        if (!g_dev_seen || dev > g_dev_peak) g_dev_peak = dev;
        g_dev_seen = 1;
    }
}

/* movej_joints：多关节同时运动完整流程 = 下发(读实际位置) + 等待同步到位 */
static void movej_joints(Robot *robot, int num_joints, const int joints[6],
                         const double angles[6], double speed,
                         int accel_ms, int decel_ms,
                         const double *line_a, const double *line_b)
{
    int32_t tgt[7] = {0};
    uint8_t pend[7] = {0};
    int remain = movej_issue(robot, num_joints, joints, angles, NULL,
                             speed, accel_ms, decel_ms, 1, tgt, pend);
    if (remain <= 0) return;
    movej_wait(robot, joints, tgt, pend, remain, line_a, line_b);
}

/* movej_issue_noread：六轴位置【连发】——只写不等响应，每轴之间留 MOVEJ_NR_GAP_MS。
 *
 * 【⚠️ 2026-09-18 实测推翻了下面这段原本的动机，保留原文以便对照】
 * 为什么（原以为）值得这么做：等响应的写法每轴要 15.4ms（其中等响应 14.85ms，
 * 线上只占 1.8ms），六轴一轮 92ms ⇒ 六轴起步相差约 75ms，以为这是画直线画歪的主因。
 *
 * 【实测结果：连发不仅没用，反而更差，默认已关闭（noread_gap_ms = 0）】
 * 修好按段速度归一化（movl_set_seg_speed）之后做的 A/B/A（每轮 4 趟 50mm）：
 *     gap=4 连发    0.31 0.32 0.20 0.19 | 0.31 0.18 0.27 0.20 ⇒ 均值 0.25 mm
 *     gap=0 等响应  0.17 0.14 0.18 0.19                        ⇒ 均值 0.17 mm
 * 等响应更准 32%，且不担撞帧风险 ⇒ 六轴启动差根本不是主因，主因一直是速度失配。
 *
 * 为什么必须留帧间延迟：从站【照样会回一帧】，半双工下它会和我们的下一帧撞车。
 * nrtest 实测 0/1/2ms 都会读失败 1/6，3ms 起干净，这里取 4ms 留余量。
 *
 * 安全性：帧真被撞坏时该轴收不到命令 ⇒ movej_wait 等不到到位 ⇒ 触发超时急停，
 * 不会"跑飞"。代价是最坏多等 MOVEJ_TIMEOUT_MS。
 *
 * gap_ms <= 0 时【回退】成每轴等响应的旧做法（用于 A/B 对比）。 */
static void movej_issue_noread(Robot *robot, const int joints[6], int32_t tgt[7],
                               int gap_ms)
{
    int j;
    for (j = 0; j < 6; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        if (gap_ms > 0) {
            motor_move_abs_noread(robot, joints[j], tgt[joints[j]]);
            Sleep((DWORD)gap_ms);
        } else {
            motor_move_abs(robot, joints[j], tgt[joints[j]]);
        }
    }
}

/* movl_set_seg_speed：按【本段】的每轴位移重新归一化速度，让六轴同时到位。
 *
 * 【为什么必须按段算，不能像旧实现那样整段算一次】
 * 笛卡尔直线上，各轴的速率比沿途是【变】的。实测 50mm 线（2 段）：
 *   第 1 段：J1 走 9.94°、J2 只走 0.97°  ⇒ 需要速率比 0.098
 *   整段比：J2 总位移 3.16° / J1 总位移 19.52° = 0.162
 * 旧实现用整段比 0.162 给 J2 定速 ⇒ J2/J3/J5 被给快了约 1.7 倍，
 * 于是它们【提前到位并停住】，剩下的行程只由 J1/J6 单独走完，路径被甩成弓形。
 * 现场能直接看到：偏差在每个航点处回落到 ~0.05mm，段中鼓到 1.9mm。
 *
 * 按段归一化后，段内六轴按比例同时到达 ⇒ 段内就是纯关节空间弦，
 * 偏差应当退回【几何弓高】量级（50mm 2 段预测 0.54mm，而不是实测的 1.9mm）。
 *
 * red 为 0 基的减速比表（steps/deg），与 cmd_movel 里 DEG2STEPS 用的一致。 */
static void movl_set_seg_speed(Robot *robot, const int joints[6],
                               const double qa[6], const double qb[6],
                               const uint16_t red[6], double seg_rpm)
{
    double seg[7] = {0}, smax = 0.0, spd;
    int j;

    for (j = 0; j < 6; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        seg[joints[j]] = fabs(qb[j] - qa[j]) * (double)red[j];
        if (seg[joints[j]] > smax) smax = seg[joints[j]];
    }
    if (smax <= 0.0) return;          /* 本段无位移，保持原速 */
    for (j = 0; j < 6; j++) {
        if (robot_is_masked(robot, joints[j])) continue;
        spd = seg[joints[j]] / smax * seg_rpm;
        if (spd < MOVEJ_MIN_RPM) spd = MOVEJ_MIN_RPM;
        motor_set_speed(robot, joints[j], spd);
    }
}

/* movej_multi：多关节同时运动（从 ParsedCmd 调用） */
static void movej_multi(Robot *robot, const ParsedCmd *cmd)
{
    int joints[6], idx = 0;
    double angles[6];
    int acc, dec;
    for (int i = 0; i < cmd->num_joints; i++) {
        joints[idx] = cmd->joints[i];
        angles[idx] = cmd->angles[i];
        idx++;
    }
    /* movej 此前完全没有加减速兜底，输多小就下发多小；与 movel 统一按下限取 max */
    acc = (cmd->accel_ms[0] >= MOVL_ACC_FLOOR_MS) ? cmd->accel_ms[0] : MOVL_ACC_FLOOR_MS;
    dec = (cmd->decel_ms[0] >= MOVL_ACC_FLOOR_MS) ? cmd->decel_ms[0] : MOVL_ACC_FLOOR_MS;
    if ((cmd->accel_ms[0] > 0 && cmd->accel_ms[0] < MOVL_ACC_FLOOR_MS) ||
        (cmd->decel_ms[0] > 0 && cmd->decel_ms[0] < MOVL_ACC_FLOOR_MS))
        printf("[提示] 加/减速 %d/%d ms 低于安全下限，已抬到 %d/%d ms\n",
               cmd->accel_ms[0], cmd->decel_ms[0], acc, dec);
    movej_joints(robot, cmd->num_joints, joints, angles,
                 cmd->speeds[0], acc, dec, NULL, NULL);
}

/* movl_noread_gap_ms：六轴位置连发的帧间延迟(ms)，从 ini [movel] noread_gap_ms 读。
 * 返回 0 表示【禁用】连发，回退到"每轴等响应"的旧做法。
 * 用 ini 而不是写死，是为了能直接做 A/B 对比：同一个段数、同一条线，
 * 改一次 ini 重启就能看出连发到底有没有收益。 */
static int movl_noread_gap_ms(void)
{
    FILE *f;
    char line[256];
    int in_movel = 0;

    f = fopen(INI_PATH, "r");
    if (f == NULL) return MOVEJ_NR_GAP_MS;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_movel = (strncmp(p, "[movel]", 7) == 0) ? 1 : 0;
            continue;
        }
        if (!in_movel) continue;
        if (strncmp(p, "noread_gap_ms", 13) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            int v = atoi(eq + 1);
            return (v >= 0) ? v : MOVEJ_NR_GAP_MS;
        }
    }
    fclose(f);
    return MOVEJ_NR_GAP_MS;
}

/* movl_stall_thresholds：取六轴堵转阈值(mA)。
 *
 * 【为什么逐轴】旧代码是 double stall = ROBOT_STALL_CURRENT_MA（全局单值），
 * 六轴机座大小不同、额定电流能差好几倍，一个值必然要么大轴一动就误报、
 * 要么小轴撞死都不报。
 *
 * 【取值优先级】运行时（Monitor） > ini > 编译期默认。
 *
 * 【2026-09-18 修】原来这里只读 ini，完全无视运行时用 stall:N:MA 改过的值，
 * 造成两个后果：
 *   ① MoveL 打印谎报 —— 明明已经 stall:1:0 关掉了，仍打印"过流保护 开启"；
 *   ② 更糟：临时改的阈值对 MoveL **根本不生效**。想临时压低阈值让保护更
 *      灵敏（比如换新工件时调到 300mA），MoveL 照样用 ini 的老值；
 *      反过来想临时关掉做对比测试，也关不掉 —— A/B 测出来的"两组一样"
 *      其实是假象，两组都还在保护。
 * 现在改为优先从 Monitor 读（cmd_stall 改的就是它），没有监控器才回退 ini。 */
static void movl_stall_thresholds(int th[6])
{
    const int def[ROBOT_JOINT_COUNT] = ROBOT_STALL_CURRENT_MA_TABLE;
    int i;

    if (g_mon != NULL) {
        for (i = 0; i < ROBOT_JOINT_COUNT; i++)
            th[i] = monitor_get_stall_threshold(g_mon, i + 1);
        return;
    }
    if (ini_read_stall_current(INI_PATH, th)) return;
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) th[i] = def[i];
}

/* movl_stall_on：是否至少有一轴配了阈值（>0）⇒ 过流保护生效 */
static int movl_stall_on(const int th[6])
{
    int i;
    for (i = 0; i < ROBOT_JOINT_COUNT; i++) {
        if (th[i] > 0) return 1;
    }
    return 0;
}

/* movl_stall_guard：逐轴过流检查 —— 返回 0=正常 / 1=已急停（调用方须立刻 return）。
 *
 * 【为什么必须有它 —— 2026-09-18】
 * 运动命令期间后台巡检是被 monitor_pause_active(1) 挂起的（它要吃掉六成总线带宽，
 * 不挂起会把控制周期拖到 6Hz）。那段挂起代码的注释写着"堵转检测本就没开，
 * 挂起没有安全损失"——**阈值一旦配上，这个前提就不成立了**：
 * 运动期间恰恰是最需要碰撞保护的时刻，而巡检此时一个字节都不上总线。
 * 运动期间的保护只能由运动循环自己来，这就是本函数。
 *
 * 【判据】每轴跟【自己的】阈值比，取"超得最狠"的比值决策：
 *   ratio_j = 电流_j / 阈值_j；≥1.0 ⇒ 急停；>0.6 ⇒ 按比例降速。
 * 未配阈值(≤0)的轴、屏蔽轴、读失败的轴一律跳过 —— 宁可漏报也不误报。
 *
 * 【覆盖不到的情况（如实告知）】只在"每段下发前"检查，所以
 * ① 整段一次下发（不分段）的短线段，运动期间不检查；
 * ② 段内等待到位的那段时间不检查（检查会拉长轮询周期、恶化段间停顿）。
 * 静止时由后台巡检（300ms 周期）兜底。 */
static int movl_stall_guard(Robot *robot, const int th[6])
{
    double worst = 0.0;
    int worst_j = 0, worst_cur = 0;
    int j;

    if (!movl_stall_on(th)) return 0;

    for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
        int cur;
        double r;
        if (th[j] <= 0) continue;
        if (robot_is_masked(robot, j + 1)) continue;
        cur = robot_read_current_ma(robot, j + 1);
        if (cur <= 0) continue;                 /* 读失败不参与判定 */
        r = (double)cur / (double)th[j];
        if (r > worst) { worst = r; worst_j = j + 1; worst_cur = cur; }
    }
    if (worst >= 1.0) {
        printf("[过流保护] 关节%d 实时电流 %d mA 超阈值 %d mA（%.0f%%），急停全部关节\n",
               worst_j, worst_cur, th[worst_j - 1], worst * 100.0);
        for (j = 0; j < ROBOT_JOINT_COUNT; j++)
            if (!robot_is_masked(robot, j + 1)) motor_estop(robot, j + 1);
        return 1;
    }
    return 0;
}

/* movl_plan：按给定弦步长做直线离散 + 逐点 IK + 分段时间表（可被 stream 重规划复用） */
static int movl_plan(const double start_pose[6], const double end_pose[6],
                     const double q_start[6], const JointLimit *limits,
                     double dist_mm, double step_mm, const double vmax[6],
                     double q_seq[LINE_MAX_POINTS][6], double seg_dt[LINE_MAX_SEGS],
                     int *out_count, double *out_total_dt, int *out_fail_idx,
                     char *fail_reason)
{
    LinePath path;
    int count = line_count_for_distance(dist_mm, step_mm);
    int i;

    if (line_plan(start_pose, end_pose, count, &path) != 0) return -1;
    if (line_solve(&path, DH_TABLE, limits, q_start, q_seq, out_fail_idx, fail_reason) != 0) return -2;
    if (line_time_table(q_seq, count, vmax, seg_dt, out_total_dt) != 0) return -3;
    *out_count = count;

    /* ---------- 规划体检测（两条，都是"甩臂"的前兆）---------- */
    int warn_j = 0, warn_seg = 0;
    double warn_deg;

    /* 体检①：相邻插补点之间的单关节跳变。
     * 这是最通用的"甩臂"前兆 —— 不管成因是奇异、分支翻转还是选型跳变，
     * 最终都表现为"一小段笛卡尔位移需要某个关节转一大圈"。
     *
     * 【2026-09-18 用户拍板：从"只告警"改成"超阈值直接拒绝"】
     * 原先的考虑是"末端位姿仍然精确，是否接受由用户判断"，但实测下来这个
     * 自由度没有价值：真到了 90° 那一档，段时长只有几毫秒，关节根本跑不完，
     * 结果不是"走得凶"，而是臂以一种你没预料到的姿态扫过去（实测：该段被
     * 时间表按关节限速拉长到 12.5 s，期间末端只挪 1mm、J4 却转过 89.97°）。
     * 留着让用户选，等于把撞机风险当成选项。
     * 现在两档：>15° 只告警（提示，仍执行）；>30° 拒绝整条 MoveL。
     *
     * 【实测】home 位形（J5=0，腕部奇异）出发沿 +Y 走 30mm：
     * 第 1 段就要 J4 转 89.98° —— θ5 一离开 0，θ4 就被位姿唯一锁死，
     * 这是奇异位形的物理本质，IK 层无法消掉，只能绕开（先用 MoveJ 挪开 J5）。
     *
     * 【为什么用 line_max_joint_jump】判据抽成了 trajectory/line.c 里的纯函数，
     * 这样它能被离线单测覆盖 —— 这段逻辑原本埋在 CLI 里，没法测。 */
    warn_deg = line_max_joint_jump(q_seq, count, &warn_j, &warn_seg);

    if (warn_deg > movl_max_jump_deg()) {
        printf("[拒绝] 规划路径第 %d/%d 段：关节%d 单段要转 %.1f°,\n"
               "       超过单段跳变上限 %.0f°（ini [safety] max_jump_deg）⇒ 整条 MoveL 不下发。\n"
               "       【实测后果】时间表按关节限速自动把这一段拉长到十几秒 ——\n"
               "       这段时间里末端只挪 1mm，而 J4 转过 90°：臂会以一种你\n"
               "       完全没预料到的大幅度慢慢扫过去，肘部/法兰扫过一大片空间。\n"
               "       不是「猛冲」，但撞到周围东西的概率极高，而且看起来根本不像直线。\n"
               "       成因通常是路径擦过腕部奇异位形（J5≈0）或逆解分支翻转。\n"
               "       【怎么绕开】先用 MoveJ:J1..J6 把姿态挪开（尤其把 J5 移出 ±5°），\n"
               "       再从新姿态走笛卡尔直线。确认要强走就把 max_jump_deg 调大,\n"
               "       但那等于自己承担甩臂风险。\n",
               warn_seg, count, warn_j, warn_deg, movl_max_jump_deg());
        return -4;
    }
    if (warn_deg > MOVL_JUMP_WARN_DEG) {
        printf("[警告] 规划路径第 %d/%d 段：关节%d 单段要转 %.1f°（告警阈值 %.0f°）。\n"
               "       末端位姿是精确的，但关节会走得很凶 —— 时间表会把这一段\n"
               "       按关节限速拉长，期间末端几乎不动而某个关节转过一大片角度，\n"
               "       臂会以一种你没预料到的姿态扫过去。留意周围有没有东西可撞。\n"
               "       成因通常是路径擦过奇异位形/逆解分支翻转,\n"
               "       建议先用 MoveJ:J1..J6 把姿态挪开再走笛卡尔直线。\n",
               warn_seg, count, warn_j, warn_deg, MOVL_JUMP_WARN_DEG);
    }

    /* 体检②：路径是否进入腕部奇异区（θ5≈0）。
     * 上面那条只报现象，这条报【成因】，让用户知道该动哪个轴。 */
    for (i = 0; i < count; i++) {
        if (fabs(q_seq[i][4]) < MOVL_WRIST_SINGULAR_DEG) {
            printf("[警告] 规划路径第 %d/%d 点进入腕部奇异区（J5 = %.2f° ≈ 0）：\n"
                   "       此处 θ4 与 θ6 同轴、关节分配不唯一。IK 已做退化处理\n"
                   "       （保留真实 θ5、θ4 锚定上一点），末端位姿仍然精确；\n"
                   "       但离开奇异点时 θ4 会被位姿唯一锁死，可能要求 J4 大角度转动。\n"
                   "       【实测】home 位形出发沿 +Y 走 30mm：第 1 段就要 J4 转 89.98°。\n"
                   "       建议先用 MoveJ:J1..J6 把 J5 移出 ±%.0f° 再做笛卡尔直线。\n"
                   "       注：2026-09-18 那次「末端偏差 34mm、臂乱甩」的真因不是它，\n"
                   "       而是万向锁处 asin 越界产生 NaN（见 test_line_nan.c），已修。\n",
                   i, count, q_seq[i][4], MOVL_WRIST_SINGULAR_DEG);
            break;
        }
    }
    return 0;
}

/* movl_bow_mm：单发同步模式下，关节空间线性插补相对笛卡尔直线 AB 的最大偏差(弓高, mm)。
 * 驱动器只对终点做插补，中间走的是关节空间直线，故偏离规划直线，偏离量随行程平方增长。 */
static double movl_bow_mm(const double q0[6], const double q1[6],
                          const double a[3], const double b[3])
{
    double ab[3], ab2 = 0.0, worst = 0.0;
    int k, i;

    for (i = 0; i < 3; i++) { ab[i] = b[i] - a[i]; ab2 += ab[i] * ab[i]; }
    if (ab2 < 1e-9) return 0.0;

    for (k = 1; k < MOVL_BOW_SAMPLES; k++) {
        double s = (double)k / (double)MOVL_BOW_SAMPLES;
        double q[6], m[4][4], t = 0.0, d2 = 0.0;
        for (i = 0; i < 6; i++) q[i] = q0[i] + s * (q1[i] - q0[i]);
        dh_forward(DH_TABLE, q, m);
        for (i = 0; i < 3; i++) {
            double v = m[i][3] - a[i];
            t += v * ab[i];
        }
        t /= ab2;
        if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
        for (i = 0; i < 3; i++) {
            double e = (m[i][3] - a[i]) - t * ab[i];
            d2 += e * e;
        }
        if (d2 > worst) worst = d2;
    }
    return sqrt(worst);
}

/* ====================== moveL：笛卡尔真直线插补 ======================
 * 实现：起点FK → line_plan(位置线性+姿态SLERP) → line_solve(逐点IK+分支连续)
 *       → line_time_table(按关节限速生成等间隔分段时间表) → 逐段 movej_joints 下发。
 * 末端走空间直线（密集弦逼近），而非单终点 IK 的弧线。
 * P2 电流自适应限速：逐轴阈值（ini [stall] j1..j6）下，某轴实时电流逼近自己的
 *       阈值则按比例降速，超阈值则急停全部关节（过流=降速/限流，非加力）；
 *       六轴阈值全为 0 时整段保护关闭（当前默认，量完 curtest 再填）。
 * step 模式：逐段下发并等待六轴到位后再发下一段，段间有停顿但绝不过冲（真直线）。
 * stream 模式：按分段时间表【周期刷新】——段间不等待到位，加减速只在首段写一次，
 *       每个节拍只写速度+绝对位置（6轴×2事务，省去读位置与重复写 profile），
 *       末端走完最后一段后才统一等待真正到位。轨迹连续无段间停顿，
 *       代价是轨迹整体滞后于规划时刻（轴未到位即被下一目标覆盖），
 *       故 stream 依赖节拍 ≥ 总线耗时，开启 P2 电流轮询会拉长每节拍、需放宽节拍。
 * sync 模式（默认）：仍用密集弦做整条路径的可达性/限位校验，但【只下发终点】一次，
 *       六轴按行程比例分配转速同起同停，全程一次加减速，无段间停顿。
 *       代价是段内走关节空间直线 ⇒ 偏离笛卡尔直线（弓高），已按行程估算并打印。
 *       P2 电流自适应限速依赖分段，sync 下不生效。 */

/* movl_max_step_deg：单次下发的最大位移(机械角，度)，从 ini [safety] max_step_deg 读，
 * 读不到用 MOVEJ_MAX_STEP_DEG。这是下发前的【相对位移闸门】阈值 —— 见 movej_issue。
 *
 * 【为什么由用户定】它决定"多远算离谱"，取决于你怎么用这台臂：
 * 只做小范围画图的可以设到 30° 挡得更死；要整圈转 J4/J6 的就得放宽。
 * 代码只提供默认值与实测依据，不替用户拍板。
 *
 * 【默认值 720° 的依据】关节软限位最宽的是 J4/J6（±360°，全宽 720°）。
 * 合法运动的起点终点都在限位内 ⇒ 单次位移不会超过全宽。取全宽恰好让
 * "任何限位内的点到另一个限位内的点"都放行，同时把 3.28 亿步
 * （= 约 118000°）这类逻辑错误死死挡住。 */
static double movl_max_step_deg(void)
{
    double v;
    if (ini_read_max_step_deg(INI_PATH, &v)) return v;
    return MOVEJ_MAX_STEP_DEG;
}

/* movl_max_jump_deg：MoveL 规划层允许的单段关节跳变上限(度)，
 * 从 ini [safety] max_jump_deg 读，读不到用 MOVL_JUMP_MAX_DEG。
 * 超过就【拒绝整条 MoveL】—— 见 movl_plan 返回值 -4 处。
 * 默认 30° 的依据见 MOVL_JUMP_MAX_DEG 的注释（正常 <1° vs 病态 89.98°）。 */
static double movl_max_jump_deg(void)
{
    double v;
    if (ini_read_max_jump_deg(INI_PATH, &v)) return v;
    return MOVL_JUMP_MAX_DEG;
}

/* movl_bow_budget：弓高预算(mm)，从 ini [movel] bow_mm 读，读不到用 MOVL_SYNC_BOW_MM。
 *
 * 为什么要可配置：这是"直"与"顺"的权衡旋钮——调大则段少、顺、偏离直线多，
 * 调小则段多、直、段间停顿多（一顿一顿）。每个人能接受的精度不一样，
 * 画长线段和短线段时的选择也不一样。写死在代码里，每次试都要重新编译，太笨。
 * 另外 smooth 模式可以完全不分段（流畅优先）。 */
static double movl_bow_budget(void)
{
    FILE *f;
    char line[256];
    int in_movel = 0;

    f = fopen(INI_PATH, "r");
    if (f == NULL) {
        return MOVL_SYNC_BOW_MM;
    }
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_movel = (strncmp(p, "[movel]", 7) == 0) ? 1 : 0;
            continue;
        }
        if (!in_movel) continue;
        if (strncmp(p, "bow_mm", 6) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            double v = atof(eq + 1);
            return (v > 0.0) ? v : MOVL_SYNC_BOW_MM;
        }
    }
    fclose(f);
    return MOVL_SYNC_BOW_MM;
}

/* ang_delta_deg：两角之差，归一到 (-180, 180]。
 * 姿态角是圆周量：179.9° 与 -179.9° 实际只差 0.2°，直接相减会得出 359.8°。 */
static double ang_delta_deg(double a, double b)
{
    double d = fmod(a - b, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

/* movl_tip_warn_deg：姿态偏差告警阈值(度)，ini [movel] tip_warn_deg 可覆盖。
 * 默认 MOVL_TIP_WARN_DEG = 5.0，依据见该宏注释（41.17mm 笔在 5° 下横移 3.6mm）。 */
static double movl_tip_warn_deg(void)
{
    FILE *f;
    char line[256];
    int in_movel = 0;

    f = fopen(INI_PATH, "r");
    if (f == NULL) return MOVL_TIP_WARN_DEG;
    while (fgets(line, sizeof(line), f) != NULL) {
        char *p = line;
        char *eq;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == ';' || *p == '#' || *p == '\n' || *p == '\r' || *p == '\0') continue;
        if (*p == '[') {
            in_movel = (strncmp(p, "[movel]", 7) == 0) ? 1 : 0;
            continue;
        }
        if (!in_movel) continue;
        if (strncmp(p, "tip_warn_deg", 12) != 0) continue;
        eq = strchr(p, '=');
        if (eq == NULL) continue;
        fclose(f);
        {
            double v = atof(eq + 1);
            return (v > 0.0) ? v : MOVL_TIP_WARN_DEG;
        }
    }
    fclose(f);
    return MOVL_TIP_WARN_DEG;
}

/* movl_pose_warn：目标姿态与【当前】姿态不一致时，下发前告警。
 *
 * 【为什么必须在下发前拦这一下 —— 2026-09-19 真机事故】
 * 用户报"画的是斜线、落点不对"。离线复算【法兰】轨迹却是完美直线
 * （80mm 线直线度 0.0000mm）—— 只看法兰坐标永远发现不了问题。
 * 真因：moveL 的第 4~6 个数（Rx,Ry,Rz）抄成了【别的姿态】下的值
 * （home 姿态 getpos 打印 Ry=89.99，绘图位姿实际是 0）。
 * line_plan 用 SLERP 把姿态从起点一路拧到目标，中途姿态持续变化，
 * 装在法兰下方 L mm 处的笔尖就绕法兰摆 L×2sin(θ/2)（θ = 法兰倾角变化）。
 *
 * 【离线实测：80mm Y 向线，起点 J=0,0,110,0,70,0，笔长 41.17】
 *   填 -180,0,-180（=起点）⇒ 倾角变化 0.000°  笔尖纸面偏离 0.0000 mm
 *   填 115,90,115（home 抄的）⇒ 倾角变化 88.9° 笔尖纸面偏离 7.71 mm
 * ⇒ 差 7.7mm，肉眼一眼就看出线是斜的，而法兰坐标完全正常。
 *
 * 【为什么只告警不拒绝】姿态变化本身不撞机（撞机由 max_jump_deg 管），
 * 它只毁【画出来的线的质量】。确有正当用途（比如边走边转姿态），
 * 所以这里给足信息让用户自己判断，不替他做决定。 */
static void movl_pose_warn(const double start_pose[6], const double end_pose[6])
{
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double pen_mm = 0.0;
    double drx, dry, drz, worst, warn;
    double m0[4][4], m1[4][4];
    double n0[3], n1[3], dot, tilt, swing;

    /* 笔长：优先 [tool] pen_length（只用于告警，不改坐标）；
     * 没配就回退 tool_length（那时 TCP 真在笔尖，用它也对）；都没有按 0：只给角度。 */
    if (!ini_read_pen_length(INI_PATH, &pen_mm) || pen_mm <= 0.0) {
        pen_mm = 0.0;
        if (ini_read_tool_length(INI_PATH, &pen_mm) && pen_mm <= 0.0) pen_mm = 0.0;
    }

    drx = fabs(ang_delta_deg(end_pose[3], start_pose[3]));
    dry = fabs(ang_delta_deg(end_pose[4], start_pose[4]));
    drz = fabs(ang_delta_deg(end_pose[5], start_pose[5]));
    worst = drx;
    if (dry > worst) worst = dry;
    if (drz > worst) worst = drz;
    warn = movl_tip_warn_deg();

    /* 真实倾角 = 两个法兰法线的夹角（Z 轴列），比"欧拉角分量最大差"准。
     * 门限仍用欧拉角分量 worst（用户看得懂、和 getpos 对得上），
     * 但毫米数用真实倾角算，不准的度不该乘出准的毫米。 */
    line_pose_to_matrix(start_pose, m0);
    line_pose_to_matrix(end_pose, m1);
    n0[0] = m0[0][2]; n0[1] = m0[1][2]; n0[2] = m0[2][2];
    n1[0] = m1[0][2]; n1[1] = m1[1][2]; n1[2] = m1[2][2];
    dot = n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2];
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;
    tilt = acos(dot) * RAD2DEG;
    swing = pen_mm * 2.0 * sin(tilt / 2.0 / RAD2DEG);

    if (worst <= warn) return;

    printf("[警告] 目标姿态与【当前】姿态不一致：ΔRx=%.2f° ΔRy=%.2f° ΔRz=%.2f°"
           "（法兰倾角变化 %.2f°）\n", drx, dry, drz, tilt);
    printf("       MoveL 会用 SLERP 把姿态【从起点一路拧到目标】，中途姿态一直在变。\n");
    printf("       当前姿态是 Rx=%.2f Ry=%.2f Rz=%.2f —— 就抄 getpos 打印的这三个。\n",
           start_pose[3], start_pose[4], start_pose[5]);
    if (pen_mm > 0.0) {
        printf("       笔长 %.2fmm ⇒ 笔尖绕法兰摆 %.2f×2sin(%.2f°/2) = %.2f mm（空间摆幅）。\n",
               pen_mm, pen_mm, tilt, swing);
        printf("       注意：摆幅大多花在【把笔从朝下抬起来】上，落到纸面的横向偏离"
               "通常远小于它\n       （88.9° 那次空间摆 57.6mm，纸上只有 7.71mm）。\n");
    } else {
        printf("       笔长未配置 ⇒ 算不出毫米数。ini [tool] pen_length 填上法兰面到笔尖的"
               "mm 就能换算。\n");
    }
    printf("       ⇒ 法兰仍走直线，但【笔尖画的是弧/斜线，落点也会偏】。\n");
    printf("       要笔尖走直线：把 Rx,Ry,Rz 抄成 getpos 打印的原值（含负号、含 -0.00）。\n");
    printf("       （告警阈值 %.1f°，ini [movel] tip_warn_deg 可调）\n", warn);
}

void cmd_movel(Robot *robot, const ParsedCmd *cmd)
{
    const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double q_start[6];
    double start_pose[6];
    double end_pose[6];
    double q_seq[LINE_MAX_POINTS][6];
    double seg_dt[LINE_MAX_SEGS];
    double vmax[6];
    JointLimit limits[ROBOT_JOINT_COUNT];
    int joints[6] = {1, 2, 3, 4, 5, 6};
    int count, fail_idx = -1, i, j;
    char fail_reason[64] = {0};
    double total_dt = 0.0;
    double base_rpm = (cmd->speeds[0] > 0) ? cmd->speeds[0] : 60.0;
    double speed_rpm = base_rpm;
    int acc = (cmd->accel_ms[0] >= MOVL_ACC_FLOOR_MS) ? cmd->accel_ms[0] : MOVL_ACC_FLOOR_MS;
    int dec = (cmd->decel_ms[0] >= MOVL_ACC_FLOOR_MS) ? cmd->decel_ms[0] : MOVL_ACC_FLOOR_MS;
    if ((cmd->accel_ms[0] > 0 && cmd->accel_ms[0] < MOVL_ACC_FLOOR_MS) ||
        (cmd->decel_ms[0] > 0 && cmd->decel_ms[0] < MOVL_ACC_FLOOR_MS))
        printf("[提示] 加/减速 %d/%d ms 低于安全下限 %d，已抬到 %d/%d ms"
               "（过短的减速会丢步，表现为走到一半卡住）\n",
               cmd->accel_ms[0], cmd->decel_ms[0], MOVL_ACC_FLOOR_MS, acc, dec);
    int stall_th[ROBOT_JOINT_COUNT];         /* 逐轴过流阈值 mA；0 = 该轴关闭 */
    movl_stall_thresholds(stall_th);

    /* 1) 起点关节角 + FK 求起点笛卡尔位姿 */
    for (j = 0; j < 6; j++) q_start[j] = robot_read_position_deg(robot, j + 1, NULL);
    {
        double m[4][4], rpy[3];
        dh_forward(DH_TABLE, q_start, m);
        dh_pose_to_xyz_rpy(m, start_pose, rpy);
        start_pose[3] = rpy[0] * RAD2DEG;
        start_pose[4] = rpy[1] * RAD2DEG;
        start_pose[5] = rpy[2] * RAD2DEG;
    }
    for (j = 0; j < 6; j++) end_pose[j] = cmd->cartesian[j];
    if (cmd->keep_pose) {
        /* 姿态保持当前不变：直接用 FK 出来的【起点姿态】，不要求用户抄 Rx,Ry,Rz。
         * 抄错姿态角是"画斜线"的头号根因（实测笔尖偏离 7.71mm 而法兰 0.0000mm），
         * 大多数人要的就是"平移过去、姿态别动"，那就别让他抄。 */
        for (j = 0; j < 3; j++) end_pose[3 + j] = start_pose[3 + j];
        printf("MoveL: 姿态保持当前不变（Rx=%.2f Ry=%.2f Rz=%.2f，取自当前位姿）\n",
               end_pose[3], end_pose[4], end_pose[5]);
    }

    /* 2) 按位移定插补点数 → 直线离散 + 逐点 IK + 分段时间表 */
    double dx = end_pose[0] - start_pose[0];
    double dy = end_pose[1] - start_pose[1];
    double dz = end_pose[2] - start_pose[2];
    double dist = sqrt(dx * dx + dy * dy + dz * dz);

    /* 已在目标位姿 ⇒ 直接返回，一次命令都不下发。
     * 不早退的话会拿 dist≈0 去算时间表：段时长 0、节拍 0，
     * 打印出"占段时长 154058%"这种荒谬数字（除零放大），
     * 还会无意义地下发一次运动命令、走完一整套等到位流程。
     * 姿态也要判：位置不动但姿态要转的情况必须照常执行。 */
    if (dist < MOVL_EPS_MM &&
        fabs(ang_delta_deg(end_pose[3], start_pose[3])) < MOVL_EPS_DEG &&
        fabs(ang_delta_deg(end_pose[4], start_pose[4])) < MOVL_EPS_DEG &&
        fabs(ang_delta_deg(end_pose[5], start_pose[5])) < MOVL_EPS_DEG) {
        printf("MoveL: 已在目标位姿（位移 %.2f mm），无需运动\n", dist);
        return;
    }

    /* 姿态偏差告警：姿态抄错会让笔尖画斜线，而法兰坐标完全正常（看不出来）。
     * 放在"已在目标位姿"早退之后 —— 不动就没必要吵。 */
    movl_pose_warn(start_pose, end_pose);

    {
        const double lmin[6] = ROBOT_JOINT_LIMIT_MIN_DEG;
        const double lmax[6] = ROBOT_JOINT_LIMIT_MAX_DEG;
        for (j = 0; j < 6; j++) {
            limits[j].min_deg = lmin[j];
            limits[j].max_deg = lmax[j];
        }
    }
    /* 速度口径：指令 rpm 与 MoveJ 一致，是【电机轴】rpm（0x00D8 / 0x009A 均为电机轴）。
     * 而 line_time_table 的 vmax 是【机械角】deg/s，故必须除以减速比。
     * 曾漏除 ⇒ 时间表比真实快 50 倍 ⇒ stream 段数被算成 1 ⇒ 目标在几毫秒内全砸下去、
     * 臂其实要走 5 秒 ⇒ 退化成单关节空间直插 ⇒ 末端大弧（84mm 实测 7.9mm）。 */
    {
        const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
        for (j = 0; j < 6; j++) vmax[j] = speed_rpm * 6.0 / (double)red[j];
    }

    double step_mm = MOVL_STEP_MM;
    int rc = 0;                  /* movl_plan 的返回码：-4 需要与普通失败区分 */

    /* rc == -4 = 规划层拒绝（单段跳变过大），movl_plan 自己已把原因打印清楚，
     * 这里不能再套"逆解失败/越软限位"的文案 —— 那会把用户往错误方向引。
     * 其余负值才是逆解/时间表失败，fail_idx 与 fail_reason 此时才有意义。 */
    rc = movl_plan(start_pose, end_pose, q_start, limits, dist, step_mm, vmax,
                   q_seq, seg_dt, &count, &total_dt, &fail_idx, fail_reason);
    if (rc != 0) {
        if (rc != -4) {
            printf("[错误] MoveL 第 %d 个插补点逆解失败/越软限位：%s\n",
                   fail_idx, fail_reason);
        }
        return;
    }

    /* stream：把弦步长放大到"一段≈一个节拍"。
     * 固定 1mm 在 60rpm 下每段仅约 1.4ms，而一个下发节拍要 ~100ms，
     * 轴会瞬间走完再干等，反而比 step 更抖。段数 = 总时长 / 目标节拍。
     * 节拍还必须 ≥ 加减速时间之和：段比斜坡还短时，驱动器每段都在刹车+重新爬坡，
     * 既慢（实测慢 3 倍）又顿。取 max(总线节拍, MOVL_SEG_RAMP_RATIO×(ACC+DEC))。 */
    if (cmd->movl_mode == MOVL_MODE_STREAM && count > 2) {
        double beat_s = MOVL_STREAM_BEAT_S;
        double ramp_s = MOVL_SEG_RAMP_RATIO * (double)(acc + dec) / 1000.0;
        int n_seg;
        if (ramp_s > beat_s) beat_s = ramp_s;
        n_seg = (int)ceil(total_dt / beat_s);
        if (n_seg < 1) n_seg = 1;
        if (n_seg < count - 1) {
            step_mm = dist / (double)n_seg;
            rc = movl_plan(start_pose, end_pose, q_start, limits, dist, step_mm, vmax,
                           q_seq, seg_dt, &count, &total_dt, &fail_idx, fail_reason);
            if (rc != 0) {
                if (rc != -4) {
                    printf("[错误] MoveL 第 %d 个插补点逆解失败/越软限位：%s\n",
                           fail_idx, fail_reason);
                }
                return;
            }
        }
    }

    /* sync（默认）：粗弦分段 + 逐航点等到位。
     * 与 step 的差别只在弦长：step 用 1mm（50mm 线要 50 次等待），sync 用
     * MOVL_SYNC_STEP_MM=10mm（50mm 线只要 5 次）。两者都等到位，故都真的走过航点。
     * 【已废弃的旧做法】定时注入（每 acc+20ms 发下一个航点）：实测 50mm 线偏差
     * 1.77mm，等于起点→终点一条长弦的弓高——中间航点被驱动器的 replan 丢弃了。 */
    dev_reset();   /* 全程偏差峰值从 0 开始累计，结束时只汇总一行 */

    /* SMOOTH 必须和 SYNC 走同一个块 —— 块内才有"按弓高预算定段数"的逻辑，
     * SMOOTH 就是把它强制成 1 段（见下方 target_n = 1）。
     * 【2026-09-19 实测修 bug】原来只有 SYNC 能进 ⇒ SMOOTH 掉到后面的 step
     * 路径被切成 101 段（步长 1.0mm），"流畅不分段"**从来没有真正生效过**；
     * 而下方的打印又把 sync/smooth 一律显示成"模式 step"，把排查带偏了。 */
    if (cmd->movl_mode == MOVL_MODE_SYNC ||
        cmd->movl_mode == MOVL_MODE_SMOOTH) {
        const double *q_end = q_seq[count - 1];
        const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
        const double *zero = joint_zero_get();
        const double bow_budget = movl_bow_budget();
        double dmax = 0.0, sync_rpm, sync_bow = 0.0, bow_full = 0.0, dist_axis[7] = {0};
        int32_t tgt[7] = {0};
        uint8_t pend[7] = {0};
        int nr_gap = MOVEJ_NR_GAP_MS;
        int remain = 0, stride, n_pts, last_i = 0, n_inj = 0;

        for (j = 0; j < 6; j++) {
            dist_axis[joints[j]] = fabs(q_end[j] - q_start[j]) * (double)red[j];
            if (dist_axis[joints[j]] > dmax) dmax = dist_axis[joints[j]];
        }
        sync_rpm = (total_dt > 1e-6) ? (dmax / total_dt / 6.0) : base_rpm;
        if (sync_rpm > base_rpm) sync_rpm = base_rpm;
        if (sync_rpm < 1.0) sync_rpm = 1.0;

        n_pts = count - 1;
        /* 自适应弦长：先量"整段不分段"能弯多少，再按弓高预算反推段数。
         * 弓高 ∝ 弦长² ⇒ L = dist × sqrt(预算 / bow_full)。
         * bow_full ≤ 预算时一段走完（n_inj=1），中间不停顿。 */
        bow_full = movl_bow_mm(q_seq[0], q_seq[count - 1], start_pose, end_pose);
        {
            int target_n;
            if (cmd->movl_mode == MOVL_MODE_SMOOTH) {
                /* 流畅优先：整段一次下发，交给驱动器自己做梯形规划。
                 * 与 sync 的区别只在"分不分段"，代价是会偏离笛卡尔直线，
                 * 偏离量 bow_full 已经在下面如实打印，不藏着。 */
                target_n = 1;
            } else if (bow_full <= bow_budget || dist < 1e-9) {
                target_n = 1;                    /* 够直，不用分段 */
            } else {
                double L = dist * sqrt(bow_budget / bow_full);
                if (L < MOVL_MIN_STEP_MM) L = MOVL_MIN_STEP_MM;
                /* 必须向上取整：dist=49.99 时 (int)(49.99/L) 会退化成更长的弦 */
                target_n = (int)ceil(dist / L);
            }
            if (target_n < 1) target_n = 1;
            if (target_n > n_pts) target_n = n_pts;
            stride = (n_pts + target_n - 1) / target_n;
        }
        n_inj = (n_pts + stride - 1) / stride;

        /* 弓高上界取【相邻航点之间】的最大值，不是"起点→终点"整段的弓高。
         * 因为每个航点都等到位再发下一个，实际路径就是 n_inj 条 ≤步长 的弦。
         * 弓高按弦长平方缩小：bow(L) ≈ bow(全程) × (L/全程)²。
         * （改之前打的是整段弓高 2.08mm，而实测 1.77mm——中间航点并没被走过。） */
        for (i = 0; i < count - 1; i += stride) {
            int k = i + stride;
            double d;
            if (k > count - 1) k = count - 1;
            d = movl_bow_mm(q_seq[i], q_seq[k], start_pose, end_pose);
            if (d > sync_bow) sync_bow = d;
        }

        /* 同时打出"整段不分段会弯多少"：让用户看懂段数是怎么定出来的，
         * 而不是只看到一个孤立的偏差上界。 */
        if (cmd->movl_mode == MOVL_MODE_SMOOTH) {
            printf("MoveL: 流畅(不分段), 位移 %.1f mm, 速度 %.1f rpm, 预计 %.2f s, "
                   "整段一次下发由驱动器自己规划 ⇒ 段间零停顿\n"
                   "        代价：走关节空间直线，会偏离笛卡尔直线 %.2f mm"
                   "（弓高 ∝ 长度²，线段越短越看不出来）\n",
                   dist, sync_rpm, total_dt, bow_full);
        } else {
            printf("MoveL: 同步, %d 航点(步长 %.1fmm), 位移 %.1f mm, 速度 %.1f rpm, "
                   "预计 %.2f s, 逐航点等到位, 几何弓高 ≤ %.2f mm"
                   "（整段不分段弯 %.2f mm > 预算 %.2f mm 才分段；过流保护 %s）\n",
                   n_inj, dist / (double)n_inj, dist, sync_rpm, total_dt,
                   sync_bow, bow_full, bow_budget,
                   !movl_stall_on(stall_th) ? "关闭（六轴阈值均为 0）"
                       : (n_inj > 1) ? "开启（每段下发前检查）"
                       : "本段不检查（整段一次下发，无分段点）");
        }

        if (count <= 2 || n_inj <= 1) {
            movej_joints(robot, 6, joints, q_end, sync_rpm, acc, dec,
                         start_pose, end_pose);
            dev_report("sync ");
            return;
        }

        /* 连发帧间延迟：>0 走 noread 连发，0 回退成每轴等响应（A/B 用）。
         * 实测（nrtest）：0/1/2ms 撞车，3ms 起干净，默认 4ms。 */
        nr_gap = movl_noread_gap_ms();

        /* 1) profile + 速度（只写一次） */
        for (j = 0; j < 6; j++) {
            if (robot_is_masked(robot, joints[j])) continue;
            motor_set_profile(robot, joints[j], acc, dec);
        }
        /* 【2026-09-18 改动】速度不再在这里一次性定死。
         * 旧写法用【整段】每轴位移比 dist_axis/dmax 定速，但各轴速率比沿途是变的，
         * 结果 J2/J3/J5 被给快、提前到位停住，剩下的行程只由 J1/J6 走完 ⇒ 路径甩成弓形
         * （50mm 线实测 1.9mm，几何弓高只有 0.54mm）。
         * 改成每段下发前调用 movl_set_seg_speed，按【本段】位移比重新定速。 */
        (void)dist_axis;   /* 仍用于上面算 dmax → sync_rpm，这里不再直接取用 */

        /* 2) 逐航点下发 + 【等它真正到位】再发下一个。
         * 原实现是"定时注入"（每 acc+20ms 发下一个航点）。现场实测
         * (2026-09-18, 50mm 线)：这样跑出来的实测偏差 1.77mm ≈ 起点→终点一条
         * 关节空间直线的弓高（预测 2.08mm），说明中间 4 个航点一个都没被走过——
         * 驱动器收到新的绝对位置指令会 replan，注入间隔内臂只走了约 2% 行程，
         * 旧目标直接被丢弃，最终退化成一条长弦。
         * 必须等到位。50mm 取 10mm 弦 ⇒ 5 次等待、弓高 ≈0.08mm，
         * 比 step 的 1mm 弦(50 次等待)快得多，是目前顺与直的最佳折中。 */
        for (i = 1; i < count; i += stride) {
            /* 【过流保护】每段下发前查一次。
             * sync 是默认模式，而运动期间后台巡检是被挂起的（它要吃六成带宽），
             * 所以这里是运动期间唯一的保护点 —— 少了它，默认模式下碰撞完全没人管。
             * 代价：每段多 6 笔读事务（约 90ms 段间停顿）。 */
            if (movl_stall_guard(robot, stall_th)) {
                dev_report("sync ");
                return;
            }
            for (j = 0; j < 6; j++) {
                if (robot_is_masked(robot, joints[j])) continue;
                tgt[joints[j]] = DEG2STEPS(q_seq[i][j] + zero[joints[j] - 1],
                                           red[joints[j] - 1]);
            }
            /* 按【本段】位移比定速：段的起点是上一个已下发的航点 q_seq[last_i]。
             * last_i 初值 0，第一次进来就是 q_seq[0] → q_seq[1]。 */
            movl_set_seg_speed(robot, joints, q_seq[last_i], q_seq[i], red, sync_rpm);
            /* 连发：省掉每轴 14.85ms 的等响应，六轴启动差 75ms → 约 26ms */
            movej_issue_noread(robot, joints, tgt, nr_gap);
            last_i = i;
            remain = 0;
            for (j = 0; j < 6; j++) {
                if (robot_is_masked(robot, joints[j])) continue;
                pend[joints[j]] = 1;
                remain++;
            }
            if (remain > 0)
                movej_wait(robot, joints, tgt, pend, remain, start_pose, end_pose);
        }
        /* 补发终点 */
        if (last_i != count - 1) {
            for (j = 0; j < 6; j++) {
                if (robot_is_masked(robot, joints[j])) continue;
                tgt[joints[j]] = DEG2STEPS(q_end[j] + zero[joints[j] - 1],
                                           red[joints[j] - 1]);
            }
            movl_set_seg_speed(robot, joints, q_seq[last_i], q_end, red, sync_rpm);
            movej_issue_noread(robot, joints, tgt, nr_gap);
        }

        /* 3) 等待到位。
         * 【必须先归零 remain】上面航点循环里 remain 是"按值"传给 movej_wait 的，
         * 函数内部改的是自己的副本，外面的 remain 仍停在最后一次赋的值（=轴数）。
         * 不归零就会继续累加成 2 倍轴数，而实际只有 6 个轴会置 pend——
         * 六轴全部到位后 remain 还剩一半，while(remain>0) 空转到 60s 超时。
         * 现场症状（2026-09-18）：臂已经精确走到终点（FK 反算 (149.98,2.00,114.00)
         * 对目标 (150.00,2.00,114.00)），状态行却一直刷着不动，看着像"卡死"。 */
        remain = 0;
        for (j = 0; j < 6; j++) {
            if (robot_is_masked(robot, joints[j])) continue;
            pend[joints[j]] = 1;
            remain++;
        }
        if (remain > 0)
            movej_wait(robot, joints, tgt, pend, remain, start_pose, end_pose);
        dev_report("sync ");

        /* 实测远大于几何弓高时点破原因，否则用户会以为是"分段不够密"，
         * 于是继续细分，结果更糟——每段都要重新经历一次六轴启动差。 */
        if (sync_bow > 0.0 && g_dev_peak > 3.0 * sync_bow) {
            printf("        [注意] 实测偏差是几何弓高的 %.1f 倍 ⇒ 不是「分段不够密」。\n"
                   "               按段速度归一化修好之后（2026-09-18），这条线在本机上\n"
                   "               实测已经≈几何弓高（4 段：0.26mm vs 预测 0.14mm）。\n"
                   "               现在还这么大，先怀疑这三条：\n"
                   "               ① 某轴没同时到位 —— 看上面逐行采样里有没有某个 J 提前\n"
                   "                  停住不动了（旧 bug 的典型症状，若复现请查\n"
                   "                  movl_set_seg_speed 是否真的在每段都被调用到）；\n"
                   "               ② 回差/齿隙 —— 正反向各跑一次，差值若稳定在 0.1mm 量级\n"
                   "                  且方向相关，就是它，调分段无效；\n"
                   "               ③ 负载/皮带打滑 —— 换低速（第 7 参数调小）复测。\n",
                   g_dev_peak / sync_bow);
        }

        return;
    }

    /* 段内走关节空间插补 ⇒ 偏离笛卡尔直线，把弓高上界一并打出来。
     * 段数被总线节拍压到 1 时（速度过快、总时长 < 一个节拍）弓高会很大，
     * 必须让用户看见，而不是跑完才发现画弧。 */
    {
        double bow = 0.0;
        for (i = 1; i < count; i++) {
            double d = movl_bow_mm(q_seq[i - 1], q_seq[i], start_pose, end_pose);
            if (d > bow) bow = d;
        }
        printf("MoveL: %d 段, 步长 %.1f mm, 位移 %.1f mm, 节拍 %.3f s, 预计 %.2f s, "
               "模式 %s, 几何弓高 ≤ %.2f mm（过流保护 %s）",
               count - 1, step_mm, dist, (count > 1) ? seg_dt[0] : 0.0, total_dt,
               (cmd->movl_mode == MOVL_MODE_STREAM) ? "stream"
                   : (cmd->movl_mode == MOVL_MODE_SMOOTH) ? "smooth"
                   : (cmd->movl_mode == MOVL_MODE_SYNC) ? "sync" : "step", bow,
               movl_stall_on(stall_th) ? "开启" : "关闭（六轴阈值均为 0）");
        /* 只在【超出用户自己配的预算】时才警告。
         * 旧判据是写死的 MOVL_BOW_WARN_MM=1.0mm，在 bow_mm=2.5 的新默认值下
         * 会对每一条正常线段狂刷"弓高偏大、建议降速"——那是把用户明确接受的
         * 取舍又当成错误报了一遍，纯噪音。是否可接受由 bow_mm 说了算。 */
        {
            const double bow_budget = movl_bow_budget();
            if (bow > bow_budget * 1.2 + 0.05)
                printf("\n[警告] 弓高 %.2f mm 超出预算 %.2f mm（段数被节拍压到 %d 段）："
                       "降速或调大 ACC/DEC 可把节拍拉长、段数变多；"
                       "也可调大 bow_mm 接受这条弧", bow, bow_budget, count - 1);
        }
        /* 段时长若短于加减速时间之和，驱动器每段都在重新爬坡、达不到指令转速，
         * 实际耗时会显著长于"预计"（预计按匀速算）。实测：段 99ms / 加减速 230ms → 慢 2.9 倍
         * 同一组(段时长, ACC+DEC)只提示一次——连画正方形四条边时不要刷四遍。 */
        if (count > 1 && seg_dt[0] * 1000.0 < (double)(acc + dec)) {
            static double hint_seg_ms = -1.0;
            static int    hint_ramp   = -1;
            double seg_ms = seg_dt[0] * 1000.0;
            if (fabs(seg_ms - hint_seg_ms) > 0.5 || (acc + dec) != hint_ramp) {
                hint_seg_ms = seg_ms;
                hint_ramp   = acc + dec;
                printf("\n[提示] 段时长 %.0f ms 短于加减速时间之和 %d ms：每段都爬不完坡就要刹车，"
                       "实际耗时会明显长于预计。想又快又顺就调小 ACC/DEC，"
                       "想更少停顿就调大 ACC/DEC（但会压低段数、弓高变大）",
                       seg_ms, acc + dec);
            }
        }
        printf("\n");
    }

    /* 3) 逐段下发（密集弦逼近直线），段间做 P2 电流自适应限速 */
    int32_t s_tgt[7] = {0};
    uint8_t s_pend[7] = {0};
    int s_remain = 0;
    uint32_t mv_t0 = GetTickCount();
    uint32_t iss_sum = 0, iss_max = 0, iss_n = 0;   /* 单节拍下发实测耗时统计 */
    uint32_t iss_tx = 0;                            /* 累计事务数：首段含加减速 = 3/轴，其余 2/轴 */
    int probe_n = 0;                                /* 段内偏差采样次数：0 表示没采到，偏差只覆盖收尾 */
    for (i = 1; i < count; i++) {
        if (movl_stall_guard(robot, stall_th)) return;
        /* 降速分支（未到急停，但已经逼近阈值）：把速度按比例压下来，
         * 让电流回落，而不是硬顶到报警。留 20% 余量。 */
        if (movl_stall_on(stall_th)) {
            double worst = 0.0;
            for (j = 0; j < ROBOT_JOINT_COUNT; j++) {
                int cur;
                double r;
                if (stall_th[j] <= 0) continue;
                if (robot_is_masked(robot, j + 1)) continue;
                cur = robot_read_current_ma(robot, j + 1);
                if (cur <= 0) continue;
                r = (double)cur / (double)stall_th[j];
                if (r > worst) worst = r;
            }
            if (worst > 0.6) {
                double factor = 0.8 / worst;
                if (factor > 1.0) factor = 1.0;
                speed_rpm = base_rpm * factor;
                if (speed_rpm < 1.0) speed_rpm = 1.0;
                {
                    const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
                    for (j = 0; j < 6; j++) vmax[j] = speed_rpm * 6.0 / (double)red[j];
                }
                if ((count - i) >= 2)
                    line_time_table(&q_seq[i], count - i, vmax, &seg_dt[i], NULL);
            }
        }

        /* 本段速度：取对时间最紧的关节所需【电机轴】rpm
         * （movej_issue 按电机轴行程比例分配，整段同时间到达）
         *
         * 梯形斜坡补偿：走完一段的实际耗时 = 行程/巡航速度 + (ACC+DEC)/2，
         * 直接按 行程/段时长 给速度会永远慢一截（实测慢 3 倍：预计 3.87s 实跑 11.36s）。
         * 故把可用时间扣掉 (ACC+DEC)/2 再算速度；下限取 0.25×段时长，防段时长被压得
         * 接近斜坡时间时速度发散。 */
        double seg_speed = 0.0;
        {
            const uint16_t red[ROBOT_JOINT_COUNT] = ROBOT_REDUCTION_TABLE;
            double t_eff = seg_dt[i - 1] - (double)(acc + dec) / 2000.0;
            double t_floor = 0.25 * seg_dt[i - 1];
            if (t_eff < t_floor) t_eff = t_floor;
            for (j = 0; j < 6; j++) {
                double d = fabs(q_seq[i][j] - q_seq[i - 1][j]) * (double)red[j];
                double rpm = d / t_eff / 6.0;
                if (rpm > seg_speed) seg_speed = rpm;
            }
        }
        if (seg_speed < 1.0) seg_speed = 1.0;

        if (cmd->movl_mode == MOVL_MODE_STREAM) {
            /* 周期刷新：只下发不等待；加减速仅首段写一次，行程参考取上一插补点（省读位置） */
            uint32_t t0 = GetTickCount();
            int tx_before = g_tx_written;
            int r = movej_issue(robot, 6, joints, q_seq[i], q_seq[i - 1],
                                seg_speed, acc, dec, (i == 1) ? 1 : 0, s_tgt, s_pend);
            if (r > 0) s_remain = r;
            iss_tx += (uint32_t)(g_tx_written - tx_before);
            uint32_t period_ms = (uint32_t)(seg_dt[i - 1] * 1000.0);
            if (period_ms < MOVL_STREAM_MIN_MS) period_ms = MOVL_STREAM_MIN_MS;
            uint32_t used = GetTickCount() - t0;   /* 只记下发耗时，不含下面的采样 */
            iss_sum += used; iss_n++;
            if (used > iss_max) iss_max = used;

            /* 段内偏差采样：仅当本节拍还剩得下一轮"读六轴位置"时才采。
             * stream 只在最后一段下发后统一等待，若不在段里补采，偏差统计
             * 就只剩末尾一小截（曾打出假的 0.00 mm），与 sync 的全程峰值不可比。
             * 采样用的是节拍里原本空闲的总线时间，不挤占本段下发。 */
            if (used + MOVL_STREAM_PROBE_MS <= period_ms) {
                movl_dev_sample(robot, start_pose, end_pose);
                probe_n++;
                used = GetTickCount() - t0;
            }
            if (used < period_ms) Sleep(period_ms - used);
            status_line("MoveL stream %d/%d 段", i, count - 1);
        } else {
            movej_joints(robot, 6, joints, q_seq[i], seg_speed, acc, dec,
                         start_pose, end_pose);
        }
    }

    /* stream：全部段下发完后才统一等待真正到位，避免命令返回时臂仍在运动 */
    status_clear();
    if (cmd->movl_mode == MOVL_MODE_STREAM) {
        if (s_remain > 0) movej_wait(robot, joints, s_tgt, s_pend, s_remain,
                                     start_pose, end_pose);
        printf("MoveL stream %d 段完成，实际耗时 %.2f s（预计 %.2f s）\n",
               count - 1, (GetTickCount() - mv_t0) / 1000.0, total_dt);
        if (probe_n > 0)
            printf("    （偏差为全程峰值：段内采样 %d 次 + 收尾等待）\n", probe_n);
        else
            printf("    [注意] 节拍太短，段内一次都没采到 ⇒ 下面的偏差只覆盖最后\n"
                   "           一段的收尾，不代表全程，不要拿它和 sync 比。\n"
                   "           想采到就放慢速度或调大 ACC/DEC，把节拍拉长到 %d ms 以上。\n",
                   MOVL_STREAM_PROBE_MS);
        dev_report("stream ");
        if (iss_n > 0) {
            double per = (double)iss_sum / (double)iss_n;
            double per_tx = (double)iss_sum / (double)iss_tx;
            double seg_ms = (count > 1) ? seg_dt[0] * 1000.0 : 0.0;
            /* 六轴是【顺序】下发的：第 6 轴比第 1 轴晚启动约一整个节拍。
             * 节拍占段时长的比例越大，段内各轴行程完成度越不一致 ⇒ 路径被扭曲，
             * 实测偏差会远大于弓高预测（实测：占比 48% → 0.10mm；72% → 2.58mm）。 */
            printf("    下发：每节拍 %.0f ms（峰值 %.0f ms），单事务 %.1f ms",
                   per, (double)iss_max, per_tx);
            if (seg_ms > 0.0)
                printf("，占段时长 %.0f%%", 100.0 * per / seg_ms);
            printf("\n");
            /* 下面两条【提示】每条只在本次进程里出现一次：
             * 连画正方形四条边时不应该被同一句话提醒四遍。 */
            if (seg_ms > 0.0 && per > 0.25 * seg_ms) {
                static int beat_hint_done = 0;
                if (!beat_hint_done) {
                    beat_hint_done = 1;
                    printf("    [提示] 节拍超过段时长的 25%%，六轴起步严重不同步，末端偏差会远大于弓高。\n"
                           "           优先降低单事务耗时（换低延迟转换器 / 提高波特率 / 广播下发），"
                           "其次加长段时长（调大 ACC/DEC 或放慢速度）。\n");
                }
            }
            if (per_tx > 8.0) {
                static int lat_hint_done = 0;
                if (!lat_hint_done) {
                    lat_hint_done = 1;
                    printf("    [提示] 单事务 %.1f ms 明显偏慢（115200 下纯线上时间约 1.8 ms）。\n"
                           "           FTDI / CP210x / PL2303 芯片：设备管理器 → 端口(COM 和 LPT) →\n"
                           "           你的串口 → 端口设置 → 高级 → “延迟计时器(毫秒)” 16 改 1 →\n"
                           "           确定后重开本程序，通常可提速数倍。\n"
                           "           【CH340（VID_1A86&PID_7523）不适用】：其驱动没有这一项，改不了。\n"
                           "           只能换 FTDI/CP210x 转换器；提波特率收益有限——实测 8.4 倍开销里\n"
                           "           线上只占 1.8 ms，波特率翻倍最多省一成，大头是等响应与系统调度。\n",
                           per_tx);
                }
            }
        }
    } else {
        dev_report("step ");   /* step：每段一次 movej_joints，峰值已在 g_dev_peak 里累计 */
    }
}

/* flange_tilt_deg：法兰法线（= 末端 Z 轴 = J5 回转轴，因 α6=0）与竖直向下 (0,0,-1)
 * 的夹角，度。0 = 法兰盘水平、笔/工具垂直朝下。 */
static double flange_tilt_deg(const double pose[4][4])
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double c = -pose[2][2];
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return acos(c) * RAD2DEG;
}

/* cmd_getpos：读取当前关节机械角，经正运动学求末端笛卡尔坐标与姿态 */
void cmd_getpos(Robot *robot)
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    double q[6];
    int ok[6];
    double pose[4][4];
    double xyz[3], rpy[3];
    int all_ok = 1;

    for (int i = 0; i < 6; i++) {
        q[i] = robot_read_position_deg(robot, i + 1, &ok[i]);
        if (!ok[i]) all_ok = 0;
    }

    dh_forward(DH_TABLE, q, pose);
    dh_pose_to_xyz_rpy(pose, xyz, rpy);

    for (int i = 0; i < 6; i++) {
        printf(i ? ", J%d = %.2f°" : "J%d = %.2f°", i + 1, q[i]);
    }
    printf("\n");

    printf("X = %.2f , Y = %.2f , Z = %.2f , Rx = %.2f , Ry = %.2f , Rz = %.2f\n",
           xyz[0], xyz[1], xyz[2],
           rpy[0] * RAD2DEG, rpy[1] * RAD2DEG, rpy[2] * RAD2DEG);
    printf("法兰倾角 = %.2f°  %s   (q2+q3+q5 = %.2f)\n",
           flange_tilt_deg(pose),
           flange_tilt_deg(pose) < 0.05 ? "垂直于地面" : "歪了",
           q[1] + q[2] + q[4]);

    if (!all_ok) printf("[警告] 部分关节读取失败，坐标按读取值计算\n");
}

/* cmd_fk：离线正解预览——不读硬件、不下发、不动臂。
 * 只按当前 DH 表算出"这组关节角应该得到什么位姿"，
 * 用来和 getpos 的真机读数逐项对照，一眼区分【模型错】还是【电机没走到】。 */
void cmd_fk(const ParsedCmd *cmd)
{
    static const double RAD2DEG = 180.0 / 3.14159265358979323846;
    static const double DEG2RAD = 3.14159265358979323846 / 180.0;
    double q[6], pose[4][4], xyz[3], rpy[3], tilt;
    double acc[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double t[4][4];
    int i, j, k;

    for (i = 0; i < 6; i++) q[i] = cmd->angles[i];

    dh_forward(DH_TABLE, q, pose);
    dh_pose_to_xyz_rpy(pose, xyz, rpy);
    tilt = flange_tilt_deg(pose);

    printf("关节角 : J1..J6 = %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           q[0], q[1], q[2], q[3], q[4], q[5]);
    printf("末端   : X = %.2f , Y = %.2f , Z = %.2f , "
           "Rx = %.2f , Ry = %.2f , Rz = %.2f\n",
           xyz[0], xyz[1], xyz[2],
           rpy[0] * RAD2DEG, rpy[1] * RAD2DEG, rpy[2] * RAD2DEG);
    printf("法兰   : 法线 = (%.3f, %.3f, %.3f)  与竖直向下夹角 = %.2f°  %s\n",
           pose[0][2], pose[1][2], pose[2][2], tilt,
           (tilt < 0.05) ? "垂直于地面" : "歪了");
    printf("姿态和 : q2+q3+q5 = %.2f   （=180 时法兰/笔正好竖直朝下）\n",
           q[1] + q[2] + q[4]);

    /* 各关节点坐标：用来和真机臂形对比（X,Y,Z mm） */
    printf("臂形   : 基座(0.0,0.0,0.0)");
    for (i = 0; i < 6; i++) {
        double nx[4][4];
        dh_transform(&DH_TABLE[i], q[i] * DEG2RAD + DH_TABLE[i].theta_offset, t);
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 4; k++) {
                nx[j][k] = 0.0;
                for (int m = 0; m < 4; m++) nx[j][k] += acc[j][m] * t[m][k];
            }
        }
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 4; k++) acc[j][k] = nx[j][k];
        }
        printf(" → J%d(%.1f,%.1f,%.1f)", i + 1, acc[0][3], acc[1][3], acc[2][3]);
    }
    printf("\n");
}

/* cmd_diag：总线时延体检——不动臂，只量"时间花在哪"。
 *
 * 为什么要这个命令：单事务 26ms 是整套系统的速度瓶颈，但它到底花在哪一段
 * 必须实测。本项目用的是 CH340（WCH CH341SER_A64 驱动），注册表
 * Enum\USB\VID_1A86&PID_7523\...\Device Parameters 里【没有】FTDI 那种
 * LatencyTimer 项，所以"设备管理器改延迟计时器"这条路对它根本不存在。
 * 于是把事务拆成 flush / write / read 三段分别计时，用数据说话。
 *
 * 对照实验：再打一组"只写不读"。若这一组明显更快，说明 26ms 里的大头是
 * 【等从站响应上行】，那正确的优化方向就是让纯写指令不等响应（配合每条
 * 总线只有一个从站），而不是去改一个不存在的延迟计时器。 */
void cmd_diag(Robot *robot, const ParsedCmd *cmd)
{
    (void)cmd;
    const int N = 30;
    uint32_t n = 0, n_nr = 0;
    double fl = 0.0, wr = 0.0, rd = 0.0, tot = 0.0, nr = 0.0;
    int i, ok = 0;

    if (robot == NULL) return;

    /* 【测量前先请后台巡检让出总线】
     * diag 要量的是"独占总线时的单事务耗时"。后台巡检一轮 12 笔事务，
     * 即使帧不撞车（已由机器人总线锁保证），它插进来的事务仍会排在
     * diag 的事务之间——测出来的"单事务"就变成"含排队等待"的数字，
     * 而巡检自己也会因为排队超时刷出"关节N 掉线"的假报警。
     * monitor_pause_active 只置标志，线程要到下一个循环点才停，
     * 故再等一个巡检周期，确保正在进行的那轮 poll 已经走完。 */
    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    /* 1) 完整事务：flush + write + read(等响应) */
    modbus_stats_reset();
    for (i = 0; i < N; i++) {
        int32_t p = motor_read_position(robot, 1, &ok);
        (void)p;
    }
    modbus_stats_get(&n, &fl, &wr, &rd, &tot, &n_nr, &nr);

    /* 立刻另存一份：下面第 2 段会 modbus_stats_reset() 并只跑 noread，
     * 完整事务数 n=0 ⇒ 再取一次 tot 会得到 0.0，把"上界倍数 / 推算值 /
     * 实跑吞吐"全打成 0。巡检挂起前这个 bug 被掩盖了——那时第 2 段里混着
     * 巡检插入的完整事务，tot 拿到的是巡检事务的耗时，同样不是 diag 的数。 */
    const double tot_rd = tot;

    printf("总线体检：%u 次完整事务（读 J1 位置，只读不动臂）\n", n);
    printf("    flush (PurgeComm)  %7.2f ms\n", fl);
    printf("    write  (下发请求)  %7.2f ms\n", wr);
    printf("    read   (等响应)    %7.2f ms   ← 通常就是大头\n", rd);
    printf("    ── 单事务合计      %7.2f ms\n", tot);
    printf("    一轮 6 轴 %7.1f ms  ⇒  理论刷新率 %6.1f Hz\n",
           tot * 6.0, 1000.0 / (tot * 6.0));

    /* 2) 对照：只写不读。写 J1 的【当前位置】= 原地不动，安全 */
    {
        int32_t pos0 = motor_read_position(robot, 1, &ok);
        if (!ok) {
            printf("[警告] 读 J1 位置失败，跳过只写不读对照\n");
            goto done;
        }
        modbus_stats_reset();
        for (i = 0; i < N; i++)
            motor_move_abs_noread(robot, 1, pos0);
        modbus_stats_get(&n, &fl, &wr, &rd, &tot, &n_nr, &nr);
        printf("对照：%u 次只写不读（写 J1 当前位置，原地不动）\n", n_nr);
        printf("    ── 单事务合计      %7.2f ms\n", nr);
        printf("    一轮 6 轴 %7.1f ms  ⇒  理论刷新率 %6.1f Hz\n",
               nr * 6.0, 1000.0 / (nr * 6.0));
        /* 注意：这一组【没有算总线让位时间】。RS485 半双工下从站仍会回一帧，
         * 我们不等它，但下一帧必须等从站发完才能上路，否则撞车。
         * 所以它是"省掉等响应"的收益上界，不是共享总线上可直接达到的数值。 */
        printf("    （上界：省掉等响应最多 %.1f 倍；共享总线上还要再留出从站响应时间）\n",
               (nr > 0.0 && tot_rd > 0.0) ? (tot_rd / nr) : 0.0);
    }
    /* 3) 真实一轮：连读 J1..J6 位置，直接测"一轮 6 轴"耗时。
     * 前面用"单事务 ×6"是推算，假定六个事务之间没有批处理/调度差异；
     * 这里实测真的一轮，才是 movej_wait 轮询与 stream 节拍面对的真实数字。 */
    {
        const int R = 10;
        LARGE_INTEGER freq, a, b;
        double sum = 0.0;
        QueryPerformanceFrequency(&freq);
        for (i = 0; i < R; i++) {
            int j;
            QueryPerformanceCounter(&a);
            for (j = 1; j <= 6; j++) {
                int32_t p = motor_read_position(robot, j, &ok);
                (void)p;
            }
            QueryPerformanceCounter(&b);
            sum += (double)(b.QuadPart - a.QuadPart) * 1000.0 / (double)freq.QuadPart;
        }
        printf("实测一轮：%d 次\"连读 J1..J6\"，平均 %7.2f ms  ⇒  刷新率 %6.1f Hz\n",
               R, sum / R, 1000.0 / (sum / R));
        printf("          对比推算值 %.2f ms（单事务×6）——两者差得越多，"
               "说明批处理/调度的影响越大\n", tot_rd * 6.0);
    }
    {
        /* 纯线上时间 = 21 字节 × 10 bit / 波特率。与实测对比，能看出
         * "转换器/系统开销"占了多少，以及提波特率还有多少空间。 */
        double wire = 21.0 * 10.0 / (double)MODBUS_BAUDRATE * 1000.0;
        printf("参考：%u bps 下单事务纯线上时间 %.2f ms（21 字节 × 10 bit）\n",
               (unsigned)MODBUS_BAUDRATE, wire);
        printf("      实测/线上 = %.1f 倍 ⇒ 其余全是等待与系统开销\n",
               (wire > 0.0) ? (tot_rd / wire) : 0.0);
        printf("      转换器吞吐上界约 %.0f 事务/秒，当前实跑约 %.0f 事务/秒\n",
               1000.0 / wire, 1000.0 / ((tot_rd > 0.0) ? tot_rd : 1.0));
        /* 两个天花板必须并列看，否则会误读：
         * 79 Hz 是"每笔都等从站响应"的极限（wire + 3.5字符帧间隔）；
         * 而只写不等响应实测能到 158 Hz，但那是【上界】——RS485 半双工下
         * 从站仍要回一帧、仍占着总线，下一帧必须等它发完才敢上路，
         * 所以真实值在这两行之间，且只有在"每条总线只挂一个从站"时才安全。 */
        printf("      单总线 115200 下【六轴刷新率天花板】约 %.0f Hz（读写事务，等响应）\n",
               1000.0 / ((wire + 0.30) * 6.0));
        printf("      改\"只写不等响应\"的上界约 %.0f Hz（未计从站响应占线时间）\n",
               1000.0 / (nr * 6.0));
    }
    printf("\n");

done:
    monitor_pause_active(0);   /* 体检结束，交还总线给后台巡检 */
}

/* cmd_bcast：验证广播地址(0) 是否被 LEESN 真正执行。
 *
 * 为什么要测：协议 §1 写"地址 0 = 广播，从机识别但不返回报文"。若成立，
 * "每条总线只挂一个从站 + 用广播下发位置"就是消除"等响应 15ms"最干净的方案——
 * 从站根本不发回包，既不用等，也不会像 noread 那样在半双工上撞车。
 * 这一条直接决定"6 路独立 USB-RS485"这套提速路线值不值得做。
 *
 * 分两级测：先 10H（写 0x00D8 运行速度，INT32）；全无响应再试 06H
 * （写 0x009A 连续运行速度源，UINT16）——部分国产驱动器只实现 06H 的广播，
 * 10H 广播帧被直接丢弃。两级都测，一趟跑完，不用改代码重编译。
 *
 * 为什么挑速度寄存器：广播会让总线上【所有】从站执行同一条指令，必须选
 * "写错了也无害"的寄存器。速度源不动臂，且测完逐个恢复。
 * 【绝不能】拿 0x00E8(绝对位置)/0x00DE(相对运动) 做广播试验——六轴会一起动。 */
void cmd_bcast(Robot *robot)
{
    int32_t b32[7] = {0}, a32[7] = {0};
    uint16_t b16[7] = {0}, a16[7] = {0};
    const int32_t probe32 = 10000;   /* 0x00D8 = 100.00 rpm（单位 0.01rpm） */
    const uint16_t probe16 = 100;    /* 0x009A = 100 rpm（单位 1rpm） */
    int j, hit10 = 0, hit06 = 0, amb = 0;

    if (robot == NULL) return;

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);   /* 等后台巡检线程真正停下来 */

    printf("广播帧验证：向地址 0 写速度寄存器，再逐轴读回看是否被执行\n");
    printf("  （广播会让总线上所有从站执行同一条指令，故选不动臂的速度源，测完恢复）\n\n");

    /* ---------- 第一级：10H 写 0x00D8 ---------- */
    printf("[1] 功能码 10H：广播写 0x00D8(运行速度) = 100.00 rpm\n");
    for (j = 1; j <= 6; j++) {
        int32_t v = -1;
        if (motor_read_i32(robot, j, LEESN_REG_VEL_RUN, &v) != ERR_NONE) v = -1;
        b32[j] = v;
        if (v == probe32) amb++;
    }
    if (motor_write_i32_broadcast(robot, LEESN_REG_VEL_RUN, probe32) != ERR_NONE)
        printf("  [错误] 广播帧下发失败（总线错误）\n");
    Sleep(200);      /* 给从站处理与寄存器生效的时间 */

    printf("  轴    写前(rpm)    写后(rpm)   执行\n");
    for (j = 1; j <= 6; j++) {
        int32_t v = -1;
        int h;
        if (motor_read_i32(robot, j, LEESN_REG_VEL_RUN, &v) != ERR_NONE) v = -1;
        a32[j] = v;
        h = (v == probe32) ? 1 : 0;
        if (h) hit10++;
        printf("  J%d   %10.2f   %10.2f    %s\n",
               j, (double)b32[j] / 100.0, (double)a32[j] / 100.0, h ? "是" : "否");
    }

    /* ---------- 第二级：06H 写 0x009A（仅当 10H 全无响应） ---------- */
    if (hit10 == 0) {
        printf("\n  10H 广播无响应，改测 06H（部分驱动器只实现单寄存器广播）\n");
        printf("[2] 功能码 06H：广播写 0x009A(连续运行速度源) = 100 rpm\n");
        for (j = 1; j <= 6; j++) {
            uint16_t v = 0;
            if (motor_read_u16(robot, j, LEESN_REG_RUN_SPEED16, &v) != ERR_NONE) v = 0;
            b16[j] = v;
        }
        if (motor_write_u16_broadcast(robot, LEESN_REG_RUN_SPEED16, probe16) != ERR_NONE)
            printf("  [错误] 广播帧下发失败（总线错误）\n");
        Sleep(200);

        printf("  轴    写前(rpm)    写后(rpm)   执行\n");
        for (j = 1; j <= 6; j++) {
            uint16_t v = 0;
            int h;
            if (motor_read_u16(robot, j, LEESN_REG_RUN_SPEED16, &v) != ERR_NONE) v = 0;
            a16[j] = v;
            h = (v == probe16) ? 1 : 0;
            if (h) hit06++;
            printf("  J%d   %10u   %10u    %s\n", j,
                   (unsigned)b16[j], (unsigned)a16[j], h ? "是" : "否");
        }
    }

    printf("\n判定：");
    if (hit10 == 6 || hit06 == 6) {
        printf("六轴全部执行广播（%s）⇒ 方案成立。\n", hit10 == 6 ? "10H" : "06H");
        printf("  每条总线只挂一个从站时，可用广播下发位置：从站不回包，\n");
        printf("  既省掉每笔约 15ms 的等响应，也不会像 noread 那样撞车。\n");
        printf("  ⇒ 6 路独立 USB-RS485 值得做，一轮六轴可从 92ms 降到约 6ms。\n");
    } else if (hit10 == 0 && hit06 == 0) {
        printf("两种功能码都无响应 ⇒ 该型号不执行广播帧。\n");
        printf("  退回 noread 方案（只写不等响应），需先测出从站响应占线时间；\n");
        printf("  或者保持现状——用 smooth 的单段下发换流畅，接受小弧。\n");
    } else {
        printf("只有 %d 轴响应 ⇒ 广播支持不稳定，不要用于下发位置。\n",
               hit10 ? hit10 : hit06);
    }
    if (amb > 0)
        printf("  注意：有 %d 轴写前值恰好已是 100rpm，这些轴无法区分，"
               "改探针值重测才准。\n", amb);

    /* 只恢复被改动的轴，避免多余的总线写 */
    printf("\n恢复原速度值…");
    for (j = 1; j <= 6; j++) {
        if (a32[j] != b32[j]) motor_set_speed(robot, j, (double)b32[j] / 100.0);
        if (a16[j] != b16[j]) motor_set_speed16(robot, j, (int)b16[j]);
    }
    printf("已恢复。可再跑一次 bcast 确认（写后值应等于写前值）。\n\n");

    monitor_pause_active(0);
}

/* cmd_nrtest：noread（只写不等响应）连发的帧完整性测试。
 *
 * 为什么需要它：bcast 已验证广播能执行，但广播【不区分轴】，在现在"六从站共线"
 * 的接法下用广播下发位置会让六轴跑到同一个点。所以不换硬件的话，唯一能把
 * 一轮六轴从 92ms 压下来的办法是【单播 noread】——它带地址、能区分轴，
 * 但代价是从站照样会回一帧，半双工下可能和我们的下一帧撞车。
 *
 * 测法：反复给六轴下发【当前位置】⇒ 臂原地不动（零风险），
 * 同时逐档加大帧间延迟，看哪一档开始"读回不再出错"。
 * 判据是读回成功率 + 位置是否漂移，而不是"有没有报错"——
 * 帧撞坏的典型表现是接下来的一次读拿到脏数据或超时。 */
void cmd_nrtest(Robot *robot)
{
    /* 0=完全背靠背（diag 测出的 158Hz 上界就是这一档，实际会撞车） */
    static const int delays[] = {0, 1, 2, 3, 5};
    const int rounds = 10;
    int32_t base[7];
    int j, d, r;

    if (robot == NULL) return;

    monitor_pause_active(1);
    Sleep(MONITOR_DEFAULT_INTERVAL_MS + 20);

    printf("noread 连发帧完整性测试：反复下发【当前位置】⇒ 臂原地不动\n");
    printf("（下发目标 = 当前位置，所以不会动；测的是连发会不会撞车）\n\n");

    for (j = 1; j <= 6; j++) {
        int ok = 0;
        base[j] = motor_read_position(robot, j, &ok);
        if (!ok) {
            printf("[错误] 读 J%d 初始位置失败，测试中止\n", j);
            monitor_pause_active(0);
            return;
        }
    }

    printf("  帧间延迟   一轮六轴耗时    读失败    位置漂移    判定\n");
    for (d = 0; d < (int)(sizeof(delays) / sizeof(delays[0])); d++) {
        int fail = 0, drift = 0;
        uint32_t t0 = GetTickCount();

        for (r = 0; r < rounds; r++) {
            for (j = 1; j <= 6; j++) {
                motor_move_abs_noread(robot, j, base[j]);
                if (delays[d] > 0) Sleep((DWORD)delays[d]);
            }
        }
        {
            uint32_t used = GetTickCount() - t0;
            double per_round = (double)used / (double)rounds;

            /* 验证：连发之后总线还能不能正常读回六轴、位置有没有被带偏 */
            for (j = 1; j <= 6; j++) {
                int ok = 0;
                int32_t p = motor_read_position(robot, j, &ok);
                if (!ok) {
                    fail++;
                } else if (p < base[j] - MOVEJ_INPOS_TOL ||
                           p > base[j] + MOVEJ_INPOS_TOL) {
                    drift++;
                }
            }
            printf("  %6d ms   %10.2f ms   %6d/6   %6d/6    %s\n",
                   delays[d], per_round, fail, drift,
                   (fail == 0 && drift == 0) ? "干净" : "撞车/异常");
        }
    }

    printf("\n读法：找到【判定=干净】的【最小】延迟，那一档就是安全帧间隔。\n");
    printf("      一轮六轴耗时 ≈ 6 ×(写 0.16ms + 该延迟) + 从站回包占线。\n");
    printf("      对比基线：现在等响应的做法一轮 92ms（10.8Hz）。\n");
    printf("      若连 5ms 都不干净 ⇒ 单总线 noread 走不通，只能换 6 路总线用广播。\n\n");

    monitor_pause_active(0);
}
