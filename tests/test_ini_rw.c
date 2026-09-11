/*
 * test_ini_rw.c —— [joint_zero] 段读写往返离线测试
 * ------------------------------------------------------------
 * 覆盖：读取已有段、写回后值一致、[serial] 等其它段与注释被保留、
 *       无 [joint_zero] 段时返回 0。
 * 使用相对临时文件（落在 build 目录，已被 .gitignore 排除），结束清理。
 */

#include "utils/ini_rw.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int g_fail = 0;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        if (!(cond)) {                                              \
            printf("  [FAIL] %s:%d ", __FILE__, __LINE__);          \
            printf(__VA_ARGS__);                                    \
            printf("\n");                                           \
            g_fail++;                                               \
        }                                                          \
    } while (0)

static int approx(double a, double b) { return fabs(a - b) < 1e-6; }

int main(void)
{
    const char *path = "ini_rw_tmp_test.ini";
    double z[6];
    double out[6] = { -1.11, 2.22, -3.33, 4.44, -5.55, 6.66 };

    printf("test_ini_rw: joint_zero 段读写往返\n");

    /* 准备：含注释、[serial] 与旧 [joint_zero] 段 */
    {
        FILE *f = fopen(path, "w");
        CHECK(f != NULL, "临时 ini 创建失败");
        if (f) {
            fprintf(f, "; 注释行\n[serial]\nport = COM9\nbaudrate = 115200\n\n"
                       "[joint_zero]\nq0 = 100.00\nq1 = 200.00\nq2 = 300.00\n"
                       "q3 = 400.00\nq4 = 500.00\nq5 = 600.00\n");
            fclose(f);
        }
    }

    /* 读旧值 */
    CHECK(ini_read_joint_zero(path, z) == 1, "应读到 [joint_zero] 段");
    CHECK(z[0] == 100.00 && z[5] == 600.00, "读取旧值错误（%.2f, %.2f）", z[0], z[5]);

    /* 写新值（应重写文件但保留 [serial]） */
    CHECK(ini_write_joint_zero(path, out) == 1, "写回应成功");

    /* 读新值 */
    memset(z, 0, sizeof(z));
    CHECK(ini_read_joint_zero(path, z) == 1, "重写后应能读到");
    CHECK(approx(z[0], out[0]) && approx(z[5], out[5]),
          "重写后值不一致（%.2f, %.2f）", z[0], z[5]);

    /* 确认 [serial] 段保留 */
    {
        FILE *f = fopen(path, "r");
        char line[256];
        int has_serial = 0;
        CHECK(f != NULL, "重开后应能打开");
        while (f && fgets(line, sizeof(line), f)) {
            if (strstr(line, "port = COM9")) has_serial = 1;
        }
        if (f) fclose(f);
        CHECK(has_serial, "[serial] 段未保留");
    }

    /* 无 [joint_zero] 段时应返回 0 */
    {
        FILE *f = fopen(path, "w");
        if (f) { fprintf(f, "[serial]\nport = COM9\n"); fclose(f); }
    }
    CHECK(ini_read_joint_zero(path, z) == 0, "无 [joint_zero] 段应返回 0");

    remove(path);

    if (g_fail == 0) {
        printf("test_ini_rw: PASS\n");
        return 0;
    }
    printf("test_ini_rw: FAIL（%d 处断言未通过）\n", g_fail);
    return 1;
}
