/*
 * test_modbus.c —— Modbus RTU 帧构造与解析单元测试（CTest，不连硬件）
 * ------------------------------------------------------------
 * 所属模块：测试（tests）
 * 对外接口：main（入口）
 * 依赖模块：comm/modbus_rtu、comm/crc16
 */

/*
 * test_modbus: Modbus RTU 03H/06H/10H 帧构造与解析测试（不连硬件）
 * 帧内容与立三（LEESN）《485 通讯手册》（V126）示例对齐：
 *   03H 读位置 0x0004 x2 -> 01 03 00 04 00 02 + CRC(0xCA85)
 *   03H 读状态响应       -> 01 03 04 03 02 00 00 + CRC(0xB75B)  （X1 输入+运行中）
 *   06H 写运行 0x00C8    -> 01 06 00 C8 01 01 + CRC(0x64C8)     （反向运行）
 *   10H 写绝对位置 0x00E8 -> 01 10 00 E8 00 02 04 E0 C0 FF FF + CRC(0x0DCA) （-8000，低字在前）
 * 注意：立三 32 位寄存器为「低 16 位寄存器在前、字内高字节在前」，
 * 与 robot_internal.h 中的 DWORD 字节序契约一致。
 */

#include "comm/modbus_rtu.h"
#include "comm/crc16.h"
#include "utils/err.h"

#include <stdio.h>
#include <string.h>

static int g_fail = 0;

static void check(int cond, const char *name)
{
    if (cond) {
        printf("通过 [%s]\n", name);
    } else {
        printf("失败 [%s]\n", name);
        g_fail++;
    }
}

/* 计算并附加 CRC（低字节在前）到 len 字节的帧后，返回新帧长 */
static size_t append_crc16(uint8_t *buf, size_t len)
{
    uint16_t crc = crc16_modbus(buf, len);
    buf[len]     = (uint8_t)(crc & 0xFF);
    buf[len + 1] = (uint8_t)(crc >> 8);
    return len + 2;
}

/* ========== 方案一：内存假串口（CommOps 注入验证） ==========
 * 假串口维护两条环形缓冲：tx_buf 记录主站写出的请求帧，
 * rx_buf 预置从站应答帧，read_frame 逐字节吐出，模拟一次总线往返。 */

#define FAKE_BUF_SIZE 256

static uint8_t  g_fake_tx[FAKE_BUF_SIZE];   /* 主站写出的帧（从站视角收到） */
static size_t   g_fake_tx_len;
static uint8_t  g_fake_rx[FAKE_BUF_SIZE];   /* 从站应答帧（主站视角收到） */
static size_t   g_fake_rx_pos;
static size_t   g_fake_rx_len;
static int      g_fake_open_calls;

/* 预置应答：write_frame 收到请求帧后才"上线"，模拟真实从站响应时序 */
static uint8_t  g_fake_resp[FAKE_BUF_SIZE];
static size_t   g_fake_resp_len;

static int fake_open(const char *port, uint32_t baud,
                     uint8_t data_bits, char parity, uint8_t stop_bits)
{
    (void)port; (void)baud; (void)data_bits; (void)parity; (void)stop_bits;
    g_fake_open_calls++;
    return 0;
}

static void fake_close(void)
{
    g_fake_open_calls = 0;
}

static int fake_read_frame(uint8_t *buf, int cap, int timeout_ms)
{
    size_t n;
    (void)timeout_ms;
    if (g_fake_rx_pos >= g_fake_rx_len) {
        return 0;   /* 无数据 = 超时 */
    }
    n = g_fake_rx_len - g_fake_rx_pos;
    if ((int)n > cap) n = (size_t)cap;
    memcpy(buf, g_fake_rx + g_fake_rx_pos, n);
    g_fake_rx_pos += n;
    return (int)n;
}

static int fake_write_frame(const uint8_t *buf, int len)
{
    if (len > FAKE_BUF_SIZE) return -1;
    memcpy(g_fake_tx, buf, (size_t)len);
    g_fake_tx_len = (size_t)len;
    /* 主站请求完整写出后，从站应答帧上线 */
    if (g_fake_resp_len > 0) {
        memcpy(g_fake_rx, g_fake_resp, g_fake_resp_len);
        g_fake_rx_len = g_fake_resp_len;
        g_fake_rx_pos = 0;
    }
    return len;
}

static void fake_flush(void)
{
    g_fake_rx_pos = 0;
    g_fake_rx_len = 0;
}

/* 预置从站应答帧 */
static void fake_set_response(const uint8_t *buf, size_t len)
{
    if (len > FAKE_BUF_SIZE) len = FAKE_BUF_SIZE;
    memcpy(g_fake_resp, buf, len);
    g_fake_resp_len = len;
}

static const CommOps g_fake_comm_ops = {
    fake_open,
    fake_close,
    fake_read_frame,
    fake_write_frame,
    fake_flush
};

int main(void)
{
    uint8_t frame[64];
    uint8_t rx[64];
    ModbusFrame resp;
    size_t len;
    uint16_t vals[2];

    printf("== test_modbus ==\n");

    /* ========== 03H 读多个寄存器：读实时位置 0x0004~0x0005 ========== */
    /* 请求帧：01 03 00 04 00 02 + CRC(0xCA85) -> 帧尾 85 CA（手册 p7） */
    len = modbus_build_read(0x01, 0x0004, 0x0002, frame);
    check(len == 8, "03H 帧长度");
    check(frame[0] == 0x01 && frame[1] == 0x03, "03H 从站/功能码");
    check(frame[2] == 0x00 && frame[3] == 0x04 && frame[4] == 0x00 && frame[5] == 0x02,
          "03H 地址/数量");
    check(frame[6] == 0x85 && frame[7] == 0xCA, "03H CRC 低字节在前 (0xCA85)");

    /* 03H 响应解析：读状态 0x0006~0x0007（手册 p8 示例）：
     * 01 03 04 03 02 00 00 + CRC(0xB75B)
     * 状态 UINT32 低字在前：低 16 位 = data[0..1] = 0x0302
     *  -> bit1(X1 输入)=1、bit8~9(运行状态)=11=正在运行，与手册语义一致 */
    rx[0] = 0x01; rx[1] = 0x03; rx[2] = 0x04;
    rx[3] = 0x03; rx[4] = 0x02; rx[5] = 0x00; rx[6] = 0x00;
    len = append_crc16(rx, 7);
    check(modbus_parse_response(rx, len, &resp) == ERR_NONE, "03H 响应 CRC 通过");
    check(resp.data_len == 4 && resp.data[0] == 0x03 && resp.data[1] == 0x02,
          "03H 响应数据（低字在前 data[0..1]=0x0302）");
    {
        int st = (int)((resp.data[0] << 8) | resp.data[1]);  /* 与 robot_read_status 一致 */
        check((st & 0x0300) == 0x0300, "状态低16位 bit8~9 = 正在运行");
        check((st & 0x0002) != 0, "状态低16位 bit1 = X1 有输入");
    }

    /* 03H 响应解析：读位置 -8000（低字在前）：
     * 01 03 04 E0 C0 FF FF + CRC
     * 解析（与 robot_read_position_steps 一致）：(data[2..3]<<16)|data[0..1] = -8000 */
    rx[0] = 0x01; rx[1] = 0x03; rx[2] = 0x04;
    rx[3] = 0xE0; rx[4] = 0xC0; rx[5] = 0xFF; rx[6] = 0xFF;
    len = append_crc16(rx, 7);
    check(modbus_parse_response(rx, len, &resp) == ERR_NONE, "03H 位置响应 CRC 通过");
    {
        int32_t pos = (int32_t)(((uint32_t)resp.data[2] << 24) |
                                ((uint32_t)resp.data[3] << 16) |
                                ((uint32_t)resp.data[0] << 8) |
                                (uint32_t)resp.data[1]);
        check(pos == -8000, "03H 位置解析 = -8000（低字在前）");
    }

    /* ========== 04H 读单个寄存器：读实时电流 0x001A ========== */
    /* 请求帧：01 04 00 1A 00 01 + CRC（帧格式与 03H 相同，立三手册 V126 功能码 0x04） */
    len = modbus_build_read_input(0x01, 0x001A, frame);
    check(len == 8, "04H 帧长度");
    check(frame[0] == 0x01 && frame[1] == 0x04 && frame[2] == 0x00 &&
          frame[3] == 0x1A && frame[4] == 0x00 && frame[5] == 0x01,
          "04H 从站/功能码/地址/数量");
    check(modbus_check_crc(frame, len) == ERR_NONE, "04H 帧 CRC 自校验");

    /* 04H 响应解析：01 04 02 07 D0 + CRC（电流 0x07D0 = 2000 mA） */
    rx[0] = 0x01; rx[1] = 0x04; rx[2] = 0x02;
    rx[3] = 0x07; rx[4] = 0xD0;
    len = append_crc16(rx, 5);
    check(modbus_parse_response(rx, len, &resp) == ERR_NONE, "04H 响应 CRC 通过");
    check(resp.func == 0x04 && resp.data_len == 2 &&
          resp.data[0] == 0x07 && resp.data[1] == 0xD0,
          "04H 响应数据（0x07D0 = 2000 mA）");

    /* ========== 06H 写单寄存器：反向运行 0x00C8 = 0x0101 ========== */
    /* 请求帧：01 06 00 C8 01 01 + CRC(0x64C8) -> 帧尾 C8 64（手册 p16） */
    len = modbus_build_write_single(0x01, 0x00C8, 0x0101, frame);
    check(len == 8, "06H 帧长度");
    check(frame[1] == 0x06 && frame[2] == 0x00 && frame[3] == 0xC8 &&
          frame[4] == 0x01 && frame[5] == 0x01, "06H 内容");
    check(frame[6] == 0xC8 && frame[7] == 0x64, "06H CRC 低字节在前 (0x64C8)");

    /* 06H 响应回显解析（回显 = 请求帧） */
    memcpy(rx, frame, len);
    check(modbus_parse_response(rx, len, &resp) == ERR_NONE, "06H 回显解析");
    check(resp.reg_addr == 0x00C8 && resp.reg_count == 1, "06H 地址/数量");

    /* 06H 写使能 0x00D4 = 0（立三使能语义与旧 Zeta 相反） */
    len = modbus_build_write_single(0x01, 0x00D4, 0x0000, frame);
    check(frame[1] == 0x06 && frame[2] == 0x00 && frame[3] == 0xD4 &&
          frame[4] == 0x00 && frame[5] == 0x00, "06H 写 0x00D4=0（使能）");
    check(modbus_check_crc(frame, len) == ERR_NONE, "06H 使能帧 CRC 自校验");

    /* ========== 10H 写多寄存器：运行到绝对位置 0x00E8~0x00E9 ========== */
    /* 写 -8000 脉冲：立三 10H DWORD 低字在前 -> values={0xE0C0, 0xFFFF}
     * 帧数据 E0 C0 FF FF + CRC(0x0DCA) -> 帧尾 CA 0D（手册 p21 示例） */
    vals[0] = 0xE0C0; vals[1] = 0xFFFF;
    len = modbus_build_write_multi(0x01, 0x00E8, vals, 2, frame);
    check(len == 13, "10H 帧长度");
    check(frame[0] == 0x01 && frame[1] == 0x10 && frame[2] == 0x00 && frame[3] == 0xE8,
          "10H 从站/功能码/地址");
    check(frame[4] == 0x00 && frame[5] == 0x02 && frame[6] == 0x04, "10H 数量/字节数");
    check(frame[7] == 0xE0 && frame[8] == 0xC0 && frame[9] == 0xFF && frame[10] == 0xFF,
          "10H 数据（低字在前 E0 C0 FF FF）");
    check(frame[11] == 0xCA && frame[12] == 0x0D, "10H CRC 低字节在前 (0x0DCA)");

    /* 10H 响应回显解析：01 10 00 E8 00 02 + CRC */
    memcpy(rx, frame, 6);
    len = append_crc16(rx, 6);
    check(modbus_parse_response(rx, len, &resp) == ERR_NONE, "10H 回显解析");
    check(resp.reg_addr == 0x00E8 && resp.reg_count == 2, "10H 地址/数量");

    /* ========== 异常响应与坏 CRC ========== */
    /* 异常响应：01 83 02 错误码（功能码|0x80） */
    rx[0] = 0x01; rx[1] = 0x83; rx[2] = 0x02;
    len = append_crc16(rx, 3);
    check(modbus_parse_response(rx, len, &resp) == ERR_EXCEPTION, "异常响应识别");
    check(resp.data[0] == 0x02, "异常码提取");

    /* 坏 CRC */
    rx[0] = 0x01; rx[1] = 0x03; rx[2] = 0x02; rx[3] = 0x00; rx[4] = 0x01;
    rx[5] = 0x00; rx[6] = 0x00;
    check(modbus_parse_response(rx, 7, &resp) == ERR_CRC, "坏 CRC 拒绝");

    /* ========== 方案一：CommOps 注入后帧收发一致（内存假串口） ========== */
    modbus_comm_set(&g_fake_comm_ops);

    /* 06H 写使能 0x00D4=0 请求，从站回显同帧（Modbus 06H 响应=请求回显） */
    len = modbus_build_write_single(0x01, 0x00D4, 0x0000, frame);
    fake_set_response(frame, len);
    check(modbus_transact(frame, len, &resp) == ERR_NONE, "假串口 transact 成功");
    check(g_fake_tx_len == len && memcmp(g_fake_tx, frame, len) == 0,
          "假串口收到完整请求帧（写帧一致）");
    check(resp.reg_addr == 0x00D4 && resp.reg_count == 1 && resp.data[0] == 0x00 &&
          resp.data[1] == 0x00, "假串口解析 06H 回显（地址/值一致）");

    /* 未注入总线时 transact 应返回 ERR_PORT（隔离真实串口） */
    modbus_comm_set(NULL);
    check(modbus_transact(frame, len, &resp) == ERR_PORT, "未注入总线返回 ERR_PORT");
    modbus_comm_set(&g_fake_comm_ops);

    /* ========== err_str 中文描述覆盖全枚举 ========== */
    check(err_str(ERR_NONE) != NULL && err_str(ERR_NONE)[0] != '\0', "err_str(ERR_NONE) 非空");
    check(strstr(err_str(ERR_PORT), "串口") != NULL, "err_str(ERR_PORT) 含串口");
    check(strstr(err_str(ERR_TIMEOUT), "超时") != NULL, "err_str(ERR_TIMEOUT) 含超时");
    check(strstr(err_str(ERR_CRC), "CRC") != NULL, "err_str(ERR_CRC) 含CRC");
    check(strstr(err_str(ERR_EXCEPTION), "异常") != NULL, "err_str(ERR_EXCEPTION) 含异常");
    check(strstr(err_str(ERR_LEN), "帧") != NULL, "err_str(ERR_LEN) 含帧");
    check(strstr(err_str(ERR_ARG), "参数") != NULL, "err_str(ERR_ARG) 含参数");
    check(strstr(err_str(ERR_OVERFLOW), "越界") != NULL, "err_str(ERR_OVERFLOW) 含越界");
    check(strstr(err_str((ErrCode)99), "未知") != NULL, "err_str(未知码) 返回未知错误");

    if (g_fail == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("共 %d 项失败\n", g_fail);
    return 1;
}
