/*
 * udp_telemetry_test.c —— 不接硬件，验证 C 端 UDP 遥测真的能发出去
 * ------------------------------------------------------------------
 * 用法（项目根目录）：
 *   1) 先开接收端：sim\.venv\Scripts\python.exe sim\live_mirror.py
 *      或只验链路：python -c "import socket;s=socket.socket(2,2);s.bind(('127.0.0.1',9900));print(s.recvfrom(4096))"
 *   2) 再跑本程序：build\bin\udp_telemetry_test.exe
 * 期望：接收端收到 3 包，q1 分别是 0 / 10 / 20。
 */
#include "utils/telemetry.h"
#include <stdio.h>

int main(void)
{
    double q[6] = {0.0, 0.0, 110.0, 0.0, 70.0, 0.0};
    int i;

    telemetry_init("src/config/robot_config.ini");
    for (i = 0; i < 3; i++) {
        q[0] = i * 10.0;
        telemetry_send(q, "cmd");
    }
    telemetry_shutdown();
    printf("已尝试发送 3 包（q1 = 0 / 10 / 20）。\n");
    printf("若 ini [telemetry] enabled=0，则一包都不会发 —— 这是设计行为，不是 bug。\n");
    return 0;
}
