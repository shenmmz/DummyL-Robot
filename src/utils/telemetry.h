#ifndef TELEMETRY_H
#define TELEMETRY_H

/*
 * telemetry —— 把关节角通过 UDP 回环发给本机 3D 镜像（sim/live_mirror.py）。
 *
 * 【设计红线】遥测永远不能影响控制：
 *   - 默认【关闭】。ini [telemetry] enabled = 1 才启用。
 *   - 所有失败路径静默禁用（不打印、不报错、不重试）。
 *   - sendto 是微秒级非阻塞操作，且设了 1ms 发送超时；没人监听时内核直接丢包。
 *   - 不在控制循环里做任何磁盘 I/O。
 *
 * 用法：
 *   telemetry_init(INI_PATH);            // main() 里调一次
 *   telemetry_send(angles, "cmd");       // 下发点调
 *   telemetry_shutdown();                // 退出前调
 */
void telemetry_init(const char *ini_path);
void telemetry_send(const double deg[6], const char *tag);
void telemetry_shutdown(void);

#endif /* TELEMETRY_H */
