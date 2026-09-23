#ifndef TELEMETRY_H
#define TELEMETRY_H

/* 按 ini 配置初始化遥测（没配就静默不做事，不报错）。 */
void telemetry_init(const char *ini_path);
/* 发一帧六轴角度 + 标签。热路径，内部不阻塞。 */
void telemetry_send(const double deg[6], const char *tag);
/* 收尾。 */
void telemetry_shutdown(void);

#endif
