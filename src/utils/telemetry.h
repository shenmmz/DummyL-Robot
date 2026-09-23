#ifndef TELEMETRY_H
#define TELEMETRY_H

void telemetry_init(const char *ini_path);
void telemetry_send(const double deg[6], const char *tag);
void telemetry_shutdown(void);

#endif
