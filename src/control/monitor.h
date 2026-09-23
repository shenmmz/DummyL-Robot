#ifndef MONITOR_H
#define MONITOR_H


#include "control/robot.h"
#include <stdint.h>

typedef struct Monitor Monitor;

#define MONITOR_DEFAULT_INTERVAL_MS 300u

#define MONITOR_PARK_TIMEOUT_MS 500u

Monitor *monitor_create(Robot *robot, const int stall_threshold_ma[6]);

int monitor_stall_hit(int cur_ma, int threshold_ma);

void monitor_set_stall_threshold(Monitor *m, int joint, int ma);
int  monitor_get_stall_threshold(const Monitor *m, int joint);

void monitor_destroy(Monitor *m);

int monitor_start(Monitor *m, int interval_ms);

void monitor_stop(Monitor *m);

int monitor_is_running(const Monitor *m);

void monitor_pause(Monitor *m, int on);
void monitor_pause_active(int on);

void monitor_park(void);
int  monitor_park_wait(int timeout_ms);

long monitor_poll_count(void);

int monitor_poll(Monitor *m);

int monitor_check_stall(Monitor *m, int joint);

int monitor_online_count(const Monitor *m);

typedef struct MonitorSnapshot {
    int      online;
    uint32_t status;
    int      current_ma;
    int      alarm_code;
} MonitorSnapshot;

ErrCode monitor_snapshot(Monitor *m, int joint, MonitorSnapshot *out);

#endif
