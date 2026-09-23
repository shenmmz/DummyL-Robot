#ifndef COMM_IF_H
#define COMM_IF_H


#include <stdint.h>

typedef struct {
    int (*open)(const char *port, uint32_t baud);

    void (*close)(void);

    int (*read_frame)(uint8_t *buf, int cap, int timeout_ms);

    int (*write_frame)(const uint8_t *buf, int len);

    void (*flush)(void);
} CommOps;

#endif
