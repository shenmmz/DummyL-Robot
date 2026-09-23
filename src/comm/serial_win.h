#ifndef SERIAL_WIN_H
#define SERIAL_WIN_H


#include <stdint.h>
#include <stddef.h>

#include "comm/comm_if.h"

typedef struct SerialPort SerialPort;
SerialPort *serial_open(const char *port_name, uint32_t baudrate);

void serial_close(SerialPort *port);

int serial_write(SerialPort *port, const uint8_t *data, size_t len);

int serial_read(SerialPort *port, uint8_t *buf, size_t max_len, uint32_t timeout_ms);

void serial_set_timeout(SerialPort *port, uint32_t read_ms, uint32_t write_ms);

void serial_flush(SerialPort *port);

int serial_is_open(const SerialPort *port);

int serial_set_baud(uint32_t baudrate);

uint32_t serial_get_baud(void);

int serial_set_rts(int mode);

int serial_get_rts(void);

extern const CommOps serial_comm_ops;

#endif
