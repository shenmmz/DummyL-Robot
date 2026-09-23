#ifndef ERR_H
#define ERR_H


typedef enum {
    ERR_NONE      = 0,
    ERR_PORT      = 1,
    ERR_TIMEOUT   = 2,
    ERR_CRC       = 3,
    ERR_EXCEPTION = 4,
    ERR_LEN       = 5,
    ERR_ARG       = 6,
    ERR_OVERFLOW  = 7,
    ERR_MASKED    = 8,
    ERR_ALARM     = 9,
    ERR_SUBDIV    = 10,
} ErrCode;

/* 错误码转中文串。ERR_NONE → "OK"。 */
const char *err_str(ErrCode e);

#endif
