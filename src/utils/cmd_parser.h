#ifndef CMD_PARSER_H
#define CMD_PARSER_H


#define CMD_UNKNOWN   0
#define CMD_HOME      1
#define CMD_MOVEJ     2
#define CMD_DISABLE   3
#define CMD_ENABLE    4
#define CMD_MOTOR     5
#define CMD_HELP      6
#define CMD_EXIT      7
#define CMD_EMPTY     8
#define CMD_ZERO      9
#define CMD_GETPOS    10
#define CMD_ZERO_SAVE 11
#define CMD_MOVEL     13
#define CMD_FK        14
#define CMD_DIAG      15
#define CMD_BCAST     16
#define CMD_NRTEST    17
#define CMD_CURTEST   18
#define CMD_STALL     19
#define CMD_POSEOK    20
#define CMD_BUSRATE   22
#define CMD_TABTEST   21
#define CMD_ACCEL     23
#define CMD_ALARM     24
#define CMD_LOOPTEST  25
#define CMD_DRVBAUD   26

typedef enum {
    MOVL_MODE_SYNC = 0,
    MOVL_MODE_STEP = 1,
    MOVL_MODE_STREAM = 2,
    MOVL_MODE_SMOOTH = 3
} MovlMode;

typedef struct {
    int      type;
    int      joint;
    double   angle_deg;
    double   speed_rpm;
    int      rel;
    double   zero_vals[6];
    int      num_joints;
    int      joints[6];
    double   angles[6];
    double   speeds[6];
    int      accel_ms[6];
    int      decel_ms[6];
    double   cartesian[6];
    int      keep_pose;
    int      movl_mode;
    double   param;
    char     raw[128];
} ParsedCmd;

int cmd_parse(const char *line, ParsedCmd *out);

void cmd_print_help(void);

#endif