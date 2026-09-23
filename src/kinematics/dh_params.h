#ifndef DH_PARAMS_H
#define DH_PARAMS_H


#include <stddef.h>

#define DH_JOINT_COUNT 6

#define DH_D6_FLANGE_MM 91.5

typedef struct {
    double a;
    double alpha;
    double d;
    double theta_offset;
} DhParam;

extern DhParam DH_TABLE[DH_JOINT_COUNT];

#endif
