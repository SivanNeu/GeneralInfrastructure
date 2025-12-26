#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include "common.h"

typedef enum {
    LPF_TYPE_FIRST_ORDER = 0,
    LPF_TYPE_OTHER = 1
} lpf_type_t;

typedef struct {
    lpf_type_t type;
    bool is_angle;
    double alpha;
    double prev_filtered_output;
    bool initialized;
} low_pass_filter_t;

void lpf_init(low_pass_filter_t* filter, double alpha, bool is_angle, lpf_type_t type);
double lpf_step(low_pass_filter_t* filter, double data_pt);

#endif // LOW_PASS_FILTER_H

