#include "low_pass_filter.h"
#include <stdio.h>
#include <math.h>

void lpf_init(low_pass_filter_t* filter, double alpha, bool is_angle, lpf_type_t type) {
    filter->type = type;
    filter->is_angle = is_angle;
    filter->initialized = false;
    
    if (type == LPF_TYPE_FIRST_ORDER) {
        if (alpha > 1.0) {
            printf("alpha must be less than 1\n");
            alpha = 1.0;
        }
        if (alpha < 0.0) {
            alpha = 0.0;
            printf("alpha must be greater than 0\n");
        }
    }
    filter->alpha = alpha;
    filter->prev_filtered_output = 0.0;
}

static double lpf_first_order_step(low_pass_filter_t* filter, double data_pt) {
    if (!filter->initialized) {
        filter->prev_filtered_output = data_pt;
        filter->initialized = true;
        return data_pt;
    }
    
    if (filter->is_angle) {
        double diff = angle_diff_rad((1.0 - filter->alpha) * filter->prev_filtered_output, 
                                     -(filter->alpha) * data_pt);
        filter->prev_filtered_output = diff;
    } else {
        filter->prev_filtered_output = (1.0 - filter->alpha) * filter->prev_filtered_output + 
                                        (filter->alpha) * data_pt;
    }
    return filter->prev_filtered_output;
}

static double lpf_other_step(low_pass_filter_t* filter, double data_pt) {
    if (!filter->initialized) {
        filter->prev_filtered_output = data_pt;
        filter->initialized = true;
        return data_pt;
    }
    
    if (filter->is_angle) {
        double diff = angle_diff_rad(data_pt, filter->prev_filtered_output);
        if (fabs(diff) > filter->alpha) {
            if (diff < 0) {
                filter->prev_filtered_output = angle_diff_rad(filter->prev_filtered_output, filter->alpha);
            } else {
                filter->prev_filtered_output = angle_diff_rad(filter->prev_filtered_output, -filter->alpha);
            }
        } else {
            filter->prev_filtered_output = data_pt;
        }
    }
    return filter->prev_filtered_output;
}

double lpf_step(low_pass_filter_t* filter, double data_pt) {
    if (filter->type == LPF_TYPE_FIRST_ORDER) {
        return lpf_first_order_step(filter, data_pt);
    } else if (filter->type == LPF_TYPE_OTHER) {
        return lpf_other_step(filter, data_pt);
    }
    return data_pt;
}

