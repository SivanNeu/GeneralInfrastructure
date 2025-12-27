#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H
#include "general.h"

#include "utils/LpfType.h"
#include <optional>
#include <iostream>

// LPF: data = f(n-1)*a + (1-a)*f(n)
class Low_Pass_Filter {
private:
    bool _is_angle;
    LPF_TYPE _type;
    std::optional<double> _prev_filtered_output;
    double _alpha;

    double _first_order_step(double data_pt);
    double _other_step(double data_pt);

public:
    Low_Pass_Filter(double alpha, bool is_angle, LPF_TYPE type = LPF_TYPE::FIRST_ORDER);
    double step(double data_pt);
};

#endif // LOW_PASS_FILTER_H

