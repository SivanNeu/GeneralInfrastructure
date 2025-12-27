#include "utils/LowPassFilter.h"
#include "utils/Utils.h"
#include <cmath>
#include <iostream>

Low_Pass_Filter::Low_Pass_Filter(double alpha, bool is_angle, LPF_TYPE type)
    : _is_angle(is_angle), _type(type), _prev_filtered_output(std::nullopt) {
    
    if (_type == LPF_TYPE::FIRST_ORDER) {
        if (alpha > 1.0) {
            std::cout << "alpha must be less than 1" << std::endl;
            alpha = 1.0;
        }
        if (alpha < 0.0) {
            alpha = 0.0;
            std::cout << "alpha must be greater than 0" << std::endl;
        }
    }
    _alpha = alpha;
}

double Low_Pass_Filter::step(double data_pt) {
    if (_type == LPF_TYPE::FIRST_ORDER) {
        return _first_order_step(data_pt);
    } else if (_type == LPF_TYPE::OTHER) {
        return _other_step(data_pt);
    }
    return data_pt;  // Default return if type is invalid
}

double Low_Pass_Filter::_other_step(double data_pt) {
    if (!_prev_filtered_output.has_value()) {
        _prev_filtered_output = data_pt;
        return data_pt;
    }
    
    if (_is_angle) {
        double diff = Utils::angle_diff_rad(data_pt, _prev_filtered_output.value());
        std::cout << "diff " << diff << " alpha " << _alpha << std::endl;
        
        if (std::abs(diff) > _alpha) {
            if (diff < 0) {
                _prev_filtered_output = Utils::angle_diff_rad(_prev_filtered_output.value(), _alpha);
            } else {
                _prev_filtered_output = Utils::angle_diff_rad(_prev_filtered_output.value(), -_alpha);
            }
        } else {
            _prev_filtered_output = data_pt;
        }
    }
    return _prev_filtered_output.value();
}

double Low_Pass_Filter::_first_order_step(double data_pt) {
    if (!_prev_filtered_output.has_value()) {
        _prev_filtered_output = data_pt;
        return data_pt;
    }
    
    if (_is_angle) {
        // For angles: use angle_diff_rad to handle wrapping
        // Formula: angle_diff_rad((1-alpha) * prev, -(alpha) * data_pt)
        double prev_term = (1.0 - _alpha) * _prev_filtered_output.value();
        double data_term = -_alpha * data_pt;
        _prev_filtered_output = Utils::angle_diff_rad(prev_term, data_term);
    } else {
        // Standard first-order filter: output = (1-alpha) * prev + alpha * data
        _prev_filtered_output = (1.0 - _alpha) * _prev_filtered_output.value() + _alpha * data_pt;
    }
    return _prev_filtered_output.value();
}

