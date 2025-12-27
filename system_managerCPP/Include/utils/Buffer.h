#ifndef BUFFER_H
#define BUFFER_H
#include "general.h"

#include <vector>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <cmath>

template<typename T>
class Buffer {
private:
    size_t _size;
    size_t _next_index;
    std::vector<T> _data_list;
    std::vector<int64_t> _ts_list;
    bool _use_ts_list;

    size_t _increment_index(size_t index) const {
        if (index + 1 >= _data_list.size()) {
            return 0;
        } else {
            return index + 1;
        }
    }

    size_t _decrement_index(size_t index) const {
        if (index == 0) {
            return _data_list.size() - 1;
        } else {
            return index - 1;
        }
    }

public:
    Buffer(size_t size, bool data_is_timestamped = false)
        : _size(size), _next_index(0), _use_ts_list(data_is_timestamped) {
    }

    bool add(const T& data, int64_t ts = 0) {
        if (_use_ts_list) {
            if (std::find(_ts_list.begin(), _ts_list.end(), ts) != _ts_list.end()) {
                return false;
            }
        }

        if (_data_list.size() < _size) {
            _data_list.push_back(data);
            if (_use_ts_list) {
                _ts_list.push_back(ts);
            }
        } else {
            _data_list[_next_index] = data;
            if (_use_ts_list) {
                _ts_list[_next_index] = ts;
            }
        }
        _next_index = _increment_index(_next_index);
        return true;
    }

    std::tuple<bool, int64_t, T> get_closest_data_to_ts(int64_t ts) const {
        if (!_use_ts_list) {
            return std::make_tuple(false, 0, T());
        }
        if (_ts_list.empty() || _data_list.empty()) {
            return std::make_tuple(false, 0, T());
        }

        size_t current_index = _decrement_index(_next_index);
        double prev_diff = 0.0;
        bool first = true;

        for (size_t i = 0; i < _ts_list.size(); i++) {
            double diff = std::abs(static_cast<double>(_ts_list[current_index] - ts));
            if ((diff >= prev_diff && !first) || ts == _ts_list[current_index]) {
                break;
            }
            prev_diff = diff;
            current_index = _decrement_index(current_index);
            if (first) {
                first = false;
            }
        }
        current_index = _increment_index(current_index);

        return std::make_tuple(true, _ts_list[current_index], _data_list[current_index]);
    }

    T tail() const {
        if (_data_list.empty()) {
            return T();
        }
        return _data_list[_next_index];
    }

    T head() const {
        if (_data_list.empty()) {
            return T();
        }
        return _data_list[_next_index];
    }

    std::pair<std::vector<T>, std::vector<int64_t>> get_buffer() const {
        if (_use_ts_list) {
            return std::make_pair(_data_list, _ts_list);
        } else {
            return std::make_pair(_data_list, std::vector<int64_t>());
        }
    }

    std::string to_string() const {
        std::string str_out = std::to_string(_next_index) + "\n";
        size_t max_index = 0;
        for (size_t i = 0; i < _ts_list.size(); i++) {
            str_out += std::to_string(_ts_list[i]) + ":" + std::to_string(i) + "\n";
            max_index = i;
        }
        str_out += "\n" + std::to_string(max_index);
        return str_out;
    }
};

#endif // BUFFER_H

