#ifndef FEATURE_LIST_H
#define FEATURE_LIST_H

#include "utils/Feature.h"
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <string>

class Feature_List {
public:
    std::vector<Feature> features;
    std::unordered_map<int, Feature> features_dict;
    int current_index;
    int frame_id;
    int64_t ts;

    Feature_List(int frame_id = 0, int64_t ts = 0);
    size_t size() const;
    void append(const Feature& feature);
    std::pair<bool, Feature> get_feature_by_id(int id) const;
    void append_pt(double x, double y, NED* ned_pos = nullptr);
    std::string to_string() const;
};

#endif // FEATURE_LIST_H

