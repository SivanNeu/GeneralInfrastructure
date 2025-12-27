#include "utils/FeatureList.h"
#include "utils/NED.h"
#include <sstream>

Feature_List::Feature_List(int frame_id, int64_t ts)
    : current_index(0), frame_id(frame_id), ts(ts) {
}

size_t Feature_List::size() const {
    return features.size();
}

void Feature_List::append(const Feature& feature) {
    features.push_back(feature);
    features_dict[feature.id] = feature;
}

std::pair<bool, Feature> Feature_List::get_feature_by_id(int id) const {
    auto it = features_dict.find(id);
    if (it != features_dict.end()) {
        return std::make_pair(true, it->second);
    } else {
        return std::make_pair(false, Feature(0, 0, -1));
    }
}

void Feature_List::append_pt(double x, double y, NED* ned_pos) {
    features.push_back(Feature(x, y, current_index, ned_pos));
    features_dict[current_index] = features.back();
    current_index++;
}

std::string Feature_List::to_string() const {
    std::ostringstream oss;
    for (const auto& feature : features) {
        oss << feature.to_string() << "\n";
    }
    return oss.str();
}

