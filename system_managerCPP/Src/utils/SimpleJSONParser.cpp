#include "utils/SimpleJSONParser.h"
#include <sstream>
#include <cctype>
#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <set>

void SimpleJSONParser::skip_whitespace() {
    while (pos < content.size() && std::isspace(content[pos])) pos++;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_value() {
    skip_whitespace();
    if (pos >= content.size()) return nullptr;
    
    char c = content[pos];
    if (c == '{') return parse_object();
    if (c == '[') return parse_array();
    if (c == '"') return parse_string();
    if (c == 't' || c == 'f') return parse_bool();
    if (c == 'n') return parse_null();
    if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
    return nullptr;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_object() {
    auto obj = std::make_shared<RLPolicyClean::JSONValue>();
    obj->type = RLPolicyClean::JSONValue::OBJECT;
    
    pos++; // skip '{'
    skip_whitespace();
    
    while (pos < content.size() && content[pos] != '}') {
        skip_whitespace();
        if (content[pos] == '}') break;
        
        auto key_val = parse_string();
        if (!key_val) break;
        std::string key = key_val->string_val;
        
        skip_whitespace();
        if (pos >= content.size() || content[pos] != ':') break;
        pos++; // skip ':'
        
        auto val = parse_value();
        if (val) {
            obj->object[key] = val;
        }
        
        skip_whitespace();
        if (pos < content.size() && content[pos] == ',') {
            pos++; // skip ','
        }
    }
    
    if (pos < content.size() && content[pos] == '}') pos++;
    return obj;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_array() {
    auto arr = std::make_shared<RLPolicyClean::JSONValue>();
    arr->type = RLPolicyClean::JSONValue::ARRAY;
    
    pos++; // skip '['
    skip_whitespace();
    
    while (pos < content.size() && content[pos] != ']') {
        skip_whitespace();
        if (content[pos] == ']') break;
        
        auto val = parse_value();
        if (val) {
            arr->array.push_back(val);
        }
        
        skip_whitespace();
        if (pos < content.size() && content[pos] == ',') {
            pos++; // skip ','
        }
    }
    
    if (pos < content.size() && content[pos] == ']') pos++;
    return arr;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_string() {
    auto str = std::make_shared<RLPolicyClean::JSONValue>();
    str->type = RLPolicyClean::JSONValue::STRING;
    
    if (pos >= content.size() || content[pos] != '"') return nullptr;
    pos++; // skip opening quote
    
    std::stringstream ss;
    while (pos < content.size() && content[pos] != '"') {
        if (content[pos] == '\\' && pos + 1 < content.size()) {
            pos++;
            if (content[pos] == 'n') ss << '\n';
            else if (content[pos] == 't') ss << '\t';
            else if (content[pos] == 'r') ss << '\r';
            else if (content[pos] == '\\') ss << '\\';
            else if (content[pos] == '"') ss << '"';
            else ss << content[pos];
        } else {
            ss << content[pos];
        }
        pos++;
    }
    
    if (pos < content.size() && content[pos] == '"') pos++;
    str->string_val = ss.str();
    return str;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_number() {
    auto num = std::make_shared<RLPolicyClean::JSONValue>();
    num->type = RLPolicyClean::JSONValue::NUMBER;
    
    size_t start = pos;
    
    if (pos < content.size() && content[pos] == '-') pos++;
    while (pos < content.size() && (std::isdigit(content[pos]) || content[pos] == '.' || content[pos] == 'e' || content[pos] == 'E' || content[pos] == '+' || content[pos] == '-')) {
        pos++;
    }
    
    std::string num_str = content.substr(start, pos - start);
    num->number_val = std::stod(num_str);
    return num;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_bool() {
    auto b = std::make_shared<RLPolicyClean::JSONValue>();
    b->type = RLPolicyClean::JSONValue::BOOL;
    
    if (content.substr(pos, 4) == "true") {
        b->bool_val = true;
        pos += 4;
    } else if (content.substr(pos, 5) == "false") {
        b->bool_val = false;
        pos += 5;
    }
    return b;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse_null() {
    if (content.substr(pos, 4) == "null") {
        pos += 4;
        auto n = std::make_shared<RLPolicyClean::JSONValue>();
        n->type = RLPolicyClean::JSONValue::NULL_VAL;
        return n;
    }
    return nullptr;
}

// Base64 decoding helper functions
bool SimpleJSONParser::is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}

std::vector<uint8_t> SimpleJSONParser::base64_decode(const std::string& encoded_string) {
    const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<uint8_t> result;
    int in_len = static_cast<int>(encoded_string.size());
    int i = 0;
    int in = 0;
    unsigned char char_array_4[4], char_array_3[3];
    
    while (in_len-- && (encoded_string[in] != '=') && is_base64(encoded_string[in])) {
        char_array_4[i++] = encoded_string[in]; in++;
        if (i == 4) {
            for (i = 0; i < 4; i++) {
                size_t pos = chars.find(char_array_4[i]);
                if (pos != std::string::npos) {
                    char_array_4[i] = static_cast<unsigned char>(pos);
                } else {
                    char_array_4[i] = 0;
                }
            }
            
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
            
            for (i = 0; (i < 3); i++)
                result.push_back(char_array_3[i]);
            i = 0;
        }
    }
    
    if (i) {
        for (int j = i; j < 4; j++)
            char_array_4[j] = 0;
        
        for (int j = 0; j < 4; j++) {
            size_t pos = chars.find(char_array_4[j]);
            if (pos != std::string::npos) {
                char_array_4[j] = static_cast<unsigned char>(pos);
            } else {
                char_array_4[j] = 0;
            }
        }
        
        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
        
        for (int j = 0; (j < i - 1); j++) result.push_back(char_array_3[j]);
    }
    
    return result;
}

std::shared_ptr<RLPolicyClean::JSONValue> SimpleJSONParser::parse(const std::string& json_str) {
    content = json_str;
    pos = 0;
    return parse_value();
}

bool SimpleJSONParser::parse_file(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << json_path << std::endl;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    json_data_ = parse(buffer.str());
    
    if (!json_data_ || json_data_->type != RLPolicyClean::JSONValue::OBJECT) {
        std::cerr << "Error: Failed to parse JSON file" << std::endl;
        return false;
    }
    
    return true;
}

bool SimpleJSONParser::extract_tensor(const std::string& key, std::vector<float>& data, std::vector<int>& shape) {
    if (!json_data_ || json_data_->type != RLPolicyClean::JSONValue::OBJECT) return false;
    
    std::shared_ptr<RLPolicyClean::JSONValue> weights;
    
    // First try direct weights access
    auto weights_it = json_data_->object.find("weights");
    if (weights_it != json_data_->object.end()) {
        weights = weights_it->second;
    } else {
        // Check if there's a "model" wrapper
        auto model_it = json_data_->object.find("model");
        if (model_it != json_data_->object.end() && model_it->second->type == RLPolicyClean::JSONValue::OBJECT) {
            auto model_weights_it = model_it->second->object.find("weights");
            if (model_weights_it != model_it->second->object.end()) {
                weights = model_weights_it->second;
            } else {
                weights = model_it->second;
            }
        }
    }
    
    if (!weights || weights->type != RLPolicyClean::JSONValue::OBJECT) {
        return false;
    }
    
    auto key_it = weights->object.find(key);
    if (key_it == weights->object.end()) {
        return false;
    }
    
    auto tensor_obj = key_it->second;
    if (tensor_obj->type != RLPolicyClean::JSONValue::OBJECT) 
        return false;
    
    // Extract shape
    auto shape_it = tensor_obj->object.find("shape");
    if (shape_it == tensor_obj->object.end()) return false;
    auto shape_val = shape_it->second;
    if (shape_val->type != RLPolicyClean::JSONValue::ARRAY) return false;
    
    shape.clear();
    for (auto& dim : shape_val->array) {
        if (dim->type == RLPolicyClean::JSONValue::NUMBER) {
            shape.push_back(static_cast<int>(dim->number_val));
        }
    }
    
    // Extract data - check if it's base64 encoded or array format
    auto data_it = tensor_obj->object.find("data");
    if (data_it == tensor_obj->object.end()) return false;
    auto data_val = data_it->second;
    
    // Check encoding format
    auto encoding_it = tensor_obj->object.find("encoding");
    bool is_base64_encoded = false;
    if (encoding_it != tensor_obj->object.end() && 
        encoding_it->second->type == RLPolicyClean::JSONValue::STRING &&
        encoding_it->second->string_val == "base64") {
        is_base64_encoded = true;
    }
    
    data.clear();
    
    if (is_base64_encoded) {
        // Decode base64 string to bytes, then convert to floats
        if (data_val->type != RLPolicyClean::JSONValue::STRING) {
            std::cerr << "Error: Expected string for base64 data" << std::endl;
            return false;
        }
        
        // Decode base64
        std::vector<uint8_t> decoded_bytes = base64_decode(data_val->string_val);
        
        // Get dtype to determine element size
        auto dtype_it = tensor_obj->object.find("dtype");
        std::string dtype = "float32";  // default
        if (dtype_it != tensor_obj->object.end() && dtype_it->second->type == RLPolicyClean::JSONValue::STRING) {
            dtype = dtype_it->second->string_val;
        }
        
        // Determine element size based on dtype
        size_t element_size = 4;  // default to float32
        if (dtype.find("float64") != std::string::npos || dtype.find("double") != std::string::npos) {
            element_size = 8;
        } else if (dtype.find("float32") != std::string::npos || dtype.find("float") != std::string::npos) {
            element_size = 4;
        } else if (dtype.find("float16") != std::string::npos || dtype.find("half") != std::string::npos) {
            element_size = 2;
        }
        
        // Convert bytes to floats (assuming little-endian, native byte order)
        size_t num_elements = decoded_bytes.size() / element_size;
        data.reserve(num_elements);
        
        if (element_size == 4) {
            // float32 - direct copy is most efficient
            data.resize(num_elements);
            std::memcpy(data.data(), decoded_bytes.data(), num_elements * sizeof(float));
        } else {
            // For other sizes, convert element by element
            for (size_t i = 0; i < num_elements; i++) {
                float val = 0.0f;
                if (element_size == 8) {
                    // float64 -> convert to float32
                    double val_double = 0.0;
                    std::memcpy(&val_double, &decoded_bytes[i * 8], sizeof(double));
                    val = static_cast<float>(val_double);
                } else if (element_size == 2) {
                    // float16 -> convert to float32 (simplified)
                    uint16_t bits = 0;
                    std::memcpy(&bits, &decoded_bytes[i * 2], sizeof(uint16_t));
                    val = static_cast<float>(bits) / 65536.0f;  // rough approximation
                }
                data.push_back(val);
            }
        }
    } else {
        // Old format: extract from nested array
        extract_flattened_array(data_val, data);
    }
    
    return true;
}

void SimpleJSONParser::extract_flattened_array(std::shared_ptr<RLPolicyClean::JSONValue> val, std::vector<float>& result) {
    if (!val) return;
    if (val->type == RLPolicyClean::JSONValue::ARRAY) {
        for (auto& item : val->array) {
            extract_flattened_array(item, result);
        }
    } else if (val->type == RLPolicyClean::JSONValue::NUMBER) {
        result.push_back(static_cast<float>(val->number_val));
    }
}

bool SimpleJSONParser::find_mlp_linear_indices(std::vector<int>& indices) {
    indices.clear();
    if (!json_data_ || json_data_->type != RLPolicyClean::JSONValue::OBJECT) return false;
    
    auto weights_it = json_data_->object.find("weights");
    if (weights_it == json_data_->object.end()) return false;
    
    auto weights = weights_it->second;
    if (weights->type != RLPolicyClean::JSONValue::OBJECT) return false;
    
    std::set<int> index_set;
    std::string prefix = "encoder.encoders.obs.mlp_head.";
    
    for (const auto& pair : weights->object) {
        const std::string& key = pair.first;
        if (key.find(prefix) == 0 && key.find(".weight") != std::string::npos) {
            size_t idx_start = prefix.length();
            size_t idx_end = key.find('.', idx_start);
            if (idx_end != std::string::npos) {
                try {
                    int idx = std::stoi(key.substr(idx_start, idx_end - idx_start));
                    index_set.insert(idx);
                } catch (...) {
                    continue;
                }
            }
        }
    }
    
    indices.assign(index_set.begin(), index_set.end());
    std::sort(indices.begin(), indices.end());
    return true;
}
