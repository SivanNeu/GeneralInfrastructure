#ifndef SIMPLE_JSON_PARSER_H
#define SIMPLE_JSON_PARSER_H

#include "RLPolicyClean.h"
#include <string>
#include <memory>
#include <vector>
#include <set>

/**
 * SimpleJSONParser - A minimal JSON parser for parsing neural network weight files
 * Handles the specific structure from pth2json.py
 */
class SimpleJSONParser {
private:
    std::string content;
    size_t pos;
    std::shared_ptr<RLPolicyClean::JSONValue> json_data_;
    
    // Base64 decoding helpers
    static inline bool is_base64(unsigned char c);
    static std::vector<uint8_t> base64_decode(const std::string& encoded_string);
    
    // JSON parsing methods
    void skip_whitespace();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_value();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_object();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_array();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_string();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_number();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_bool();
    std::shared_ptr<RLPolicyClean::JSONValue> parse_null();
    
    // Helper methods for extracting data
    void extract_flattened_array(std::shared_ptr<RLPolicyClean::JSONValue> val, std::vector<float>& result);

public:
    /**
     * Parse a JSON string and return the root JSONValue
     * @param json_str The JSON string to parse
     * @return Shared pointer to the root JSONValue, or nullptr on error
     */
    std::shared_ptr<RLPolicyClean::JSONValue> parse(const std::string& json_str);
    
    /**
     * Parse a JSON file and store the parsed data
     * @param json_path Path to the JSON file
     * @return true if parsing succeeded, false otherwise
     */
    bool parse_file(const std::string& json_path);
    
    /**
     * Extract tensor data from the parsed JSON
     * @param key The tensor key (e.g., "core.core.weight_ih_l0")
     * @param data Output vector for tensor data
     * @param shape Output vector for tensor shape
     * @return true if extraction succeeded, false otherwise
     */
    bool extract_tensor(const std::string& key, std::vector<float>& data, std::vector<int>& shape);
    
    /**
     * Find MLP linear layer indices from encoder structure
     * @param indices Output vector for layer indices
     * @return true if indices were found, false otherwise
     */
    bool find_mlp_linear_indices(std::vector<int>& indices);
    
    /**
     * Get the parsed JSON data
     * @return Shared pointer to the root JSONValue
     */
    std::shared_ptr<RLPolicyClean::JSONValue> get_json_data() const { return json_data_; }
};

#endif // SIMPLE_JSON_PARSER_H
