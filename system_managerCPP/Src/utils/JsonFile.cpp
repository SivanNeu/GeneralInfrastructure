#include "utils/JsonFile.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <vector>

JsonFile::JsonFile(const std::string& jsonFilePath) : filePath(jsonFilePath), valid(false) {
    std::ifstream file(jsonFilePath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << jsonFilePath << std::endl;
        std::cerr << "  Error: " << std::strerror(errno) << std::endl;
        return;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string rawContent = buffer.str();
    file.close();
    
    // Strip comments from JSON content
    content = stripComments(rawContent);
    valid = true;
}

std::string JsonFile::stripComments(const std::string& rawContent) {
    std::stringstream result;
    std::istringstream stream(rawContent);
    std::string line;
    
    while (std::getline(stream, line)) {
        // Find // comment marker
        size_t commentPos = line.find("//");
        if (commentPos != std::string::npos) {
            // Remove everything after //
            line = line.substr(0, commentPos);
        }
        result << line;
        if (!stream.eof()) {
            result << "\n";
        }
    }
    return result.str();
}

double JsonFile::getDouble(const std::string& key, double defaultValue) const {
    if (!valid) return defaultValue;
    
    std::regex regex("\"" + key + "\"\\s*:\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)");
    std::smatch match;
    if (std::regex_search(content, match, regex)) {
        try {
            return std::stod(match[1].str());
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse double for key '" << key << "', using default" << std::endl;
        }
    }
    return defaultValue;
}

std::string JsonFile::getString(const std::string& key, const std::string& defaultValue) const {
    if (!valid) return defaultValue;
    
    std::regex regex("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch match;
    if (std::regex_search(content, match, regex)) {
        return match[1].str();
    }
    return defaultValue;
}

bool JsonFile::getBool(const std::string& key, bool defaultValue) const {
    if (!valid) return defaultValue;
    
    std::regex regex("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch match;
    if (std::regex_search(content, match, regex)) {
        return (match[1].str() == "true");
    }
    return defaultValue;
}

Vector3d JsonFile::getVector3d(const std::string& key, const Vector3d& defaultValue) const {
    if (!valid) return defaultValue;
    
    // Pattern: "key": [x, y, z]
    std::regex vec_regex("\"" + key + "\"\\s*:\\s*\\[\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*,\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*,\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*\\]");
    std::smatch match;
    if (std::regex_search(content, match, vec_regex)) {
        try {
            double x = std::stod(match[1].str());
            double y = std::stod(match[2].str());
            double z = std::stod(match[3].str());
            return Vector3d(x, y, z);
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse Vector3d for key '" << key << "', using default" << std::endl;
        }
    }
    return defaultValue;
}

std::vector<Vector3d> JsonFile::getVector3dArray(const std::string& key) const {
    std::vector<Vector3d> result;
    if (!valid) return result;
    
    // Pattern: "key": [[x1, y1, z1], [x2, y2, z2], ...]
    // First find the array start
    std::regex array_start_regex("\"" + key + "\"\\s*:\\s*\\[");
    std::smatch start_match;
    if (!std::regex_search(content, start_match, array_start_regex)) {
        return result; // Key not found or not an array
    }
    
    // Find the position after the opening bracket
    size_t start_pos = start_match.position() + start_match.length();
    
    // Extract the array content (everything until matching closing bracket)
    int bracket_count = 1;
    size_t pos = start_pos;
    size_t array_start = start_pos;
    
    while (pos < content.length() && bracket_count > 0) {
        if (content[pos] == '[') bracket_count++;
        else if (content[pos] == ']') bracket_count--;
        pos++;
    }
    
    if (bracket_count != 0) {
        std::cerr << "Warning: Could not find matching closing bracket for array '" << key << "'" << std::endl;
        return result;
    }
    
    // Extract the array content
    std::string array_content = content.substr(array_start, pos - array_start - 1);
    
    // Parse individual vectors: [x, y, z]
    std::regex vec_regex("\\[\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*,\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*,\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*\\]");
    std::sregex_iterator iter(array_content.begin(), array_content.end(), vec_regex);
    std::sregex_iterator end;
    
    for (; iter != end; ++iter) {
        std::smatch match = *iter;
        try {
            double x = std::stod(match[1].str());
            double y = std::stod(match[2].str());
            double z = std::stod(match[3].str());
            result.push_back(Vector3d(x, y, z));
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse Vector3d in array '" << key << "', skipping" << std::endl;
        }
    }
    
    return result;
}

Matrix3d JsonFile::getDiagonalMatrix3d(const std::string& key, const Matrix3d& defaultValue) const {
    if (!valid) return defaultValue;
    
    // Pattern: "key": [x, y, z] - diagonal vector
    std::regex vector_regex("\"" + key + "\"\\s*:\\s*\\[\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*,\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*,\\s*(-?\\d+\\.?\\d*(?:[eE][+-]?\\d+)?)\\s*\\]");
    std::smatch match;
    if (std::regex_search(content, match, vector_regex)) {
        try {
            double x = std::stod(match[1].str());
            double y = std::stod(match[2].str());
            double z = std::stod(match[3].str());
            // Convert diagonal vector to diagonal matrix
            Matrix3d mat = Matrix3d::Zero();
            mat.diagonal() = Vector3d(x, y, z);
            return mat;
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not parse diagonal matrix for key '" << key << "', using default" << std::endl;
        }
    }
    return defaultValue;
}
