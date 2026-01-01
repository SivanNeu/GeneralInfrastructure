#ifndef JSON_FILE_H
#define JSON_FILE_H

#include "general.h"
#include <string>
#include <memory>
#include <vector>

/**
 * JsonFile - A simple JSON parser for configuration files
 * Supports comments (//) and provides type-safe access to JSON values
 */
class JsonFile {
private:
    std::string content;
    std::string filePath;
    bool valid;

    // Strip comments from JSON content
    static std::string stripComments(const std::string& content);

public:
    /**
     * Constructor - loads and parses JSON file
     * @param jsonFilePath Path to the JSON file
     */
    JsonFile(const std::string& jsonFilePath);

    /**
     * Check if the file was loaded successfully
     * @return true if file is valid, false otherwise
     */
    bool isValid() const { return valid; }

    /**
     * Get the file path
     * @return The path to the JSON file
     */
    std::string getFilePath() const { return filePath; }

    /**
     * Get a double value by key
     * @param key The parameter name
     * @param defaultValue Default value if key not found
     * @return The double value or defaultValue
     */
    double getDouble(const std::string& key, double defaultValue = 0.0) const;

    /**
     * Get a string value by key
     * @param key The parameter name
     * @param defaultValue Default value if key not found
     * @return The string value or defaultValue
     */
    std::string getString(const std::string& key, const std::string& defaultValue = "") const;

    /**
     * Get a boolean value by key
     * @param key The parameter name
     * @param defaultValue Default value if key not found
     * @return The boolean value or defaultValue
     */
    bool getBool(const std::string& key, bool defaultValue = false) const;

    /**
     * Get a Vector3d from JSON array [x, y, z]
     * @param key The parameter name
     * @param defaultValue Default value if key not found
     * @return The Vector3d value or defaultValue
     */
    Vector3d getVector3d(const std::string& key, const Vector3d& defaultValue = Vector3d::Zero()) const;

    /**
     * Get an array of Vector3d from JSON array of arrays [[x1, y1, z1], [x2, y2, z2], ...]
     * @param key The parameter name
     * @return Vector of Vector3d values, empty if key not found
     */
    std::vector<Vector3d> getVector3dArray(const std::string& key) const;

    /**
     * Get a diagonal Matrix3d from JSON array [x, y, z]
     * Converts the diagonal vector to a diagonal matrix
     * @param key The parameter name
     * @param defaultValue Default value if key not found
     * @return The diagonal Matrix3d or defaultValue
     */
    Matrix3d getDiagonalMatrix3d(const std::string& key, const Matrix3d& defaultValue = Matrix3d::Zero()) const;

    /**
     * Get raw content (for debugging)
     * @return The processed JSON content (comments stripped)
     */
    std::string getContent() const { return content; }
};

#endif // JSON_FILE_H
