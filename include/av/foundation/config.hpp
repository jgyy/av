#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <fstream>
#include <stdexcept>

namespace av {

using json = nlohmann::json;

// Configuration manager
class Config {
public:
    // Load configuration from JSON file
    static void load(const std::string& filePath) {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open config file: " + filePath);
        }

        try {
            file >> data_;
        } catch (const json::parse_error& e) {
            throw std::runtime_error("JSON parse error in config file: " + std::string(e.what()));
        }
    }

    // Load from string
    static void loadFromString(const std::string& jsonString) {
        try {
            data_ = json::parse(jsonString);
        } catch (const json::parse_error& e) {
            throw std::runtime_error("JSON parse error: " + std::string(e.what()));
        }
    }

    // Save configuration to JSON file
    static void save(const std::string& filePath) {
        std::ofstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open config file for writing: " + filePath);
        }
        file << data_.dump(2);
    }

    // Get value with default
    template <typename T>
    static T get(const std::string& key, const T& defaultValue) {
        try {
            if (data_.contains(key)) {
                return data_[key].get<T>();
            }
        } catch (const json::exception& e) {
            // Type conversion failed, return default
        }
        return defaultValue;
    }

    // Get value (throws if not found)
    template <typename T>
    static T get(const std::string& key) {
        if (!data_.contains(key)) {
            throw std::runtime_error("Config key not found: " + key);
        }
        return data_[key].get<T>();
    }

    // Set value
    template <typename T>
    static void set(const std::string& key, const T& value) {
        data_[key] = value;
    }

    // Check if key exists
    static bool has(const std::string& key) {
        return data_.contains(key);
    }

    // Get nested value with dot notation (e.g., "simulation.timestep")
    template <typename T>
    static T getNested(const std::string& keyPath, const T& defaultValue) {
        json* ptr = &data_;
        std::string path = keyPath;

        size_t pos = 0;
        while ((pos = path.find('.')) != std::string::npos) {
            std::string part = path.substr(0, pos);
            if (!ptr->contains(part)) {
                return defaultValue;
            }
            ptr = &(*ptr)[part];
            path.erase(0, pos + 1);
        }

        if (ptr->contains(path)) {
            try {
                return (*ptr)[path].get<T>();
            } catch (const json::exception&) {
                return defaultValue;
            }
        }

        return defaultValue;
    }

    // Get raw JSON data
    static const json& getData() {
        return data_;
    }

    // Clear all config
    static void clear() {
        data_.clear();
    }

    // Merge another config into this one
    static void merge(const json& other) {
        data_.merge_patch(other);
    }

private:
    static json data_;
};

// Inline implementation of static member
inline json Config::data_;

} // namespace av
