#pragma once

#include "av/foundation/math.hpp"
#include <string>
#include <unordered_map>

namespace av {

// Shader program wrapper
class Shader {
public:
    Shader();
    ~Shader();

    // Compile from source strings
    bool compile(const std::string& vertexSrc, const std::string& fragmentSrc);

    // Load from files
    bool loadFromFiles(const std::string& vertexPath, const std::string& fragmentPath);

    // Use this shader
    void use() const;

    // Set uniforms
    void setInt(const std::string& name, int value);
    void setFloat(const std::string& name, float value);
    void setVec2(const std::string& name, const Vec2& value);
    void setVec3(const std::string& name, const Vec3& value);
    void setVec4(const std::string& name, const Vec4& value);
    void setMat3(const std::string& name, const Mat3& value);
    void setMat4(const std::string& name, const Mat4& value);

    // Get program ID
    unsigned int getID() const { return programID_; }
    bool isValid() const { return programID_ != 0; }

private:
    unsigned int programID_ = 0;
    std::unordered_map<std::string, int> uniformCache_;

    // Compile individual shader
    unsigned int compileShader(const std::string& source, unsigned int shaderType);

    // Get uniform location (with caching)
    int getUniformLocation(const std::string& name);
};

} // namespace av
