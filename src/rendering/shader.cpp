#include "av/rendering/shader.hpp"
#include "av/foundation/logging.hpp"
#include <GLFW/glfw3.h>  // GLFW includes modern OpenGL headers
#include <fstream>
#include <sstream>

namespace av {

Shader::Shader() {
    AV_DEBUG("Shader created");
}

Shader::~Shader() {
    if (programID_ != 0) {
        glDeleteProgram(programID_);
        AV_DEBUG("Shader destroyed");
    }
}

bool Shader::compile(const std::string& vertexSrc, const std::string& fragmentSrc) {
    // Compile vertex shader
    unsigned int vertexShader = compileShader(vertexSrc, GL_VERTEX_SHADER);
    if (vertexShader == 0) {
        AV_ERROR("Failed to compile vertex shader");
        return false;
    }

    // Compile fragment shader
    unsigned int fragmentShader = compileShader(fragmentSrc, GL_FRAGMENT_SHADER);
    if (fragmentShader == 0) {
        AV_ERROR("Failed to compile fragment shader");
        glDeleteShader(vertexShader);
        return false;
    }

    // Link program
    programID_ = glCreateProgram();
    glAttachShader(programID_, vertexShader);
    glAttachShader(programID_, fragmentShader);
    glLinkProgram(programID_);

    // Check for linking errors
    int success;
    char infoLog[512];
    glGetProgramiv(programID_, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(programID_, 512, nullptr, infoLog);
        AV_ERROR("Shader program linking failed: %s", infoLog);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(programID_);
        programID_ = 0;
        return false;
    }

    // Delete shaders as they're now linked
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    AV_DEBUG("Shader program compiled and linked successfully");
    return true;
}

bool Shader::loadFromFiles(const std::string& vertexPath, const std::string& fragmentPath) {
    // Read vertex shader
    std::ifstream vertexFile(vertexPath);
    if (!vertexFile.is_open()) {
        AV_ERROR("Failed to open vertex shader file: %s", vertexPath.c_str());
        return false;
    }
    std::stringstream vertexStream;
    vertexStream << vertexFile.rdbuf();
    std::string vertexSrc = vertexStream.str();
    vertexFile.close();

    // Read fragment shader
    std::ifstream fragmentFile(fragmentPath);
    if (!fragmentFile.is_open()) {
        AV_ERROR("Failed to open fragment shader file: %s", fragmentPath.c_str());
        return false;
    }
    std::stringstream fragmentStream;
    fragmentStream << fragmentFile.rdbuf();
    std::string fragmentSrc = fragmentStream.str();
    fragmentFile.close();

    AV_INFO("Loaded shaders from: %s and %s", vertexPath.c_str(), fragmentPath.c_str());
    return compile(vertexSrc, fragmentSrc);
}

void Shader::use() const {
    if (programID_ != 0) {
        glUseProgram(programID_);
    }
}

unsigned int Shader::compileShader(const std::string& source, unsigned int shaderType) {
    unsigned int shader = glCreateShader(shaderType);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    // Check for compilation errors
    int success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        const char* shaderType_str = (shaderType == GL_VERTEX_SHADER) ? "vertex" : "fragment";
        AV_ERROR("%s shader compilation failed: %s", shaderType_str, infoLog);
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

int Shader::getUniformLocation(const std::string& name) {
    // Check cache first
    auto it = uniformCache_.find(name);
    if (it != uniformCache_.end()) {
        return it->second;
    }

    // Get and cache location
    int location = glGetUniformLocation(programID_, name.c_str());
    uniformCache_[name] = location;

    if (location == -1) {
        AV_WARN("Uniform '%s' not found in shader", name.c_str());
    }

    return location;
}

void Shader::setInt(const std::string& name, int value) {
    glUniform1i(getUniformLocation(name), value);
}

void Shader::setFloat(const std::string& name, float value) {
    glUniform1f(getUniformLocation(name), value);
}

void Shader::setVec2(const std::string& name, const Vec2& value) {
    glUniform2f(getUniformLocation(name), value.x(), value.y());
}

void Shader::setVec3(const std::string& name, const Vec3& value) {
    glUniform3f(getUniformLocation(name), value.x(), value.y(), value.z());
}

void Shader::setVec4(const std::string& name, const Vec4& value) {
    glUniform4f(getUniformLocation(name), value.x(), value.y(), value.z(), value.w());
}

void Shader::setMat3(const std::string& name, const Mat3& value) {
    glUniformMatrix3fv(getUniformLocation(name), 1, GL_FALSE, value.data());
}

void Shader::setMat4(const std::string& name, const Mat4& value) {
    glUniformMatrix4fv(getUniformLocation(name), 1, GL_FALSE, value.data());
}

} // namespace av
