#include "av/rendering/renderer.hpp"
#include "av/rendering/window.hpp"
#include "av/rendering/shader.hpp"
#include "av/rendering/mesh.hpp"
#include "av/rendering/camera.hpp"
#include "av/foundation/logging.hpp"
#include "av/foundation/math.hpp"
#include <GLFW/glfw3.h>  // GLFW includes modern OpenGL headers
#include <memory>

namespace av {

Renderer::Renderer() {
    AV_DEBUG("Renderer created");
}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::initialize(int width, int height, const std::string& title) {
    AV_INFO("Initializing Renderer with %dx%d window", width, height);

    // Create window and OpenGL context
    window_ = std::make_shared<Window>();
    if (!window_->initialize(width, height, title)) {
        AV_ERROR("Failed to initialize window");
        return false;
    }

    // Get OpenGL version info
    const char* glVersion = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    const char* glslVersion = reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION));
    if (glVersion) {
        AV_DEBUG("OpenGL version: %s", glVersion);
    }
    if (glslVersion) {
        AV_DEBUG("GLSL version: %s", glslVersion);
    }

    // Configure OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // Initialize shaders
    initializeShaders();
    if (!basicShader_ || !debugShader_) {
        AV_ERROR("Failed to initialize shaders");
        return false;
    }

    // Initialize debug meshes
    initializeDebugMeshes();

    // Create default camera
    if (!activeCamera_) {
        activeCamera_ = std::make_shared<FreeCamera>(
            Vec3(0, 5, 10),
            60.0f,
            getAspectRatio()
        );
    }

    AV_INFO("Renderer initialized successfully");
    return true;
}

bool Renderer::shouldContinue() const {
    return window_ && !window_->shouldClose();
}

void Renderer::beginFrame() {
    if (window_) {
        window_->update();
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::endFrame() {
    if (window_) {
        window_->swapBuffers();
    }
}

void Renderer::setCamera(std::shared_ptr<Camera> camera) {
    activeCamera_ = camera;
    if (activeCamera_) {
        activeCamera_->setAspectRatio(getAspectRatio());
    }
}

void Renderer::renderMesh(const std::shared_ptr<Mesh>& mesh, const Mat4& transform, const Vec3& color) {
    if (!mesh || !basicShader_) {
        return;
    }

    basicShader_->use();

    // Set matrices
    if (activeCamera_) {
        Mat4 view = activeCamera_->getViewMatrix();
        Mat4 projection = activeCamera_->getProjectionMatrix();

        basicShader_->setMat4("view", view);
        basicShader_->setMat4("projection", projection);
    }

    basicShader_->setMat4("model", transform);
    basicShader_->setVec3("color", color);

    mesh->render();
}

void Renderer::renderLine(const Vec3& from, const Vec3& to, const Vec3& color) {
    if (!debugShader_) {
        return;
    }

    // Create temporary line mesh
    auto tempLineMesh = std::make_shared<Mesh>();
    std::vector<Vertex> vertices = {
        Vertex(from, Vec3::Zero(), Vec2::Zero()),
        Vertex(to, Vec3::Zero(), Vec2::Zero())
    };
    std::vector<unsigned int> indices = { 0, 1 };
    tempLineMesh->setData(vertices, indices);

    debugShader_->use();

    if (activeCamera_) {
        Mat4 view = activeCamera_->getViewMatrix();
        Mat4 projection = activeCamera_->getProjectionMatrix();

        debugShader_->setMat4("view", view);
        debugShader_->setMat4("projection", projection);
    }

    debugShader_->setMat4("model", Mat4::Identity());
    debugShader_->setVec3("color", color);

    // Enable line mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    tempLineMesh->render();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::renderBox(const Vec3& center, const Vec3& size, const Vec3& color) {
    if (!cubeMesh_ || !debugShader_) {
        return;
    }

    // Create transformation matrix
    Mat4 transform = translate(Mat4::Identity(), center);
    transform = scale(transform, size * 0.5f);

    debugShader_->use();

    if (activeCamera_) {
        Mat4 view = activeCamera_->getViewMatrix();
        Mat4 projection = activeCamera_->getProjectionMatrix();

        debugShader_->setMat4("view", view);
        debugShader_->setMat4("projection", projection);
    }

    debugShader_->setMat4("model", transform);
    debugShader_->setVec3("color", color);

    cubeMesh_->render();
}

void Renderer::renderSphere(const Vec3& center, float radius, const Vec3& color) {
    if (!sphereMesh_ || !debugShader_) {
        return;
    }

    // Create transformation matrix
    Mat4 transform = translate(Mat4::Identity(), center);
    transform = scale(transform, Vec3::Ones() * radius);

    debugShader_->use();

    if (activeCamera_) {
        Mat4 view = activeCamera_->getViewMatrix();
        Mat4 projection = activeCamera_->getProjectionMatrix();

        debugShader_->setMat4("view", view);
        debugShader_->setMat4("projection", projection);
    }

    debugShader_->setMat4("model", transform);
    debugShader_->setVec3("color", color);

    sphereMesh_->render();
}

float Renderer::getAspectRatio() const {
    if (!window_) {
        return 1.777f;
    }
    return static_cast<float>(window_->getWidth()) / static_cast<float>(window_->getHeight());
}

void Renderer::shutdown() {
    cubeMesh_ = nullptr;
    sphereMesh_ = nullptr;
    lineMesh_ = nullptr;
    basicShader_ = nullptr;
    debugShader_ = nullptr;
    activeCamera_ = nullptr;

    if (window_) {
        window_->shutdown();
        window_ = nullptr;
    }

    AV_DEBUG("Renderer shut down");
}

void Renderer::initializeShaders() {
    // Create basic shader for mesh rendering
    basicShader_ = std::make_shared<Shader>();

    // Simple vertex shader source
    const char* basicVertexShader = R"(
        #version 410 core
        layout(location = 0) in vec3 position;
        layout(location = 1) in vec3 normal;
        layout(location = 2) in vec2 texCoord;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        out vec3 FragPos;
        out vec3 Normal;
        out vec2 TexCoord;

        void main() {
            FragPos = vec3(model * vec4(position, 1.0));
            Normal = mat3(transpose(inverse(model))) * normal;
            TexCoord = texCoord;

            gl_Position = projection * view * vec4(FragPos, 1.0);
        }
    )";

    const char* basicFragmentShader = R"(
        #version 410 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec2 TexCoord;

        uniform vec3 color;

        out vec4 FragColor;

        void main() {
            vec3 norm = normalize(Normal);
            vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));

            float diff = max(dot(norm, lightDir), 0.0);
            vec3 diffuse = (0.3 + 0.7 * diff) * color;

            FragColor = vec4(diffuse, 1.0);
        }
    )";

    // Compile basic shader
    if (!basicShader_->compile(basicVertexShader, basicFragmentShader)) {
        AV_ERROR("Failed to compile basic shader");
        basicShader_ = nullptr;
    } else {
        AV_DEBUG("Basic shader compiled successfully");
    }

    // Create debug shader
    debugShader_ = std::make_shared<Shader>();

    const char* debugVertexShader = R"(
        #version 410 core
        layout(location = 0) in vec3 position;
        layout(location = 1) in vec3 normal;
        layout(location = 2) in vec2 texCoord;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        void main() {
            gl_Position = projection * view * model * vec4(position, 1.0);
        }
    )";

    const char* debugFragmentShader = R"(
        #version 410 core
        uniform vec3 color;
        out vec4 FragColor;

        void main() {
            FragColor = vec4(color, 1.0);
        }
    )";

    // Compile debug shader
    if (!debugShader_->compile(debugVertexShader, debugFragmentShader)) {
        AV_ERROR("Failed to compile debug shader");
        debugShader_ = nullptr;
    } else {
        AV_DEBUG("Debug shader compiled successfully");
    }
}

void Renderer::initializeDebugMeshes() {
    cubeMesh_ = Mesh::createCube(1.0f);
    sphereMesh_ = Mesh::createSphere(1.0f, 32);

    // Create a simple line mesh (two vertices)
    lineMesh_ = std::make_shared<Mesh>();
    std::vector<Vertex> lineVertices = {
        Vertex(Vec3(0, 0, 0), Vec3::Zero(), Vec2::Zero()),
        Vertex(Vec3(1, 0, 0), Vec3::Zero(), Vec2::Zero())
    };
    std::vector<unsigned int> lineIndices = { 0, 1 };
    lineMesh_->setData(lineVertices, lineIndices);

    AV_DEBUG("Debug meshes initialized");
}

} // namespace av
