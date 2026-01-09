#pragma once

#include "av/foundation/math.hpp"
#include <vector>
#include <memory>

namespace av {

// Forward declarations
class Shader;

// Vertex structure
struct Vertex {
    Vec3 position;
    Vec3 normal;
    Vec2 texCoord;

    Vertex() = default;
    Vertex(const Vec3& pos, const Vec3& norm = Vec3::Zero(), const Vec2& tex = Vec2::Zero())
        : position(pos), normal(norm), texCoord(tex) {}
};

// Mesh class
class Mesh {
public:
    Mesh();
    ~Mesh();

    // Create from vertices and indices
    void setData(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);

    // Create simple shapes
    static std::shared_ptr<Mesh> createCube(float size = 1.0f);
    static std::shared_ptr<Mesh> createPlane(float width = 10.0f, float depth = 10.0f, int segments = 10);
    static std::shared_ptr<Mesh> createSphere(float radius = 1.0f, int segments = 32);
    static std::shared_ptr<Mesh> createCylinder(float radius = 1.0f, float height = 2.0f, int segments = 32);

    // Render the mesh
    void render() const;

    // Getters
    size_t getVertexCount() const { return vertices_.size(); }
    size_t getIndexCount() const { return indices_.size(); }
    bool isValid() const { return vao_ != 0; }

private:
    std::vector<Vertex> vertices_;
    std::vector<unsigned int> indices_;
    unsigned int vao_ = 0;  // Vertex Array Object
    unsigned int vbo_ = 0;  // Vertex Buffer Object
    unsigned int ebo_ = 0;  // Element Buffer Object

    void setupMesh();
};

} // namespace av
