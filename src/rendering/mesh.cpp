#include "av/rendering/mesh.hpp"
#include "av/foundation/logging.hpp"
#include <GLFW/glfw3.h>  // GLFW includes modern OpenGL headers
#include <cmath>

namespace av {

Mesh::Mesh() {
    AV_DEBUG("Mesh created");
}

Mesh::~Mesh() {
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
    }
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
    }
    if (ebo_ != 0) {
        glDeleteBuffers(1, &ebo_);
    }
    AV_DEBUG("Mesh destroyed");
}

void Mesh::setData(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) {
    vertices_ = vertices;
    indices_ = indices;
    setupMesh();
    AV_DEBUG("Mesh data set: %zu vertices, %zu indices", vertices_.size(), indices_.size());
}

void Mesh::setupMesh() {
    // Create vertex array object
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    // Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), vertices_.data(), GL_STATIC_DRAW);

    // Element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), indices_.data(), GL_STATIC_DRAW);

    // Vertex attributes
    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));

    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    // Texture coordinate
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoord));

    glBindVertexArray(0);
}

void Mesh::render() const {
    if (vao_ == 0) {
        AV_WARN("Attempting to render invalid mesh");
        return;
    }

    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

std::shared_ptr<Mesh> Mesh::createCube(float size) {
    auto mesh = std::make_shared<Mesh>();

    float s = size / 2.0f;
    std::vector<Vertex> vertices = {
        // Front face
        Vertex(Vec3(-s, -s, s), Vec3(0, 0, 1), Vec2(0, 0)),
        Vertex(Vec3(s, -s, s), Vec3(0, 0, 1), Vec2(1, 0)),
        Vertex(Vec3(s, s, s), Vec3(0, 0, 1), Vec2(1, 1)),
        Vertex(Vec3(-s, s, s), Vec3(0, 0, 1), Vec2(0, 1)),
        // Back face
        Vertex(Vec3(-s, -s, -s), Vec3(0, 0, -1), Vec2(1, 0)),
        Vertex(Vec3(-s, s, -s), Vec3(0, 0, -1), Vec2(1, 1)),
        Vertex(Vec3(s, s, -s), Vec3(0, 0, -1), Vec2(0, 1)),
        Vertex(Vec3(s, -s, -s), Vec3(0, 0, -1), Vec2(0, 0)),
        // Top face
        Vertex(Vec3(-s, s, -s), Vec3(0, 1, 0), Vec2(0, 1)),
        Vertex(Vec3(-s, s, s), Vec3(0, 1, 0), Vec2(0, 0)),
        Vertex(Vec3(s, s, s), Vec3(0, 1, 0), Vec2(1, 0)),
        Vertex(Vec3(s, s, -s), Vec3(0, 1, 0), Vec2(1, 1)),
        // Bottom face
        Vertex(Vec3(-s, -s, -s), Vec3(0, -1, 0), Vec2(1, 1)),
        Vertex(Vec3(s, -s, -s), Vec3(0, -1, 0), Vec2(0, 1)),
        Vertex(Vec3(s, -s, s), Vec3(0, -1, 0), Vec2(0, 0)),
        Vertex(Vec3(-s, -s, s), Vec3(0, -1, 0), Vec2(1, 0)),
        // Right face
        Vertex(Vec3(s, -s, -s), Vec3(1, 0, 0), Vec2(1, 0)),
        Vertex(Vec3(s, s, -s), Vec3(1, 0, 0), Vec2(1, 1)),
        Vertex(Vec3(s, s, s), Vec3(1, 0, 0), Vec2(0, 1)),
        Vertex(Vec3(s, -s, s), Vec3(1, 0, 0), Vec2(0, 0)),
        // Left face
        Vertex(Vec3(-s, -s, -s), Vec3(-1, 0, 0), Vec2(0, 0)),
        Vertex(Vec3(-s, -s, s), Vec3(-1, 0, 0), Vec2(1, 0)),
        Vertex(Vec3(-s, s, s), Vec3(-1, 0, 0), Vec2(1, 1)),
        Vertex(Vec3(-s, s, -s), Vec3(-1, 0, 0), Vec2(0, 1)),
    };

    std::vector<unsigned int> indices = {
        0, 1, 2, 2, 3, 0,       // Front
        4, 5, 6, 6, 7, 4,       // Back
        8, 9, 10, 10, 11, 8,    // Top
        12, 13, 14, 14, 15, 12, // Bottom
        16, 17, 18, 18, 19, 16, // Right
        20, 21, 22, 22, 23, 20  // Left
    };

    mesh->setData(vertices, indices);
    AV_INFO("Created cube mesh");
    return mesh;
}

std::shared_ptr<Mesh> Mesh::createPlane(float width, float depth, int segments) {
    auto mesh = std::make_shared<Mesh>();

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    float halfWidth = width / 2.0f;
    float halfDepth = depth / 2.0f;
    float segmentWidth = width / segments;
    float segmentDepth = depth / segments;

    // Create vertices
    for (int z = 0; z <= segments; z++) {
        for (int x = 0; x <= segments; x++) {
            float posX = -halfWidth + x * segmentWidth;
            float posZ = -halfDepth + z * segmentDepth;
            float texX = static_cast<float>(x) / segments;
            float texZ = static_cast<float>(z) / segments;

            vertices.emplace_back(
                Vec3(posX, 0.0f, posZ),
                Vec3(0, 1, 0),
                Vec2(texX, texZ)
            );
        }
    }

    // Create indices
    for (int z = 0; z < segments; z++) {
        for (int x = 0; x < segments; x++) {
            unsigned int a = z * (segments + 1) + x;
            unsigned int b = a + 1;
            unsigned int c = a + segments + 1;
            unsigned int d = c + 1;

            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
    }

    mesh->setData(vertices, indices);
    AV_INFO("Created plane mesh (%.1fx%.1f, %d segments)", width, depth, segments);
    return mesh;
}

std::shared_ptr<Mesh> Mesh::createSphere(float radius, int segments) {
    auto mesh = std::make_shared<Mesh>();

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    // Create vertices
    for (int i = 0; i <= segments; i++) {
        float phi = PI * i / segments;
        float sinPhi = std::sin(phi);
        float cosPhi = std::cos(phi);

        for (int j = 0; j <= segments; j++) {
            float theta = TWO_PI * j / segments;
            float sinTheta = std::sin(theta);
            float cosTheta = std::cos(theta);

            Vec3 pos(
                radius * sinPhi * cosTheta,
                radius * cosPhi,
                radius * sinPhi * sinTheta
            );

            Vec3 normal = normalize(pos);

            vertices.emplace_back(pos, normal, Vec2(static_cast<float>(j) / segments, static_cast<float>(i) / segments));
        }
    }

    // Create indices
    for (int i = 0; i < segments; i++) {
        for (int j = 0; j < segments; j++) {
            unsigned int a = i * (segments + 1) + j;
            unsigned int b = a + 1;
            unsigned int c = a + segments + 1;
            unsigned int d = c + 1;

            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
    }

    mesh->setData(vertices, indices);
    AV_INFO("Created sphere mesh (radius: %.2f, segments: %d)", radius, segments);
    return mesh;
}

std::shared_ptr<Mesh> Mesh::createCylinder(float radius, float height, int segments) {
    auto mesh = std::make_shared<Mesh>();

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    float halfHeight = height / 2.0f;

    // Create side vertices
    for (int i = 0; i <= segments; i++) {
        float theta = TWO_PI * i / segments;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        // Top vertex
        vertices.emplace_back(
            Vec3(radius * cosTheta, halfHeight, radius * sinTheta),
            Vec3(cosTheta, 0, sinTheta),
            Vec2(static_cast<float>(i) / segments, 1.0f)
        );

        // Bottom vertex
        vertices.emplace_back(
            Vec3(radius * cosTheta, -halfHeight, radius * sinTheta),
            Vec3(cosTheta, 0, sinTheta),
            Vec2(static_cast<float>(i) / segments, 0.0f)
        );
    }

    // Add top center
    vertices.emplace_back(Vec3(0, halfHeight, 0), Vec3(0, 1, 0), Vec2(0.5f, 0.5f));
    unsigned int topCenter = vertices.size() - 1;

    // Add bottom center
    vertices.emplace_back(Vec3(0, -halfHeight, 0), Vec3(0, -1, 0), Vec2(0.5f, 0.5f));
    unsigned int bottomCenter = vertices.size() - 1;

    // Side indices
    for (int i = 0; i < segments; i++) {
        unsigned int top1 = i * 2;
        unsigned int bottom1 = i * 2 + 1;
        unsigned int top2 = ((i + 1) % segments) * 2;
        unsigned int bottom2 = ((i + 1) % segments) * 2 + 1;

        indices.push_back(top1);
        indices.push_back(top2);
        indices.push_back(bottom1);
        indices.push_back(bottom1);
        indices.push_back(top2);
        indices.push_back(bottom2);
    }

    // Top cap indices
    for (int i = 0; i < segments; i++) {
        indices.push_back(i * 2);
        indices.push_back(topCenter);
        indices.push_back(((i + 1) % segments) * 2);
    }

    // Bottom cap indices
    for (int i = 0; i < segments; i++) {
        indices.push_back(bottomCenter);
        indices.push_back(i * 2 + 1);
        indices.push_back(((i + 1) % segments) * 2 + 1);
    }

    mesh->setData(vertices, indices);
    AV_INFO("Created cylinder mesh (radius: %.2f, height: %.2f, segments: %d)", radius, height, segments);
    return mesh;
}

} // namespace av
