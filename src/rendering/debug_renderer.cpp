#include "av/rendering/debug_renderer.hpp"
#include "av/foundation/logging.hpp"

namespace av {

DebugRenderer::DebugRenderer(std::shared_ptr<Renderer> renderer)
    : renderer_(renderer) {
    AV_DEBUG("DebugRenderer created");
}

DebugRenderer::~DebugRenderer() {
    AV_DEBUG("DebugRenderer destroyed");
}

void DebugRenderer::addLine(const Vec3& from, const Vec3& to, const Vec3& color) {
    lines_.push_back({from, to, color});
}

void DebugRenderer::addBox(const Vec3& center, const Vec3& size, const Vec3& color) {
    boxes_.push_back({center, size, color});
}

void DebugRenderer::addSphere(const Vec3& center, float radius, const Vec3& color) {
    spheres_.push_back({center, radius, color});
}

void DebugRenderer::addTrajectory(const std::vector<Vec3>& points, const Vec3& color) {
    trajectories_.push_back({points, color});
}

void DebugRenderer::drawAxes(const Vec3& position, float size) {
    // X-axis (red)
    addLine(position, position + Vec3::UnitX() * size, Vec3(1, 0, 0));
    // Y-axis (green)
    addLine(position, position + Vec3::UnitY() * size, Vec3(0, 1, 0));
    // Z-axis (blue)
    addLine(position, position + Vec3::UnitZ() * size, Vec3(0, 0, 1));
}

void DebugRenderer::drawGrid(float size, int gridCount, const Vec3& color) {
    float step = size / gridCount;
    float halfSize = size / 2.0f;

    // Draw grid lines along X direction
    for (int i = 0; i <= gridCount; ++i) {
        float offset = -halfSize + i * step;
        Vec3 start(offset, 0, -halfSize);
        Vec3 end(offset, 0, halfSize);
        addLine(start, end, color);
    }

    // Draw grid lines along Z direction
    for (int i = 0; i <= gridCount; ++i) {
        float offset = -halfSize + i * step;
        Vec3 start(-halfSize, 0, offset);
        Vec3 end(halfSize, 0, offset);
        addLine(start, end, color);
    }
}

void DebugRenderer::drawFrustum(const Mat4& projMat, const Vec3& color) {
    // Simplified frustum visualization using 8 corner points
    // In a full implementation, would extract frustum corners from projection matrix
    float near = 0.1f;
    float far = 100.0f;
    float width = 5.0f;
    float height = 5.0f;

    // Near plane corners
    Vec3 nTL(-width, height, near);
    Vec3 nTR(width, height, near);
    Vec3 nBL(-width, -height, near);
    Vec3 nBR(width, -height, near);

    // Far plane corners
    Vec3 fTL(-width * (far / near), height * (far / near), far);
    Vec3 fTR(width * (far / near), height * (far / near), far);
    Vec3 fBL(-width * (far / near), -height * (far / near), far);
    Vec3 fBR(width * (far / near), -height * (far / near), far);

    // Near plane
    addLine(nTL, nTR, color);
    addLine(nTR, nBR, color);
    addLine(nBR, nBL, color);
    addLine(nBL, nTL, color);

    // Far plane
    addLine(fTL, fTR, color);
    addLine(fTR, fBR, color);
    addLine(fBR, fBL, color);
    addLine(fBL, fTL, color);

    // Connecting lines
    addLine(nTL, fTL, color);
    addLine(nTR, fTR, color);
    addLine(nBL, fBL, color);
    addLine(nBR, fBR, color);
}

void DebugRenderer::render() {
    if (!enabled_ || !renderer_) {
        return;
    }

    // Render all lines
    for (const auto& line : lines_) {
        renderer_->renderLine(line.from, line.to, line.color);
    }

    // Render all boxes
    for (const auto& box : boxes_) {
        renderer_->renderBox(box.center, box.size, box.color);
    }

    // Render all spheres
    for (const auto& sphere : spheres_) {
        renderer_->renderSphere(sphere.center, sphere.radius, sphere.color);
    }

    // Render trajectories as line segments
    for (const auto& traj : trajectories_) {
        for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
            renderer_->renderLine(traj.points[i], traj.points[i + 1], traj.color);
        }
    }
}

void DebugRenderer::clear() {
    lines_.clear();
    boxes_.clear();
    spheres_.clear();
    trajectories_.clear();
}

} // namespace av
