#include <gtest/gtest.h>
#include "av/rendering/mesh.hpp"
#include "av/rendering/shader.hpp"
#include "av/rendering/camera.hpp"
#include "av/rendering/debug_renderer.hpp"
#include "av/rendering/imgui_helper.hpp"
#include "av/foundation/logging.hpp"
#include <memory>

namespace av {

// Test Mesh creation and properties
class MeshTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::init();
    }

    void TearDown() override {
        Logger::shutdown();
    }
};

TEST_F(MeshTest, CreateCube) {
    auto cube = Mesh::createCube(2.0f);
    ASSERT_NE(cube, nullptr);
    EXPECT_GT(cube->getVertexCount(), 0);
    EXPECT_GT(cube->getIndexCount(), 0);
}

TEST_F(MeshTest, CreatePlane) {
    auto plane = Mesh::createPlane(10.0f, 10.0f, 5);
    ASSERT_NE(plane, nullptr);
    EXPECT_GT(plane->getVertexCount(), 0);
    EXPECT_GT(plane->getIndexCount(), 0);
}

TEST_F(MeshTest, CreateSphere) {
    auto sphere = Mesh::createSphere(1.0f, 32);
    ASSERT_NE(sphere, nullptr);
    EXPECT_GT(sphere->getVertexCount(), 0);
    EXPECT_GT(sphere->getIndexCount(), 0);
}

TEST_F(MeshTest, CreateCylinder) {
    auto cylinder = Mesh::createCylinder(1.0f, 2.0f, 16);
    ASSERT_NE(cylinder, nullptr);
    EXPECT_GT(cylinder->getVertexCount(), 0);
    EXPECT_GT(cylinder->getIndexCount(), 0);
}

// Test Camera functionality
class CameraTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::init();
    }

    void TearDown() override {
        Logger::shutdown();
    }
};

TEST_F(CameraTest, BaseCamera) {
    Camera camera(Vec3(0, 5, 10), Vec3(0, 0, 0));

    EXPECT_FLOAT_EQ(camera.getPosition().x(), 0.0f);
    EXPECT_FLOAT_EQ(camera.getPosition().y(), 5.0f);
    EXPECT_FLOAT_EQ(camera.getPosition().z(), 10.0f);

    camera.setPosition(Vec3(1, 2, 3));
    EXPECT_FLOAT_EQ(camera.getPosition().x(), 1.0f);
    EXPECT_FLOAT_EQ(camera.getPosition().y(), 2.0f);
    EXPECT_FLOAT_EQ(camera.getPosition().z(), 3.0f);
}

TEST_F(CameraTest, CameraMatrices) {
    Camera camera(Vec3(0, 0, 10), Vec3(0, 0, 0), 60.0f, 1.777f);

    Mat4 view = camera.getViewMatrix();
    Mat4 proj = camera.getProjectionMatrix();
    Mat4 viewProj = camera.getViewProjectionMatrix();

    // Check that matrices are not identity
    EXPECT_NE(view, Mat4::Identity());
    EXPECT_NE(proj, Mat4::Identity());
    EXPECT_NE(viewProj, Mat4::Identity());
}

TEST_F(CameraTest, OrbitCamera) {
    OrbitCamera camera(Vec3(0, 0, 0), 10.0f);

    Vec3 initialPos = camera.getPosition();
    EXPECT_FLOAT_EQ(initialPos.length(), 10.0f);  // At distance 10

    camera.rotate(3.14159f / 4.0f, 0.0f);  // 45 degrees
    camera.update(0.016f);

    Vec3 newPos = camera.getPosition();
    // Position should still be at distance 10 from center
    EXPECT_NEAR(newPos.length(), 10.0f, 0.1f);
}

TEST_F(CameraTest, FreeCamera) {
    FreeCamera camera(Vec3(0, 0, 0));

    camera.setMoveSpeed(5.0f);
    EXPECT_FLOAT_EQ(camera.getPosition().length(), 0.0f);

    camera.move(Vec3::UnitZ());
    EXPECT_GT(camera.getPosition().z(), 0.0f);
}

// Test DebugRenderer
class DebugRendererTest : public ::testing::Test {
protected:
    std::shared_ptr<Renderer> mockRenderer_;

    void SetUp() override {
        Logger::init();
        // Create a simple mock renderer
        mockRenderer_ = std::make_shared<Renderer>();
    }

    void TearDown() override {
        mockRenderer_ = nullptr;
        Logger::shutdown();
    }
};

TEST_F(DebugRendererTest, Creation) {
    auto debugRen = std::make_shared<DebugRenderer>(mockRenderer_);
    ASSERT_NE(debugRen, nullptr);
    EXPECT_TRUE(debugRen->isEnabled());
}

TEST_F(DebugRendererTest, AddPrimitives) {
    auto debugRen = std::make_shared<DebugRenderer>(mockRenderer_);

    debugRen->addLine(Vec3::Zero(), Vec3::UnitX(), Vec3(1, 0, 0));
    debugRen->addBox(Vec3(0, 5, 0), Vec3(2, 2, 2), Vec3(0, 1, 0));
    debugRen->addSphere(Vec3(0, 10, 0), 1.0f, Vec3(0, 0, 1));

    EXPECT_EQ(debugRen->getLineCount(), 1);
    EXPECT_EQ(debugRen->getBoxCount(), 1);
    EXPECT_EQ(debugRen->getSphereCount(), 1);

    debugRen->clear();
    EXPECT_EQ(debugRen->getLineCount(), 0);
}

TEST_F(DebugRendererTest, DrawGrid) {
    auto debugRen = std::make_shared<DebugRenderer>(mockRenderer_);

    debugRen->drawGrid(10.0f, 5);
    // Grid should have (5+1) * 2 lines: 6 lines in each direction
    EXPECT_EQ(debugRen->getLineCount(), 12);
}

TEST_F(DebugRendererTest, DrawAxes) {
    auto debugRen = std::make_shared<DebugRenderer>(mockRenderer_);

    debugRen->drawAxes(Vec3::Zero(), 1.0f);
    // Should have X, Y, Z axis lines
    EXPECT_EQ(debugRen->getLineCount(), 3);
}

// Test DebugUI
class DebugUITest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::init();
        DebugUI::init(1280, 720);
    }

    void TearDown() override {
        DebugUI::shutdown();
        Logger::shutdown();
    }
};

TEST_F(DebugUITest, Initialization) {
    DebugUI::text("Test message");  // Should not crash
}

TEST_F(DebugUITest, FrameRecording) {
    DebugUI::recordFrameTime(0.016f);
    DebugUI::recordFrameTime(0.016f);
    DebugUI::recordFrameTime(0.016f);

    float fps = DebugUI::getAverageFPS();
    EXPECT_NEAR(fps, 62.5f, 1.0f);  // ~60 FPS
}

// Integration test: Mesh + Camera rendering
TEST_F(CameraTest, MeshRenderingIntegration) {
    auto cube = Mesh::createCube(1.0f);
    Camera camera(Vec3(0, 5, 10), Vec3(0, 0, 0));

    // Verify that camera and mesh are compatible
    EXPECT_NE(cube, nullptr);
    EXPECT_GT(cube->getVertexCount(), 0);

    Mat4 viewProj = camera.getViewProjectionMatrix();
    EXPECT_NE(viewProj, Mat4::Identity());
}

}  // namespace av
