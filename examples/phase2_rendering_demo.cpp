/*
 * Phase 2: Rendering System Demo
 *
 * This example demonstrates the complete Phase 2 rendering and visualization system:
 * - Window creation and OpenGL context management
 * - 3D mesh rendering with shaders
 * - Multiple camera types (free, orbit, follow)
 * - Debug visualization (grids, axes, trajectories)
 * - Scene management with multiple objects
 * - Debug UI for real-time statistics
 */

#include "av/foundation/logging.hpp"
#include "av/foundation/math.hpp"
#include "av/rendering/renderer.hpp"
#include "av/rendering/scene.hpp"
#include "av/rendering/camera.hpp"
#include "av/rendering/debug_renderer.hpp"
#include "av/rendering/imgui_helper.hpp"
#include <memory>
#include <chrono>

namespace av {

class Phase2Demo {
public:
    Phase2Demo() = default;
    ~Phase2Demo() = default;

    bool initialize() {
        AV_INFO("=== Phase 2 Rendering System Demo ===");

        // Initialize logger
        Logger::init("phase2_demo.log");
        Logger::setLevel(Logger::DEBUG);

        // Create renderer
        renderer_ = std::make_shared<Renderer>();
        if (!renderer_->initialize(1280, 720, "AV Simulator - Phase 2 Rendering")) {
            AV_ERROR("Failed to initialize renderer");
            return false;
        }

        // Create scene
        scene_ = std::make_shared<Scene>(renderer_);
        scene_->initialize();

        // Add some test objects
        scene_->addVehicle("vehicle_1", Vec3(0, 1, 0), Vec3(0.2f, 0.2f, 0.8f));
        scene_->addVehicle("vehicle_2", Vec3(5, 1, 5), Vec3(0.8f, 0.2f, 0.2f));

        // Setup camera
        auto freeCamera = std::make_shared<FreeCamera>(
            Vec3(0, 10, 20),
            60.0f,
            renderer_->getAspectRatio()
        );
        renderer_->setCamera(freeCamera);

        // Initialize debug UI
        DebugUI::init(1280, 720);

        AV_INFO("Phase 2 Demo initialized successfully");
        return true;
    }

    void run() {
        auto lastTime = std::chrono::high_resolution_clock::now();
        int frameCount = 0;

        while (renderer_->shouldContinue()) {
            // Calculate delta time
            auto currentTime = std::chrono::high_resolution_clock::now();
            float deltaTime = std::chrono::duration<float>(currentTime - lastTime).count();
            lastTime = currentTime;

            // Update debug UI
            DebugUI::beginFrame();
            DebugUI::recordFrameTime(deltaTime);

            // Render frame
            renderer_->beginFrame();

            // Render scene
            scene_->render();
            scene_->renderDebug();

            // Display statistics
            DebugUI::begin("Statistics");
            DebugUI::text("FPS: %.1f", DebugUI::getAverageFPS());
            DebugUI::text("Delta Time: %.3f ms", deltaTime * 1000.0f);
            DebugUI::text("Vehicles: %zu", scene_->getObjectCount() - 1);  // -1 for ground
            DebugUI::separator();
            DebugUI::text("Scene Objects:");
            for (size_t i = 0; i < scene_->getObjectCount(); ++i) {
                DebugUI::text("  Object %zu", i);
            }
            DebugUI::end();

            // Debug controls
            DebugUI::begin("Debug Options");
            static bool showGrid = true;
            static bool showAxes = true;
            DebugUI::checkbox("Show Grid", showGrid);
            DebugUI::checkbox("Show Axes", showAxes);
            scene_->enableDebugGrid(showGrid);
            scene_->enableDebugAxes(showAxes);
            DebugUI::end();

            // Demo: Update vehicle positions
            static float time = 0.0f;
            time += deltaTime;

            // Rotate vehicle 1 around origin
            Transform vehicle1Transform;
            vehicle1Transform.setPosition(Vec3(
                std::cos(time) * 5.0f,
                1.0f,
                std::sin(time) * 5.0f
            ));
            scene_->updateObjectTransform("vehicle_1", vehicle1Transform);

            // Move vehicle 2 back and forth
            Transform vehicle2Transform;
            vehicle2Transform.setPosition(Vec3(
                5.0f + std::sin(time) * 3.0f,
                1.0f,
                5.0f
            ));
            scene_->updateObjectTransform("vehicle_2", vehicle2Transform);

            DebugUI::endFrame();
            renderer_->endFrame();

            frameCount++;

            // Print stats every 60 frames
            if (frameCount % 60 == 0) {
                AV_DEBUG("Frame %d, FPS: %.1f", frameCount, DebugUI::getAverageFPS());
            }
        }

        AV_INFO("Demo ended after %d frames", frameCount);
    }

    void shutdown() {
        AV_INFO("Shutting down Phase 2 Demo");

        DebugUI::shutdown();
        scene_ = nullptr;
        renderer_->shutdown();
        renderer_ = nullptr;

        Logger::shutdown();
    }

private:
    std::shared_ptr<Renderer> renderer_;
    std::shared_ptr<Scene> scene_;
};

}  // namespace av

/*
 * Main entry point (would be in main.cpp)
 *
 * int main() {
 *     av::Phase2Demo demo;
 *     if (demo.initialize()) {
 *         demo.run();
 *         demo.shutdown();
 *         return 0;
 *     }
 *     return 1;
 * }
 */
