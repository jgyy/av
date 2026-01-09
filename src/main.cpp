#include "av/foundation/logging.hpp"
#include "av/foundation/clock.hpp"
#include "av/foundation/config.hpp"
#include "av/foundation/math.hpp"
#include "av/simulation/simulator.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    try {
        // Initialize logging
        av::Logger::init("av.log");
        AV_INFO("===== Autonomous Vehicle Simulation =====");
        AV_INFO("Starting application...");

        // Load configuration
        try {
            av::Config::load("assets/configs/simulation_config.json");
            AV_INFO("Configuration loaded successfully");
        } catch (const std::exception& e) {
            AV_WARN("Failed to load configuration file: {}", e.what());
            AV_INFO("Using default configuration");
        }

        // Create and initialize simulator
        av::Simulator simulator;
        AV_INFO("Initializing simulator...");
        simulator.initialize();

        // Load scenario
        AV_INFO("Loading scenario...");
        av::Scenario scenario;
        try {
            scenario.load("assets/maps/simple_city.json");
            AV_INFO("Scenario loaded successfully");
        } catch (const std::exception& e) {
            AV_WARN("Failed to load scenario: {}", e.what());
        }

        // Main simulation loop
        av::Clock clock;
        clock.reset();
        clock.start();

        const float TARGET_FPS = 60.0f;
        const float FRAME_TIME = 1.0f / TARGET_FPS;
        float accumulator = 0.0f;

        AV_INFO("Starting main simulation loop (Press Ctrl+C to exit)");

        while (simulator.isRunning()) {
            float deltaTime = clock.tick();

            // Cap delta time to prevent spiral of death
            if (deltaTime > 0.1f) {
                deltaTime = 0.1f;
            }

            // Fixed timestep simulation
            accumulator += deltaTime;

            // Step physics at fixed rate
            const float PHYSICS_TIMESTEP = 0.01f; // 100 Hz
            while (accumulator >= PHYSICS_TIMESTEP) {
                simulator.step(PHYSICS_TIMESTEP);
                accumulator -= PHYSICS_TIMESTEP;
            }

            // Render
            simulator.render();

            // Log performance every 5 seconds
            static float logTimer = 0.0f;
            logTimer += deltaTime;
            if (logTimer >= 5.0f) {
                AV_DEBUG("Simulation running... Elapsed: {:.1f}s", clock.getElapsedTime());
                logTimer = 0.0f;
            }
        }

        AV_INFO("Simulator shutdown requested");

        // Cleanup
        av::Logger::shutdown();
        std::cout << "Thank you for using AV Simulator!" << std::endl;

        return 0;

    } catch (const std::exception& e) {
        AV_CRITICAL("Fatal error: {}", e.what());
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}
