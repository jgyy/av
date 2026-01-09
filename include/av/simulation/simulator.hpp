#pragma once

#include "av/foundation/math.hpp"
#include <memory>

namespace av {

// Forward declarations
class PhysicsWorld;
class World;
class VehicleDynamics;
class VehicleController;
class SimulatorImpl;

// Main simulator orchestrator
class Simulator {
public:
    Simulator();
    ~Simulator();

    // Initialize simulator (sets up all systems)
    void initialize();

    // Step simulation by deltaTime
    void step(float deltaTime);

    // Render (visualization, will be implemented in Phase 2)
    void render();

    // Shutdown simulator
    void shutdown();

    // Check if simulator is running
    bool isRunning() const;

    // Get references to subsystems
    std::shared_ptr<VehicleDynamics> getEgoVehicle() const;
    std::shared_ptr<VehicleController> getVehicleController() const;
    std::shared_ptr<World> getWorld() const;
    PhysicsWorld* getPhysicsWorld() const;

private:
    std::unique_ptr<SimulatorImpl> impl_;
    bool running_ = false;
};

} // namespace av
