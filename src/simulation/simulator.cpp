#include "av/simulation/simulator.hpp"
#include "av/physics/physics_world.hpp"
#include "av/physics/vehicle_dynamics.hpp"
#include "av/physics/vehicle_controller.hpp"
#include "av/world/world.hpp"
#include "av/foundation/logging.hpp"
#include <iostream>

namespace av {

// Simple implementation - will be expanded in Phase 2 and beyond
class SimulatorImpl {
public:
    std::unique_ptr<PhysicsWorld> physicsWorld;
    std::shared_ptr<World> world;
    std::shared_ptr<VehicleDynamics> egoVehicle;
    std::shared_ptr<VehicleController> vehicleController;
};

Simulator::Simulator()
    : impl_(std::make_unique<SimulatorImpl>()) {
    AV_DEBUG("Simulator created");
}

Simulator::~Simulator() {
    shutdown();
    AV_DEBUG("Simulator destroyed");
}

void Simulator::initialize() {
    AV_INFO("Initializing Simulator");

    try {
        // Initialize physics world
        impl_->physicsWorld = std::make_unique<PhysicsWorld>();
        impl_->physicsWorld->initialize();
        AV_INFO("Physics world initialized");

        // Initialize world
        impl_->world = std::make_shared<World>();
        impl_->world->initialize();
        AV_INFO("World initialized");

        // Create ego vehicle
        VehicleParams vehicleParams;
        vehicleParams.mass = 1500.0f;
        vehicleParams.wheelbase = 2.7f;
        vehicleParams.trackWidth = 1.6f;
        vehicleParams.maxSteeringAngle = 30.0f;
        vehicleParams.maxSpeed = 50.0f;
        vehicleParams.maxAcceleration = 5.0f;

        impl_->egoVehicle = std::make_shared<VehicleDynamics>(vehicleParams);
        impl_->egoVehicle->setPosition(Vec3(0.0f, 0.5f, 0.0f));
        AV_INFO("Ego vehicle created");

        // Create vehicle controller
        impl_->vehicleController = std::make_shared<VehicleController>(impl_->egoVehicle);
        AV_INFO("Vehicle controller created");

        running_ = true;
        AV_INFO("Simulator initialized successfully");

        // Print controls info
        std::cout << "\n===== Autonomous Vehicle Simulator Controls =====" << std::endl;
        std::cout << "W/A/S/D - Forward/Left/Backward/Right" << std::endl;
        std::cout << "Space   - Brake" << std::endl;
        std::cout << "Shift   - Boost/Nitro" << std::endl;
        std::cout << "ESC     - Exit" << std::endl;
        std::cout << "===============================================\n" << std::endl;

    } catch (const std::exception& e) {
        AV_ERROR("Failed to initialize simulator: {}", e.what());
        running_ = false;
        throw;
    }
}

void Simulator::step(float deltaTime) {
    if (!running_) {
        return;
    }

    try {
        // Clamp deltaTime
        deltaTime = clamp(deltaTime, 0.001f, 0.1f);

        // Update world
        if (impl_->world) {
            impl_->world->update(deltaTime);
        }

        // Update vehicle controller
        if (impl_->vehicleController) {
            impl_->vehicleController->update(deltaTime);
        }

        // Step physics
        if (impl_->physicsWorld) {
            impl_->physicsWorld->step(deltaTime);
        }

    } catch (const std::exception& e) {
        AV_ERROR("Error during simulation step: {}", e.what());
        running_ = false;
    }
}

void Simulator::render() {
    // TODO: Implement rendering in Phase 2
    // For now, this is a no-op
    // Will integrate with OpenGL/visualization system
}

void Simulator::shutdown() {
    AV_INFO("Shutting down simulator");

    running_ = false;

    // Clean up in reverse order
    impl_->vehicleController.reset();
    impl_->egoVehicle.reset();
    impl_->world.reset();

    if (impl_->physicsWorld) {
        impl_->physicsWorld->shutdown();
        impl_->physicsWorld.reset();
    }

    AV_INFO("Simulator shutdown complete");
}

bool Simulator::isRunning() const {
    return running_;
}

std::shared_ptr<VehicleDynamics> Simulator::getEgoVehicle() const {
    return impl_->egoVehicle;
}

std::shared_ptr<VehicleController> Simulator::getVehicleController() const {
    return impl_->vehicleController;
}

std::shared_ptr<World> Simulator::getWorld() const {
    return impl_->world;
}

PhysicsWorld* Simulator::getPhysicsWorld() const {
    return impl_->physicsWorld.get();
}

} // namespace av
