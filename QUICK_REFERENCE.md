# Quick Reference Guide

## Project Status

**Completed Phases:**
- Phase 0 ✅ - Project Setup & Foundation Module
- Phase 1 ✅ - Physics Foundation & Vehicle Dynamics

**In Development:**
- Phase 2 - Rendering & Visualization (next)

## Available Components

### Foundation Module
Complete mathematical and utility library:

```cpp
// Math utilities
Vec3 v(1, 2, 3);
float distance = av::distance(v, Vec3::Zero());
Vec3 normalized = av::normalize(v);
float angle = av::toRadians(45.0f);

// Transforms
av::Transform t(Vec3(10, 0, 5), av::Quat::Identity());
Vec3 transformed = t.transformPoint(Vec3(1, 0, 0));

// Geometry
av::AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
bool inside = box.contains(Vec3(0, 0, 0));

// Timing
av::Clock clock;
clock.reset();
clock.start();
float dt = clock.tick();

// Configuration
av::Config::load("config.json");
float maxSpeed = av::Config::get<float>("vehicle.max_speed", 50.0f);

// Logging
AV_INFO("Message: {}", value);
AV_DEBUG("Debug message");
AV_ERROR("Error: {}", error_msg);
```

### Physics Module

#### Creating a Physics World
```cpp
auto physicsWorld = std::make_unique<av::PhysicsWorld>();
physicsWorld->initialize();

// Set gravity
physicsWorld->setGravity(Vec3(0, -9.81f, 0));

// Create a rigid body
auto body = physicsWorld->createRigidBody(1000.0f, Vec3(2, 1.5f, 4));

// Step simulation
physicsWorld->step(0.01f);
```

#### Vehicle Dynamics
```cpp
// Create vehicle with parameters
av::VehicleParams params;
params.mass = 1500.0f;
params.wheelbase = 2.7f;
params.maxSpeed = 50.0f;
params.maxAcceleration = 5.0f;

auto vehicle = std::make_shared<av::VehicleDynamics>(params);

// Set control inputs (-1 to 1 for steering, 0 to 1 for throttle/brake)
vehicle->setSteeringAngle(15.0f);  // degrees
vehicle->setThrottle(0.8f);        // 0 to 1
vehicle->setBrake(0.0f);           // 0 to 1

// Update vehicle
vehicle->update(deltaTime);

// Get state
Vec3 position = vehicle->getPosition();
Vec3 velocity = vehicle->getVelocity();
float speed = velocity.norm();
```

#### Vehicle Controller
```cpp
auto controller = std::make_shared<av::VehicleController>(vehicle);

// Set normalized inputs
controller->setSteeringInput(0.5f);  // -1 (left) to 1 (right)
controller->setThrottleInput(0.7f);  // 0 to 1
controller->setBrakeInput(0.0f);     // 0 to 1

// Auto-applies smoothing and updates vehicle
controller->update(deltaTime);
```

### World Module

#### World Management
```cpp
auto world = std::make_shared<av::World>();
world->initialize();

// Create traffic entities
auto trafficVehicle = world->createTrafficVehicle();
trafficVehicle->setPosition(Vec3(10, 0, 20));

auto pedestrian = world->createPedestrian();
pedestrian->setPosition(Vec3(15, 0, 5));

// Set environment
world->setTimeOfDay(12.5f);  // 12:30 PM
world->setWeather(0);        // Clear weather

// Update world
world->update(deltaTime);
```

### Simulation Module

#### Main Simulator
```cpp
av::Simulator simulator;

// Initialize
simulator.initialize();

// Main loop
while (simulator.isRunning()) {
    // Step physics with fixed timestep
    const float DT = 0.01f;  // 100 Hz
    simulator.step(DT);

    // Render would go here (Phase 2)
    simulator.render();
}

// Get subsystem references
auto vehicle = simulator.getEgoVehicle();
auto controller = simulator.getVehicleController();
auto world = simulator.getWorld();
auto physics = simulator.getPhysicsWorld();
```

## Configuration Files

### simulation_config.json
Located in `assets/configs/`

```json
{
  "simulation": {
    "timestep": 0.01,
    "target_fps": 60,
    "gravity": [0, -9.81, 0]
  },
  "vehicle": {
    "mass": 1500,
    "wheelbase": 2.7,
    "max_speed": 50.0,
    "max_acceleration": 5.0
  },
  "sensors": {
    "lidar": { "max_range": 120.0 },
    "camera": { "fov_degrees": 60 }
  }
}
```

## Typical Usage Pattern

```cpp
#include "av/foundation/logging.hpp"
#include "av/simulation/simulator.hpp"

int main() {
    // Initialize logging
    av::Logger::init("av.log");
    AV_INFO("Starting simulation");

    // Create and run simulator
    av::Simulator sim;
    sim.initialize();

    av::Clock clock;
    clock.reset();
    clock.start();

    while (sim.isRunning()) {
        float dt = clock.tick();

        // Update
        sim.step(dt);

        // Render
        sim.render();
    }

    av::Logger::shutdown();
    return 0;
}
```

## Testing

### Run All Tests
```bash
cd build
ctest --output-on-failure
```

### Run Specific Test Suite
```bash
./tests/test_foundation
./tests/test_physics
```

### Run with Verbose Output
```bash
ctest --verbose --output-on-failure
```

## Debug Features

### Logging Levels
- `AV_TRACE(msg)` - Detailed trace information
- `AV_DEBUG(msg)` - Debug-level messages
- `AV_INFO(msg)` - Informational messages
- `AV_WARN(msg)` - Warning messages
- `AV_ERROR(msg)` - Error messages
- `AV_CRITICAL(msg)` - Critical errors

### Performance Timing
```cpp
av::Timer timer;
// ... do work ...
float elapsed = timer.getElapsedSeconds();
AV_DEBUG("Operation took {:.3f}ms", elapsed * 1000);
```

## File Organization

```
av/
├── include/av/              # Public headers
│   ├── foundation/          # Math, geometry, utilities
│   ├── physics/             # Physics engine
│   ├── world/               # World and traffic
│   ├── sensors/             # (Phase 4+) Sensor simulation
│   ├── perception/          # (Phase 7+) Perception
│   ├── planning/            # (Phase 9+) Path planning
│   ├── control/             # (Phase 12+) Control
│   ├── rendering/           # (Phase 2) Visualization
│   └── simulation/          # Simulator orchestration
│
├── src/                     # Implementations
├── tests/                   # Unit tests
├── assets/                  # Resources
│   ├── configs/             # Configuration files
│   ├── maps/                # Road networks
│   ├── models/              # 3D models (Phase 2)
│   └── shaders/             # GLSL shaders (Phase 2)
│
├── scripts/                 # Build scripts
└── docs/                    # Documentation
```

## Common Patterns

### Creating a Custom Vehicle Behavior
```cpp
class MyController {
public:
    void update(av::VehicleController* ctrl, float dt) {
        // Simple proportional steering
        Vec3 targetPos(50, 0, 0);
        Vec3 vehiclePos = ctrl->getVehiclePos();
        float error = (targetPos - vehiclePos).norm();

        // Steer toward target
        float steering = (targetPos.x() - vehiclePos.x()) * 0.1f;
        ctrl->setSteeringInput(clamp(steering, -1, 1));
        ctrl->setThrottleInput(0.5f);
    }
};
```

### Querying World State
```cpp
auto world = simulator.getWorld();

// Get all traffic vehicles
auto roadNetwork = world->getRoadNetwork();
Vec3 closestLanePoint = roadNetwork->getClosestLanePoint(vehiclePos);

// Distance to nearest traffic
Vec3 egoPos = simulator.getEgoVehicle()->getPosition();
float minDist = INFINITY;
// Iterate and calculate distances
```

## Performance Tips

1. **Physics Timestep**: Use fixed 0.01s (100 Hz) for stability
2. **Vehicle Updates**: Can safely run at 100+ Hz
3. **World Updates**: Run at simulation rate (100 Hz minimum)
4. **Logging**: Use appropriate log levels to reduce overhead

## Next Phase (Phase 2) Preview

Phase 2 will add:
- OpenGL-based 3D visualization
- Real-time rendering of vehicle and environment
- Debug visualization tools
- Camera control (free, follow, chase)
- ImGui debug interface

## Support & Documentation

- **Implementation Plan**: `/home/jegoh/.claude/plans/fancy-finding-leaf.md`
- **Phase 1 Details**: `PHASE_1_PROGRESS.md`
- **Main README**: `README.md`
- **Logging**: `av.log` (generated at runtime)

## Quick Build Commands

```bash
# Clean build
rm -rf build && mkdir build && cd build && cmake .. && make -j$(nproc)

# Fast rebuild
cd build && make -j$(nproc)

# With tests
cd build && ctest --output-on-failure

# Release build
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc)
```

---

**For detailed implementation information, see PHASE_1_PROGRESS.md**
