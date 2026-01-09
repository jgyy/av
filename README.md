# Autonomous Vehicle Simulation

A realistic, full-stack autonomous vehicle simulation system built from scratch in C++. This project aims to provide a comprehensive platform for testing and developing autonomous driving algorithms with high-fidelity sensor simulation, physics-based vehicle dynamics, and a complete perception-planning-control pipeline.

## Project Overview

This is a comprehensive autonomous vehicle simulator featuring:

- **Complete Sensor Suite**: LIDAR (ray-traced point clouds), Cameras (realistic optics), Radar, Ultrasonic, GPS, and IMU
- **Physics Simulation**: Bullet Physics integration with realistic vehicle dynamics
- **Full Traffic Simulation**: Multi-vehicle traffic, pedestrians, traffic lights, and complex road networks
- **Autonomous Driving Stack**: Perception → Planning → Control pipeline
- **Real-time 3D Visualization**: OpenGL rendering with debug visualization

## System Architecture

```
┌─────────────────────────────────────────┐
│    Application (Simulator, UI)          │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  AV Stack: Perception → Plan > Control  │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  Sensors: LIDAR│Camera│Radar│GPS│IMU    │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  Simulation: Physics│World│Traffic      │
└─────────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────────┐
│  Foundation: Math│Geometry│Threading    │
└─────────────────────────────────────────┘
```

## Building

### Prerequisites

- CMake 3.20+
- C++17 compatible compiler
- Conan or vcpkg for dependencies

### Quick Start

```bash
chmod +x scripts/build.sh
./scripts/build.sh Release

cd build
./av_exe
```

## Project Structure

```
av/
├── include/av/              # Public headers
│   ├── foundation/          # Math, geometry, time, logging, config
│   ├── physics/             # Physics engine wrapper
│   ├── world/               # Environment and traffic
│   ├── sensors/             # Sensor simulation
│   ├── perception/          # Perception pipeline
│   ├── planning/            # Path planning and behavior
│   ├── control/             # Vehicle control
│   ├── rendering/           # 3D visualization
│   └── simulation/          # Main simulator
├── src/                     # Implementation
├── tests/                   # Unit tests
├── benchmarks/              # Performance tests
├── assets/                  # Models, shaders, configs
└── scripts/                 # Build and utility scripts
```

## Status

**Phase 0: Project Setup** - COMPLETE
- ✅ CMake build system configured
- ✅ Foundation module implemented (math, geometry, clock, logging, config)
- ✅ All module headers and stubs created
- ✅ Unit tests framework set up
- ✅ Documentation and configuration templates

**Phase 1-15: Implementation in progress**

See detailed implementation plan at: `/home/jegoh/.claude/plans/fancy-finding-leaf.md`

## License

Apache License 2.0
