#pragma once

#include "av/foundation/math.hpp"
#include <string>
#include <memory>
#include <vector>

namespace av {

// Forward declarations
class World;

// Base class for all sensors
class Sensor {
public:
    enum class Status {
        UNINITIALIZED,
        ACTIVE,
        ERROR,
        CALIBRATING
    };

    virtual ~Sensor() = default;

    // Core sensor interface
    virtual void update(float deltaTime) = 0;
    virtual std::string getName() const = 0;

    // Sensor state management
    virtual bool initialize() { return true; }
    virtual void shutdown() {}

    // Status queries
    Status getStatus() const { return status_; }
    void setStatus(Status status) { status_ = status; }

    // Configuration
    void setTransform(const Vec3& position, const Quat& rotation);
    const Vec3& getPosition() const { return position_; }
    const Quat& getRotation() const { return rotation_; }

    // World reference (optional)
    void setWorld(std::shared_ptr<World> world) { world_ = world; }
    std::shared_ptr<World> getWorld() const { return world_; }

    // Timestamp
    float getLastUpdateTime() const { return lastUpdateTime_; }
    int getFrameCount() const { return frameCount_; }

protected:
    Status status_ = Status::UNINITIALIZED;
    Vec3 position_ = Vec3::Zero();
    Quat rotation_ = Quat::Identity();
    std::shared_ptr<World> world_;

    float lastUpdateTime_ = 0.0f;
    int frameCount_ = 0;
};

} // namespace av
