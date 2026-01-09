#pragma once

#include "av/foundation/math.hpp"

namespace av {

class Sensor {
public:
    virtual ~Sensor() = default;
    virtual void update(float deltaTime) = 0;
    virtual std::string getName() const = 0;
};

} // namespace av
