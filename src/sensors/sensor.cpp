#include "av/sensors/sensor.hpp"
#include "av/foundation/logging.hpp"

namespace av {

void Sensor::setTransform(const Vec3& position, const Quat& rotation) {
    position_ = position;
    rotation_ = rotation;
    AV_DEBUG("Sensor {} transform updated: pos=({:.2f},{:.2f},{:.2f})",
             getName(), position.x(), position.y(), position.z());
}

} // namespace av
