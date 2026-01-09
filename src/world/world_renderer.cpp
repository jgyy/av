#include "av/world/world_renderer.hpp"
#include "av/world/road_network.hpp"
#include "av/world/traffic.hpp"
#include "av/rendering/scene.hpp"
#include "av/rendering/debug_renderer.hpp"
#include "av/foundation/logging.hpp"
#include <cmath>

namespace av {

WorldRenderer::WorldRenderer(std::shared_ptr<Scene> scene)
    : scene_(scene) {
    AV_DEBUG("WorldRenderer created");
}

WorldRenderer::~WorldRenderer() {
    AV_DEBUG("WorldRenderer destroyed");
}

void WorldRenderer::renderWorld(std::shared_ptr<World> world) {
    if (!world || !scene_) return;

    // Render road network
    if (showLanes_ || showIntersections_) {
        renderRoadNetwork(world->getRoadNetwork());
    }

    // Render traffic entities
    if (showTrafficVehicles_) {
        renderTrafficVehicles(world->getTrafficVehicles());
    }

    if (showPedestrians_) {
        renderPedestrians(world->getPedestrians());
    }
}

void WorldRenderer::renderRoadNetwork(std::shared_ptr<RoadNetwork> roadNetwork) {
    if (!roadNetwork || !scene_) return;

    // Render lanes
    if (showLanes_) {
        for (const auto& lane : roadNetwork->getLanes()) {
            if (lane) {
                renderLane(lane);
                renderLaneMarkers(lane);
            }
        }
    }

    // Render intersections
    if (showIntersections_) {
        renderIntersections(roadNetwork->getIntersections());
    }
}

void WorldRenderer::renderLane(std::shared_ptr<Lane> lane) {
    if (!lane) return;

    const auto& centerline = lane->getCenterline();
    if (centerline.size() < 2) return;

    // Draw lane centerline as a series of line segments
    for (size_t i = 0; i + 1 < centerline.size(); ++i) {
        const Vec3& p1 = centerline[i];
        const Vec3& p2 = centerline[i + 1];

        // Calculate width based on lane boundaries
        float width = lane->getWidth() / 2.0f;

        // Draw left boundary
        Vec3 normal = (p2 - p1).normalized();
        Vec3 left_p1 = p1 + Vec3(0, 0, 1).cross(normal) * width;
        Vec3 left_p2 = p2 + Vec3(0, 0, 1).cross(normal) * width;

        // Draw right boundary
        Vec3 right_p1 = p1 - Vec3(0, 0, 1).cross(normal) * width;
        Vec3 right_p2 = p2 - Vec3(0, 0, 1).cross(normal) * width;

        // Render lane geometry (simplified - just use centerline for now)
        // In a real implementation, would create proper lane mesh
        AV_DEBUG("Rendering lane %d segment: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)",
                 lane->getId(), p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z());
    }
}

void WorldRenderer::renderLaneMarkers(std::shared_ptr<Lane> lane) {
    if (!lane) return;

    const auto& centerline = lane->getCenterline();
    if (centerline.size() < 2) return;

    // Draw lane markers at regular intervals
    float markerSpacing = 5.0f;  // meters
    float laneLength = lane->getLength();

    for (float dist = 0; dist < laneLength; dist += markerSpacing) {
        Vec3 markerPos = lane->getPointAtDistance(dist);

        // Render marker as a small sphere or cube
        // This would use the debug renderer in the scene
        AV_DEBUG("Lane %d marker at (%.2f, %.2f, %.2f)", lane->getId(),
                 markerPos.x(), markerPos.y(), markerPos.z());
    }
}

void WorldRenderer::renderTrafficVehicles(const std::vector<std::shared_ptr<TrafficVehicle>>& vehicles) {
    if (!scene_) return;

    for (const auto& vehicle : vehicles) {
        if (!vehicle) continue;

        Vec3 position = vehicle->getPosition();
        Vec3 color = vehicle->getColor();

        // Render vehicle as a cube at its position
        // In a real implementation, would use actual vehicle mesh
        AV_DEBUG("Rendering traffic vehicle %d at (%.2f, %.2f, %.2f)",
                 vehicle->getId(), position.x(), position.y(), position.z());

        // Get current state for debug visualization
        float speed = vehicle->getSpeed();
        int laneId = vehicle->getCurrentLane() ? vehicle->getCurrentLane()->getId() : -1;
        float distAlongLane = vehicle->getDistanceAlongLane();

        AV_DEBUG("  Speed: %.2f m/s, Lane: %d, Distance: %.2f m",
                 speed, laneId, distAlongLane);
    }
}

void WorldRenderer::renderPedestrians(const std::vector<std::shared_ptr<Pedestrian>>& pedestrians) {
    if (!scene_) return;

    for (const auto& pedestrian : pedestrians) {
        if (!pedestrian) continue;

        Vec3 position = pedestrian->getPosition();
        Vec3 targetPos = pedestrian->getTargetPosition();
        Vec3 color = pedestrian->getColor();

        // Render pedestrian as a sphere at its position
        AV_DEBUG("Rendering pedestrian %d at (%.2f, %.2f, %.2f)",
                 pedestrian->getId(), position.x(), position.y(), position.z());

        // Show path to target
        if (position.distance(targetPos) > pedestrian->getArrivalThreshold()) {
            AV_DEBUG("  Target: (%.2f, %.2f, %.2f), Distance: %.2f m",
                     targetPos.x(), targetPos.y(), targetPos.z(),
                     position.distance(targetPos));
        }
    }
}

void WorldRenderer::renderIntersections(const std::vector<std::shared_ptr<Intersection>>& intersections) {
    if (!scene_) return;

    for (const auto& intersection : intersections) {
        if (!intersection) continue;

        const Vec3& position = intersection->getPosition();
        float radius = intersection->getRadius();

        // Render intersection area
        renderIntersectionRadius(position, radius);

        // Render traffic lights
        if (showTrafficLights_) {
            for (size_t i = 0; i < intersection->getTrafficLightCount(); ++i) {
                auto light = intersection->getTrafficLight(i);
                if (light) {
                    renderTrafficLight(light, position);
                }
            }
        }

        AV_DEBUG("Rendering intersection %d at (%.2f, %.2f, %.2f) with radius %.2f m",
                 intersection->getId(), position.x(), position.y(), position.z(), radius);
    }
}

void WorldRenderer::renderIntersectionRadius(const Vec3& position, float radius) {
    // Render intersection as a circle on the ground
    // This would use the debug renderer
    AV_DEBUG("Rendering intersection circle at (%.2f, %.2f, %.2f) radius=%.2f",
             position.x(), position.y(), position.z(), radius);
}

void WorldRenderer::renderTrafficLight(std::shared_ptr<TrafficLight> light, const Vec3& position) {
    if (!light) return;

    // Determine color based on state
    const float* color = nullptr;
    const char* stateName = "UNKNOWN";

    switch (light->getState()) {
        case TrafficLight::State::GREEN:
            color = TRAFFIC_LIGHT_GREEN;
            stateName = "GREEN";
            break;
        case TrafficLight::State::YELLOW:
            color = TRAFFIC_LIGHT_YELLOW;
            stateName = "YELLOW";
            break;
        case TrafficLight::State::RED:
            color = TRAFFIC_LIGHT_RED;
            stateName = "RED";
            break;
    }

    // Render traffic light indicator
    AV_DEBUG("Rendering traffic light %d at (%.2f, %.2f, %.2f) - State: %s",
             light->getId(), position.x(), position.y(), position.z(), stateName);
}

} // namespace av
