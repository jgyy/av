#pragma once

#include "av/foundation/math.hpp"
#include "av/world/world.hpp"
#include <memory>
#include <vector>

namespace av {

// Forward declarations
class Scene;
class Renderer;

// Rendering integration for the world simulation
// Bridges World entities (lanes, vehicles, pedestrians, traffic lights) with rendering system
class WorldRenderer {
public:
    WorldRenderer(std::shared_ptr<Scene> scene);
    ~WorldRenderer();

    // Update and render world state
    void renderWorld(std::shared_ptr<World> world);

    // Render specific components
    void renderRoadNetwork(std::shared_ptr<RoadNetwork> roadNetwork);
    void renderTrafficVehicles(const std::vector<std::shared_ptr<TrafficVehicle>>& vehicles);
    void renderPedestrians(const std::vector<std::shared_ptr<Pedestrian>>& pedestrians);
    void renderIntersections(const std::vector<std::shared_ptr<Intersection>>& intersections);

    // Configure visibility
    void setShowLanes(bool show) { showLanes_ = show; }
    void setShowIntersections(bool show) { showIntersections_ = show; }
    void setShowTrafficVehicles(bool show) { showTrafficVehicles_ = show; }
    void setShowPedestrians(bool show) { showPedestrians_ = show; }
    void setShowTrafficLights(bool show) { showTrafficLights_ = show; }
    void setShowDebugGrid(bool show) { showDebugGrid_ = show; }

    bool getShowLanes() const { return showLanes_; }
    bool getShowIntersections() const { return showIntersections_; }
    bool getShowTrafficVehicles() const { return showTrafficVehicles_; }
    bool getShowPedestrians() const { return showPedestrians_; }
    bool getShowTrafficLights() const { return showTrafficLights_; }
    bool getShowDebugGrid() const { return showDebugGrid_; }

private:
    std::shared_ptr<Scene> scene_;

    // Visibility flags
    bool showLanes_ = true;
    bool showIntersections_ = true;
    bool showTrafficVehicles_ = true;
    bool showPedestrians_ = true;
    bool showTrafficLights_ = true;
    bool showDebugGrid_ = true;

    // Helper methods for rendering
    void renderLane(std::shared_ptr<Lane> lane);
    void renderLaneMarkers(std::shared_ptr<Lane> lane);
    void renderTrafficLight(std::shared_ptr<TrafficLight> light, const Vec3& position);
    void renderIntersectionRadius(const Vec3& position, float radius);

    // Colors for visualization
    static constexpr float LANE_COLOR[3] = {0.7f, 0.7f, 0.7f};
    static constexpr float LANE_MARKER_COLOR[3] = {1.0f, 1.0f, 0.0f};
    static constexpr float INTERSECTION_COLOR[3] = {0.8f, 0.4f, 0.2f};
    static constexpr float TRAFFIC_LIGHT_GREEN[3] = {0.0f, 1.0f, 0.0f};
    static constexpr float TRAFFIC_LIGHT_YELLOW[3] = {1.0f, 1.0f, 0.0f};
    static constexpr float TRAFFIC_LIGHT_RED[3] = {1.0f, 0.0f, 0.0f};
};

} // namespace av
