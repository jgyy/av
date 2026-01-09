#pragma once

#include "av/foundation/math.hpp"
#include "av/world/road_network.hpp"
#include "av/world/traffic.hpp"
#include <memory>
#include <string>
#include <vector>

namespace av {

// Forward declarations
class World;

// JSON-based world and road network loader
class WorldLoader {
public:
    // Load complete world from JSON file
    static bool loadWorld(const std::string& filePath, std::shared_ptr<World> world);

    // Load road network from JSON file
    static bool loadRoadNetwork(const std::string& filePath, std::shared_ptr<RoadNetwork> roadNetwork);

    // Save road network to JSON file
    static bool saveRoadNetwork(const std::string& filePath, const std::shared_ptr<RoadNetwork>& roadNetwork);

private:
    // Lane loading helpers
    static bool loadLane(const void* laneData, std::shared_ptr<RoadNetwork> roadNetwork);
    static bool loadLaneConnections(const void* connectionsData, std::shared_ptr<RoadNetwork> roadNetwork);

    // Intersection loading helpers
    static bool loadIntersection(const void* intersectionData, std::shared_ptr<RoadNetwork> roadNetwork);
    static bool loadTrafficLight(const void* lightData, std::shared_ptr<Intersection> intersection);

    // Traffic vehicle loading
    static bool loadTrafficVehicle(const void* vehicleData, std::shared_ptr<World> world,
                                   std::shared_ptr<RoadNetwork> roadNetwork);

    // Pedestrian loading
    static bool loadPedestrian(const void* pedestrianData, std::shared_ptr<World> world);
};

// Example JSON structure for world configuration:
/*
{
  "world": {
    "timeOfDay": 12.0,
    "weather": 0,
    "bounds": {
      "min": [-100, -10, -100],
      "max": [100, 10, 100]
    }
  },
  "roadNetwork": {
    "lanes": [
      {
        "id": 0,
        "type": "DRIVING",
        "speedLimit": 13.4,
        "leftBoundary": 2.0,
        "rightBoundary": 2.0,
        "centerline": [
          [0, 0, 0], [10, 0, 0], [20, 0, 0]
        ],
        "nextLaneId": 1,
        "previousLaneId": -1
      }
    ],
    "intersections": [
      {
        "id": 0,
        "position": [20, 0, 0],
        "radius": 20.0,
        "incomingLaneIds": [0],
        "outgoingLaneIds": [1],
        "trafficLights": [
          {
            "id": 0,
            "greenDuration": 30.0,
            "yellowDuration": 3.0,
            "redDuration": 30.0
          }
        ]
      }
    ]
  },
  "traffic": {
    "vehicles": [
      {
        "id": 0,
        "position": [0, 1, 0],
        "laneId": 0,
        "color": [0.8, 0.2, 0.2],
        "targetSpeed": 13.4
      }
    ],
    "pedestrians": [
      {
        "id": 0,
        "position": [20, 0, 0],
        "targetPosition": [30, 0, 0],
        "walkSpeed": 1.4
      }
    ]
  }
}
*/

} // namespace av
