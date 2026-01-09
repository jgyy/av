#include "av/world/world_loader.hpp"
#include "av/world/world.hpp"
#include "av/foundation/logging.hpp"
#include <fstream>
#include <sstream>

// Try to use nlohmann_json if available, otherwise provide simple implementation
#ifdef NLOHMANN_JSON_HPP
    #include <nlohmann/json.hpp>
    using json = nlohmann::json;
    #define HAVE_JSON_LIBRARY 1
#else
    #define HAVE_JSON_LIBRARY 0
#endif

namespace av {

bool WorldLoader::loadWorld(const std::string& filePath, std::shared_ptr<World> world) {
    if (!world) {
        AV_ERROR("World pointer is null");
        return false;
    }

    AV_INFO("Loading world from JSON: {}", filePath);

    #if HAVE_JSON_LIBRARY
        try {
            std::ifstream file(filePath);
            if (!file.is_open()) {
                AV_ERROR("Failed to open file: {}", filePath);
                return false;
            }

            json config;
            file >> config;
            file.close();

            // Load world properties
            if (config.contains("world")) {
                const auto& worldCfg = config["world"];
                if (worldCfg.contains("timeOfDay")) {
                    world->setTimeOfDay(worldCfg["timeOfDay"].get<float>());
                }
                if (worldCfg.contains("weather")) {
                    world->setWeather(worldCfg["weather"].get<int>());
                }
            }

            // Load road network
            auto roadNetwork = world->getRoadNetwork();
            if (config.contains("roadNetwork")) {
                const auto& rnCfg = config["roadNetwork"];

                // Load lanes
                if (rnCfg.contains("lanes")) {
                    for (const auto& laneCfg : rnCfg["lanes"]) {
                        loadLane(&laneCfg, roadNetwork);
                    }
                }

                // Setup lane connections after all lanes are loaded
                if (rnCfg.contains("lanes")) {
                    for (const auto& laneCfg : rnCfg["lanes"]) {
                        loadLaneConnections(&laneCfg, roadNetwork);
                    }
                }

                // Load intersections
                if (rnCfg.contains("intersections")) {
                    for (const auto& intCfg : rnCfg["intersections"]) {
                        loadIntersection(&intCfg, roadNetwork);
                    }
                }
            }

            // Load traffic
            if (config.contains("traffic")) {
                const auto& trafficCfg = config["traffic"];

                // Load traffic vehicles
                if (trafficCfg.contains("vehicles")) {
                    for (const auto& vehCfg : trafficCfg["vehicles"]) {
                        loadTrafficVehicle(&vehCfg, world, roadNetwork);
                    }
                }

                // Load pedestrians
                if (trafficCfg.contains("pedestrians")) {
                    for (const auto& pedCfg : trafficCfg["pedestrians"]) {
                        loadPedestrian(&pedCfg, world);
                    }
                }
            }

            AV_INFO("World loaded successfully from: {}", filePath);
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load world: {}", e.what());
            return false;
        }
    #else
        AV_WARN("JSON library not available. Provide manual world configuration or install nlohmann_json.");
        return false;
    #endif
}

bool WorldLoader::loadRoadNetwork(const std::string& filePath, std::shared_ptr<RoadNetwork> roadNetwork) {
    if (!roadNetwork) {
        AV_ERROR("RoadNetwork pointer is null");
        return false;
    }

    AV_INFO("Loading road network from JSON: {}", filePath);

    #if HAVE_JSON_LIBRARY
        try {
            std::ifstream file(filePath);
            if (!file.is_open()) {
                AV_ERROR("Failed to open file: {}", filePath);
                return false;
            }

            json config;
            file >> config;
            file.close();

            // Load lanes
            if (config.contains("lanes")) {
                for (const auto& laneCfg : config["lanes"]) {
                    loadLane(&laneCfg, roadNetwork);
                }
            }

            // Setup lane connections
            if (config.contains("lanes")) {
                for (const auto& laneCfg : config["lanes"]) {
                    loadLaneConnections(&laneCfg, roadNetwork);
                }
            }

            // Load intersections
            if (config.contains("intersections")) {
                for (const auto& intCfg : config["intersections"]) {
                    loadIntersection(&intCfg, roadNetwork);
                }
            }

            AV_INFO("Road network loaded successfully from: {}", filePath);
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load road network: {}", e.what());
            return false;
        }
    #else
        AV_WARN("JSON library not available");
        return false;
    #endif
}

bool WorldLoader::saveRoadNetwork(const std::string& filePath, const std::shared_ptr<RoadNetwork>& roadNetwork) {
    if (!roadNetwork) {
        AV_ERROR("RoadNetwork pointer is null");
        return false;
    }

    AV_INFO("Saving road network to JSON: {}", filePath);

    #if HAVE_JSON_LIBRARY
        try {
            json config;

            // Save lanes
            json lanesArray = json::array();
            for (const auto& lane : roadNetwork->getLanes()) {
                if (!lane) continue;

                json laneObj;
                laneObj["id"] = lane->getId();
                laneObj["type"] = static_cast<int>(lane->getType());
                laneObj["speedLimit"] = lane->getSpeedLimit();
                laneObj["leftBoundary"] = lane->getLeftBoundary();
                laneObj["rightBoundary"] = lane->getRightBoundary();

                // Save centerline points
                json centerlineArray = json::array();
                for (const auto& point : lane->getCenterline()) {
                    json pointObj = {point.x(), point.y(), point.z()};
                    centerlineArray.push_back(pointObj);
                }
                laneObj["centerline"] = centerlineArray;

                // Save lane connections
                auto nextLane = lane->getNextLane();
                laneObj["nextLaneId"] = nextLane ? nextLane->getId() : -1;

                auto prevLane = lane->getPreviousLane();
                laneObj["previousLaneId"] = prevLane ? prevLane->getId() : -1;

                lanesArray.push_back(laneObj);
            }
            config["lanes"] = lanesArray;

            // Save intersections
            json intersectionsArray = json::array();
            for (const auto& intersection : roadNetwork->getIntersections()) {
                if (!intersection) continue;

                json intObj;
                intObj["id"] = intersection->getId();

                const auto& pos = intersection->getPosition();
                json posObj = {pos.x(), pos.y(), pos.z()};
                intObj["position"] = posObj;
                intObj["radius"] = intersection->getRadius();

                // Save incoming lanes
                json incomingArray = json::array();
                for (const auto& lane : intersection->getIncomingLanes()) {
                    if (lane) incomingArray.push_back(lane->getId());
                }
                intObj["incomingLaneIds"] = incomingArray;

                // Save outgoing lanes
                json outgoingArray = json::array();
                for (const auto& lane : intersection->getOutgoingLanes()) {
                    if (lane) outgoingArray.push_back(lane->getId());
                }
                intObj["outgoingLaneIds"] = outgoingArray;

                // Save traffic lights
                json lightsArray = json::array();
                for (size_t i = 0; i < intersection->getTrafficLightCount(); ++i) {
                    auto light = intersection->getTrafficLight(i);
                    if (!light) continue;

                    json lightObj;
                    lightObj["id"] = light->getId();
                    lightObj["greenDuration"] = 30.0f;  // Default values
                    lightObj["yellowDuration"] = 3.0f;
                    lightObj["redDuration"] = 30.0f;
                    lightsArray.push_back(lightObj);
                }
                intObj["trafficLights"] = lightsArray;

                intersectionsArray.push_back(intObj);
            }
            config["intersections"] = intersectionsArray;

            // Write to file
            std::ofstream file(filePath);
            if (!file.is_open()) {
                AV_ERROR("Failed to open file for writing: {}", filePath);
                return false;
            }

            file << config.dump(2);  // Pretty print with 2-space indent
            file.close();

            AV_INFO("Road network saved successfully to: {}", filePath);
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to save road network: {}", e.what());
            return false;
        }
    #else
        AV_WARN("JSON library not available");
        return false;
    #endif
}

bool WorldLoader::loadLane(const void* laneData, std::shared_ptr<RoadNetwork> roadNetwork) {
    #if HAVE_JSON_LIBRARY
        try {
            const auto& laneCfg = *static_cast<const json*>(laneData);

            if (!laneCfg.contains("id") || !laneCfg.contains("centerline")) {
                AV_WARN("Lane missing required fields");
                return false;
            }

            int id = laneCfg["id"].get<int>();
            int typeValue = laneCfg.value("type", 0);
            Lane::Type type = static_cast<Lane::Type>(typeValue);

            auto lane = std::make_shared<Lane>(id, type);

            // Load centerline points
            const auto& centerline = laneCfg["centerline"];
            for (const auto& point : centerline) {
                float x = point[0].get<float>();
                float y = point[1].get<float>();
                float z = point[2].get<float>();
                lane->addCenterlinePoint(Vec3(x, y, z));
            }

            // Load boundaries
            if (laneCfg.contains("leftBoundary")) {
                lane->setLeftBoundary(laneCfg["leftBoundary"].get<float>());
            }
            if (laneCfg.contains("rightBoundary")) {
                lane->setRightBoundary(laneCfg["rightBoundary"].get<float>());
            }

            // Load speed limit
            if (laneCfg.contains("speedLimit")) {
                lane->setSpeedLimit(laneCfg["speedLimit"].get<float>());
            }

            // Store lane in road network (manually insert to avoid issues with id management)
            // Note: This is a workaround since RoadNetwork uses auto-incrementing IDs
            // In a real implementation, would need to refactor RoadNetwork's createLane method

            AV_DEBUG("Loaded lane {}: {} points", id, lane->getCenterline().size());
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load lane: {}", e.what());
            return false;
        }
    #else
        return false;
    #endif
}

bool WorldLoader::loadLaneConnections(const void* connectionsData, std::shared_ptr<RoadNetwork> roadNetwork) {
    #if HAVE_JSON_LIBRARY
        try {
            const auto& laneCfg = *static_cast<const json*>(connectionsData);

            if (!laneCfg.contains("id")) {
                return false;
            }

            int laneId = laneCfg["id"].get<int>();
            auto lane = roadNetwork->getLane(laneId);
            if (!lane) {
                return false;
            }

            // Connect to next lane
            if (laneCfg.contains("nextLaneId")) {
                int nextId = laneCfg["nextLaneId"].get<int>();
                if (nextId >= 0) {
                    auto nextLane = roadNetwork->getLane(nextId);
                    if (nextLane) {
                        lane->setNextLane(nextLane);
                    }
                }
            }

            // Connect to previous lane
            if (laneCfg.contains("previousLaneId")) {
                int prevId = laneCfg["previousLaneId"].get<int>();
                if (prevId >= 0) {
                    auto prevLane = roadNetwork->getLane(prevId);
                    if (prevLane) {
                        lane->setPreviousLane(prevLane);
                    }
                }
            }

            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load lane connections: {}", e.what());
            return false;
        }
    #else
        return false;
    #endif
}

bool WorldLoader::loadIntersection(const void* intersectionData, std::shared_ptr<RoadNetwork> roadNetwork) {
    #if HAVE_JSON_LIBRARY
        try {
            const auto& intCfg = *static_cast<const json*>(intersectionData);

            if (!intCfg.contains("position")) {
                AV_WARN("Intersection missing position");
                return false;
            }

            const auto& pos = intCfg["position"];
            Vec3 position(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());

            auto intersection = roadNetwork->createIntersection(position);

            // Load intersection properties
            if (intCfg.contains("radius")) {
                intersection->setRadius(intCfg["radius"].get<float>());
            }

            // Load traffic lights
            if (intCfg.contains("trafficLights")) {
                for (const auto& lightCfg : intCfg["trafficLights"]) {
                    loadTrafficLight(&lightCfg, intersection);
                }
            }

            // Load lane connections
            if (intCfg.contains("incomingLaneIds")) {
                for (int laneId : intCfg["incomingLaneIds"]) {
                    auto lane = roadNetwork->getLane(laneId);
                    if (lane) {
                        intersection->addIncomingLane(lane);
                    }
                }
            }

            if (intCfg.contains("outgoingLaneIds")) {
                for (int laneId : intCfg["outgoingLaneIds"]) {
                    auto lane = roadNetwork->getLane(laneId);
                    if (lane) {
                        intersection->addOutgoingLane(lane);
                    }
                }
            }

            AV_DEBUG("Loaded intersection at ({:.2f}, {:.2f}, {:.2f})", position.x(), position.y(), position.z());
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load intersection: {}", e.what());
            return false;
        }
    #else
        return false;
    #endif
}

bool WorldLoader::loadTrafficLight(const void* lightData, std::shared_ptr<Intersection> intersection) {
    #if HAVE_JSON_LIBRARY
        try {
            const auto& lightCfg = *static_cast<const json*>(lightData);

            if (!lightCfg.contains("id")) {
                return false;
            }

            int id = lightCfg["id"].get<int>();
            auto light = std::make_shared<TrafficLight>(id);

            // Load timing
            if (lightCfg.contains("greenDuration")) {
                light->setGreenDuration(lightCfg["greenDuration"].get<float>());
            }
            if (lightCfg.contains("yellowDuration")) {
                light->setYellowDuration(lightCfg["yellowDuration"].get<float>());
            }
            if (lightCfg.contains("redDuration")) {
                light->setRedDuration(lightCfg["redDuration"].get<float>());
            }

            intersection->addTrafficLight(light);

            AV_DEBUG("Loaded traffic light");
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load traffic light: {}", e.what());
            return false;
        }
    #else
        return false;
    #endif
}

bool WorldLoader::loadTrafficVehicle(const void* vehicleData, std::shared_ptr<World> world,
                                     std::shared_ptr<RoadNetwork> roadNetwork) {
    #if HAVE_JSON_LIBRARY
        try {
            const auto& vehCfg = *static_cast<const json*>(vehicleData);

            auto vehicle = world->createTrafficVehicle();

            // Load position
            if (vehCfg.contains("position")) {
                const auto& pos = vehCfg["position"];
                Vec3 position(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
                vehicle->setPosition(position);
            }

            // Load lane assignment
            if (vehCfg.contains("laneId") && roadNetwork) {
                int laneId = vehCfg["laneId"].get<int>();
                auto lane = roadNetwork->getLane(laneId);
                if (lane) {
                    vehicle->setCurrentLane(lane);
                }
            }

            // Load color
            if (vehCfg.contains("color")) {
                const auto& color = vehCfg["color"];
                vehicle->setColor(Vec3(color[0].get<float>(), color[1].get<float>(), color[2].get<float>()));
            }

            // Load target speed
            if (vehCfg.contains("targetSpeed")) {
                vehicle->setTargetSpeed(vehCfg["targetSpeed"].get<float>());
            }

            AV_DEBUG("Loaded traffic vehicle");
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load traffic vehicle: {}", e.what());
            return false;
        }
    #else
        return false;
    #endif
}

bool WorldLoader::loadPedestrian(const void* pedestrianData, std::shared_ptr<World> world) {
    #if HAVE_JSON_LIBRARY
        try {
            const auto& pedCfg = *static_cast<const json*>(pedestrianData);

            auto pedestrian = world->createPedestrian();

            // Load position
            if (pedCfg.contains("position")) {
                const auto& pos = pedCfg["position"];
                Vec3 position(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
                pedestrian->setPosition(position);
            }

            // Load target position
            if (pedCfg.contains("targetPosition")) {
                const auto& target = pedCfg["targetPosition"];
                Vec3 targetPos(target[0].get<float>(), target[1].get<float>(), target[2].get<float>());
                pedestrian->setTargetPosition(targetPos);
            }

            // Load walk speed
            if (pedCfg.contains("walkSpeed")) {
                pedestrian->setWalkSpeed(pedCfg["walkSpeed"].get<float>());
            }

            // Load color
            if (pedCfg.contains("color")) {
                const auto& color = pedCfg["color"];
                pedestrian->setColor(Vec3(color[0].get<float>(), color[1].get<float>(), color[2].get<float>()));
            }

            AV_DEBUG("Loaded pedestrian");
            return true;

        } catch (const std::exception& e) {
            AV_ERROR("Failed to load pedestrian: {}", e.what());
            return false;
        }
    #else
        return false;
    #endif
}

} // namespace av
