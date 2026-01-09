#pragma once

#include "av/foundation/math.hpp"
#include <string>
#include <functional>

namespace av {

// Simple ImGui-style debug UI helper (no external ImGui dependency)
class DebugUI {
public:
    struct Style {
        float windowX = 10.0f;
        float windowY = 10.0f;
        float windowWidth = 300.0f;
        float windowHeight = 400.0f;
        bool showFPS = true;
        bool showCameraInfo = true;
        bool showStats = true;
    };

    static void init(int windowWidth, int windowHeight);
    static void beginFrame();
    static void endFrame();
    static void shutdown();

    // UI Elements (would be implemented with ImGui in real version)
    static void text(const char* format, ...);
    static void separator();
    static bool button(const char* label);
    static bool checkbox(const char* label, bool& value);
    static bool sliderFloat(const char* label, float& value, float min, float max);
    static bool inputFloat(const char* label, float& value);
    static bool inputVec3(const char* label, Vec3& value);
    static void pushID(int id);
    static void popID();

    // Window management
    static bool begin(const char* name, bool* open = nullptr);
    static void end();

    // Statistics tracking
    static void recordFrameTime(float deltaTime);
    static float getAverageFPS();
    static void displayStats();

    static const Style& getStyle() { return style_; }
    static void setStyle(const Style& style) { style_ = style; }

private:
    static bool initialized_;
    static Style style_;
    static float frameTimeBuffer_[60];
    static int frameTimeIndex_;
    static int windowWidth_;
    static int windowHeight_;
};

} // namespace av
