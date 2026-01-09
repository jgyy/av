#include "av/rendering/imgui_helper.hpp"
#include "av/foundation/logging.hpp"
#include <cstdio>
#include <cstdarg>
#include <cstring>

namespace av {

// Static member initialization
bool DebugUI::initialized_ = false;
DebugUI::Style DebugUI::style_;
float DebugUI::frameTimeBuffer_[60] = {};
int DebugUI::frameTimeIndex_ = 0;
int DebugUI::windowWidth_ = 1280;
int DebugUI::windowHeight_ = 720;

void DebugUI::init(int windowWidth, int windowHeight) {
    if (initialized_) {
        return;
    }
    windowWidth_ = windowWidth;
    windowHeight_ = windowHeight;
    initialized_ = true;
    AV_INFO("DebugUI initialized");
}

void DebugUI::beginFrame() {
    if (!initialized_) {
        return;
    }
    // In a real implementation, this would call ImGui::NewFrame()
}

void DebugUI::endFrame() {
    if (!initialized_) {
        return;
    }
    // In a real implementation, this would call ImGui::Render()
}

void DebugUI::shutdown() {
    initialized_ = false;
    AV_DEBUG("DebugUI shut down");
}

void DebugUI::text(const char* format, ...) {
    if (!initialized_) {
        return;
    }
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);
    std::fprintf(stderr, "\n");
}

void DebugUI::separator() {
    if (!initialized_) {
        return;
    }
    std::fprintf(stderr, "---\n");
}

bool DebugUI::button(const char* label) {
    if (!initialized_) {
        return false;
    }
    // In a real implementation with ImGui, would return button press state
    AV_DEBUG("Button: %s", label);
    return false;
}

bool DebugUI::checkbox(const char* label, bool& value) {
    if (!initialized_) {
        return false;
    }
    // In a real implementation, would toggle value and return true if changed
    return false;
}

bool DebugUI::sliderFloat(const char* label, float& value, float min, float max) {
    if (!initialized_) {
        return false;
    }
    // Clamp value
    if (value < min) value = min;
    if (value > max) value = max;
    return false;
}

bool DebugUI::inputFloat(const char* label, float& value) {
    if (!initialized_) {
        return false;
    }
    return false;
}

bool DebugUI::inputVec3(const char* label, Vec3& value) {
    if (!initialized_) {
        return false;
    }
    return false;
}

void DebugUI::pushID(int id) {
    if (!initialized_) {
        return;
    }
    // ImGui::PushID(id) equivalent
}

void DebugUI::popID() {
    if (!initialized_) {
        return;
    }
    // ImGui::PopID() equivalent
}

bool DebugUI::begin(const char* name, bool* open) {
    if (!initialized_) {
        return false;
    }
    AV_DEBUG("=== %s ===", name);
    return true;
}

void DebugUI::end() {
    if (!initialized_) {
        return;
    }
}

void DebugUI::recordFrameTime(float deltaTime) {
    if (!initialized_) {
        return;
    }
    frameTimeBuffer_[frameTimeIndex_] = deltaTime;
    frameTimeIndex_ = (frameTimeIndex_ + 1) % 60;
}

float DebugUI::getAverageFPS() {
    if (!initialized_) {
        return 0.0f;
    }
    float totalTime = 0.0f;
    for (int i = 0; i < 60; ++i) {
        totalTime += frameTimeBuffer_[i];
    }
    float avgTime = totalTime / 60.0f;
    if (avgTime > 0.0f) {
        return 1.0f / avgTime;
    }
    return 0.0f;
}

void DebugUI::displayStats() {
    if (!initialized_) {
        return;
    }
    if (!style_.showStats) {
        return;
    }

    if (begin("Statistics")) {
        if (style_.showFPS) {
            text("FPS: %.1f", getAverageFPS());
        }
        separator();
        end();
    }
}

} // namespace av
