#pragma once

#include <chrono>

namespace av {

// High-resolution clock for simulation timing
class Clock {
public:
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    using Duration = std::chrono::duration<float>;

    Clock() : started_(false), paused_(false), deltaTime_(0.0f) {
        reset();
    }

    void reset() {
        startTime_ = std::chrono::high_resolution_clock::now();
        lastFrameTime_ = startTime_;
        pauseTime_ = startTime_;
        started_ = true;
        paused_ = false;
        deltaTime_ = 0.0f;
    }

    // Start timing (if paused, resumes from pause time)
    void start() {
        if (!started_) {
            startTime_ = std::chrono::high_resolution_clock::now();
            lastFrameTime_ = startTime_;
            started_ = true;
        }
        if (paused_) {
            TimePoint now = std::chrono::high_resolution_clock::now();
            startTime_ += (now - pauseTime_);
            lastFrameTime_ = now;
            paused_ = false;
        }
    }

    // Pause timing
    void pause() {
        if (started_ && !paused_) {
            pauseTime_ = std::chrono::high_resolution_clock::now();
            paused_ = true;
        }
    }

    // Update and get delta time since last update
    float tick() {
        TimePoint now = std::chrono::high_resolution_clock::now();
        Duration duration = now - lastFrameTime_;
        deltaTime_ = duration.count();
        lastFrameTime_ = now;
        return deltaTime_;
    }

    // Get elapsed time since start
    float getElapsedTime() const {
        if (!started_) return 0.0f;

        TimePoint now = paused_ ? pauseTime_ : std::chrono::high_resolution_clock::now();
        Duration duration = now - startTime_;
        return duration.count();
    }

    // Get delta time from last tick
    float getDeltaTime() const {
        return deltaTime_;
    }

    // Check if clock is running
    bool isRunning() const {
        return started_ && !paused_;
    }

    bool isPaused() const {
        return paused_;
    }

private:
    TimePoint startTime_;
    TimePoint lastFrameTime_;
    TimePoint pauseTime_;
    float deltaTime_;
    bool started_;
    bool paused_;
};

// Simple timer for measuring specific code sections
class Timer {
public:
    Timer() : startTime_(std::chrono::high_resolution_clock::now()) {}

    // Get elapsed time in seconds
    float getElapsedSeconds() const {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = now - startTime_;
        return duration.count();
    }

    // Get elapsed time in milliseconds
    float getElapsedMilliseconds() const {
        return getElapsedSeconds() * 1000.0f;
    }

    // Reset timer
    void reset() {
        startTime_ = std::chrono::high_resolution_clock::now();
    }

private:
    std::chrono::high_resolution_clock::time_point startTime_;
};

} // namespace av
