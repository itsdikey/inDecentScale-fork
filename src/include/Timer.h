#pragma once

#include <stdint.h>
#include <stddef.h>

// Simple timer using ESP microsecond timer. Not tied to FreeRTOS tasks.
class Timer {
public:
    Timer();
    void start();        // start or resume
    void pause();        // pause (stop) and accumulate
    void reset();        // reset to zero and stop
    void toggle();       // start if stopped, pause if running

    bool isRunning() const;

    // elapsed time
    uint64_t elapsedMicros() const; // microseconds
    uint32_t elapsedSeconds() const; // whole seconds

    // formatted mm:ss into provided buffer (bufLen should be >=6)
    void format_mm_ss(char *buf, size_t bufLen) const;

private:
    bool _running;
    uint64_t _startUs;    // esp_timer_get_time() when started
    uint64_t _accumUs;    // accumulated microseconds while paused
};
