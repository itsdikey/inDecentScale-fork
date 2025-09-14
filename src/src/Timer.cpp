#include "Timer.h"
#include <esp_timer.h>
#include <stdio.h>

Timer::Timer()
: _running(false), _startUs(0), _accumUs(0) {}

void Timer::start() {
    if (!_running) {
        _startUs = esp_timer_get_time();
        _running = true;
    }
}

void Timer::pause() {
    if (_running) {
        uint64_t now = esp_timer_get_time();
        _accumUs += (now - _startUs);
        _running = false;
    }
}

void Timer::reset() {
    _running = false;
    _startUs = 0;
    _accumUs = 0;
}

void Timer::toggle() {
    if (_running) pause(); else start();
}

bool Timer::isRunning() const {
    return _running;
}

uint64_t Timer::elapsedMicros() const {
    if (_running) {
        return _accumUs + (esp_timer_get_time() - _startUs);
    }
    return _accumUs;
}

uint32_t Timer::elapsedSeconds() const {
    return (uint32_t)(elapsedMicros() / 1000000ULL);
}

void Timer::format_mm_ss(char *buf, size_t bufLen) const {
    uint32_t s = elapsedSeconds();
    uint32_t m = s / 60;
    uint32_t sec = s % 60;
    if (bufLen >= 6) {
        snprintf(buf, bufLen, "%02u:%02u", (unsigned)m, (unsigned)sec);
    } else if (bufLen > 0) {
        buf[0] = '\0';
    }
}
