#pragma once
#include <stdint.h>

// Timer command values passed to timer callbacks
// Use these named constants instead of raw numbers
enum TimerCommand : uint8_t {
    TIMER_PAUSE = 0, // pause/stop
    TIMER_START = 1, // start
    TIMER_RESET = 2  // reset/zero
};

// Map Decent protocol action byte to TimerCommand
static inline TimerCommand mapDecentTimerAction(uint8_t actionByte) {
    // Decent: 0x03 = start, 0x00 = stop, 0x02 = zero
    if (actionByte == 0x03) return TIMER_START;
    if (actionByte == 0x02) return TIMER_RESET;
    return TIMER_PAUSE;
}
