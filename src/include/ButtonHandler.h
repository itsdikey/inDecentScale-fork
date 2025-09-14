#pragma once
#include <Arduino.h>

// A state machine for each button
enum ButtonState {
    IDLE,
    PRESSED_SHORT,
    PRESSED_LONG,
    RELEASED
};

// Function pointer for button callbacks
typedef void (*ButtonCallback)();

namespace ButtonHandler {

void begin();
void process();

void setTareCallback(ButtonCallback cb);
void setTimeCallback(ButtonCallback cb);
void setTareLongCallback(ButtonCallback cb);
void setTimeLongCallback(ButtonCallback cb);

// Read-only accessors for button state
ButtonState getTareState();
ButtonState getTimeState();
bool isTarePressed();
bool isTimePressed();
// Switch to polling mode (true = poll inside process(), false = use interrupts)
void setUsePolling(bool enable);
bool getUsePolling();

} // namespace ButtonHandler