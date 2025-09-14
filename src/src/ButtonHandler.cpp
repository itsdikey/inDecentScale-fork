#include "ButtonHandler.h"
#include "BoardConfig.h"
#include <Arduino.h>

// Callbacks for short and long presses
static ButtonCallback tareCb = nullptr;
static ButtonCallback timeCb = nullptr;
static ButtonCallback tareLongCb = nullptr;
static ButtonCallback timeLongCb = nullptr;

// Button state variables
static volatile ButtonState tareButtonState = IDLE;
static volatile ButtonState timeButtonState = IDLE;
static volatile uint32_t tarePressTime = 0;
static volatile uint32_t timePressTime = 0;

static const uint32_t LONG_PRESS_MS = 1000; // 1 second for long press
static const uint32_t DEBOUNCE_MS = 50;
// runtime mode: false = interrupts (default), true = polling
static volatile bool usePolling = false;

// polling state (non-ISR)
static uint8_t tareRawLast = HIGH;
static uint8_t timeRawLast = HIGH;
static uint32_t tareLastBounce = 0;
static uint32_t timeLastBounce = 0;
// track whether long callback has already fired for the current press
static volatile bool tareLongFired = false;
static volatile bool timeLongFired = false;

// Interrupt Service Routines (ISRs)
static void IRAM_ATTR tare_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t currentMillis = millis();

    if (digitalRead(BUTTON_TARE) == LOW) { // Button press
        if (tareButtonState == IDLE) {
            tarePressTime = currentMillis;
            tareButtonState = PRESSED_SHORT;
        }
    } else { // Button release
        if (tareButtonState == PRESSED_SHORT || tareButtonState == PRESSED_LONG) {
            tareButtonState = RELEASED;
        }
    }

    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR time_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t currentMillis = millis();

    if (digitalRead(BUTTON_TIME) == LOW) { // Button press
        if (timeButtonState == IDLE) {
            timePressTime = currentMillis;
            timeButtonState = PRESSED_SHORT;
        }
    } else { // Button release
        if (timeButtonState == PRESSED_SHORT || timeButtonState == PRESSED_LONG) {
            timeButtonState = RELEASED;
        }
    }

    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

namespace ButtonHandler {

void begin() {
    pinMode(BUTTON_TARE, INPUT_PULLUP);
    pinMode(BUTTON_TIME, INPUT_PULLUP);
    
    // Attach interrupts by default; when polling is enabled process() will
    // perform reads and the ISR-driven state will be unused.
    attachInterrupt(digitalPinToInterrupt(BUTTON_TARE), tare_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_TIME), time_isr, FALLING);
}

void setTareCallback(ButtonCallback cb) {
    tareCb = cb;
}

void setTimeCallback(ButtonCallback cb) {
    timeCb = cb;
}

void setTareLongCallback(ButtonCallback cb) {
    tareLongCb = cb;
}

void setTimeLongCallback(ButtonCallback cb) {
    timeLongCb = cb;
}

void process() {
    uint32_t currentMillis = millis();

    if (getUsePolling()) {
        // POLLING MODE: sample pins and perform debounced edge detection here
        uint8_t tareRaw = digitalRead(BUTTON_TARE);
        if (tareRaw != tareRawLast) {
            tareLastBounce = currentMillis;
            tareRawLast = tareRaw;
        }
        if ((uint32_t)(currentMillis - tareLastBounce) >= DEBOUNCE_MS) {
            // stable state
            static uint8_t tareStable = HIGH;
            if (tareRaw != tareStable) {
                tareStable = tareRaw;
                if (tareStable == LOW) {
                    // press started
                    tarePressTime = currentMillis;
                    tareLongFired = false;
                } else {
                    // released
                    uint32_t dur = currentMillis - tarePressTime;
                    if (dur >= LONG_PRESS_MS) {
                        if (!tareLongFired && tareLongCb) tareLongCb();
                    } else {
                        if (tareCb) tareCb();
                    }
                }
            }
            // while held, check long press
            if (tareStable == LOW && !tareLongFired && (currentMillis - tarePressTime >= LONG_PRESS_MS)) {
                tareLongFired = true;
                if (tareLongCb) tareLongCb();
            }
        }

        uint8_t timeRaw = digitalRead(BUTTON_TIME);
        if (timeRaw != timeRawLast) {
            timeLastBounce = currentMillis;
            timeRawLast = timeRaw;
        }
        if ((uint32_t)(currentMillis - timeLastBounce) >= DEBOUNCE_MS) {
            static uint8_t timeStable = HIGH;
            if (timeRaw != timeStable) {
                timeStable = timeRaw;
                if (timeStable == LOW) {
                    timePressTime = currentMillis;
                    timeLongFired = false;
                } else {
                    uint32_t dur = currentMillis - timePressTime;
                    if (dur >= LONG_PRESS_MS) {
                        if (!timeLongFired && timeLongCb) timeLongCb();
                    } else {
                        if (timeCb) timeCb();
                    }
                }
            }
            if (timeStable == LOW && !timeLongFired && (currentMillis - timePressTime >= LONG_PRESS_MS)) {
                timeLongFired = true;
                if (timeLongCb) timeLongCb();
            }
        }

        // when polling, don't also run the ISR-driven state machine below
        return;
    }

    // Process Tare button state machine
    switch(tareButtonState) {
        case PRESSED_SHORT:
            if (currentMillis - tarePressTime >= LONG_PRESS_MS) {
                // Button reached long-press threshold: transition and fire callback once now
                tareButtonState = PRESSED_LONG;
                if (!tareLongFired) {
                    tareLongFired = true;
                    if (tareLongCb) tareLongCb();
                }
            } else if (digitalRead(BUTTON_TARE) == HIGH) {
                // Button released before long press threshold, call short press callback
                if (tareCb) tareCb();
                tareButtonState = IDLE; // Return to idle
                tareLongFired = false;
            }
            break;
        case PRESSED_LONG:
            // Button is being held after long fired; wait for release and clear flag
            if (digitalRead(BUTTON_TARE) == HIGH) {
                // Do not re-fire long callback on release
                tareButtonState = IDLE; // Return to idle
                tareLongFired = false;
            }
            break;
        case RELEASED: // This state is for a quick release before the debounce time passes
            if (currentMillis - tarePressTime >= DEBOUNCE_MS) {
                // We're past the debounce time, so this was a valid short press and release.
                // This state handles cases where the button is released very quickly.
                if (tareCb) tareCb();
            }
            tareButtonState = IDLE; // Always return to idle after processing a release
            break;
        default:
            // IDLE state, do nothing
            break;
    }

    // Process Time button state machine (identical logic)
    switch(timeButtonState) {
        case PRESSED_SHORT:
            if (currentMillis - timePressTime >= LONG_PRESS_MS) {
                timeButtonState = PRESSED_LONG;
                if (!timeLongFired) {
                    timeLongFired = true;
                    if (timeLongCb) timeLongCb();
                }
            } else if (digitalRead(BUTTON_TIME) == HIGH) {
                if (timeCb) timeCb();
                timeButtonState = IDLE;
                timeLongFired = false;
            }
            break;
        case PRESSED_LONG:
            if (digitalRead(BUTTON_TIME) == HIGH) {
                // Clear long-fired flag on release; don't call callback again
                timeButtonState = IDLE;
                timeLongFired = false;
            }
            break;
        case RELEASED:
            if (currentMillis - timePressTime >= DEBOUNCE_MS) {
                if (timeCb) timeCb();
            }
            timeButtonState = IDLE;
            break;
        default:
            break;
    }
}

// Accessors
ButtonState getTareState() {
    return tareButtonState;
}

ButtonState getTimeState() {
    return timeButtonState;
}

bool isTarePressed() {
    return (tareButtonState == PRESSED_SHORT || tareButtonState == PRESSED_LONG);
}

bool isTimePressed() {
    return (timeButtonState == PRESSED_SHORT || timeButtonState == PRESSED_LONG);
}

} // namespace ButtonHandler

void setUsePolling(bool enable) {
    usePolling = enable;
}

bool getUsePolling() {
    return usePolling;
}

// Namespace-qualified wrappers (match header declarations)
namespace ButtonHandler {
    void setUsePolling(bool enable) { ::setUsePolling(enable); }
    bool getUsePolling() { return ::getUsePolling(); }
}