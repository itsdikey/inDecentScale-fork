// Original SmartScales firmware - main program
#include <Arduino.h>
#include "soc/rtc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ScaleBLEService.h"
#include "TimerCommands.h"
#include "ICON_BMP.h"
#include "BLEIntegration.h"
#include "BoardConfig.h"
#include "ButtonHandler.h"
#include "Timer.h"
#include <stdio.h>
#include <Preferences.h>
#include <cmath>
// Autotare/weight stability
#define WEIGHT_STABILITY_THRESHOLD 1.0
#define TIMER_START_DELAY 5000
#define WEIGHT_INCREASE_THRESHOLD 0.5
#define STABILITY_DURATION 5000
#define TIMER_PAUSE_DELAY 2000
#define DEBOUNCE_TIME 50

#define HX711_GAIN_FACTOR 64
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET OLED_RESET_PIN
//#define TEN_HZ
#define EIGHTY_HZ

#ifdef TEN_HZ
    #define NUM_SAMPLE_AVG 5
#elif defined(EIGHTY_HZ)
    #define NUM_SAMPLE_AVG 8
#endif

bool smartfunctionOn = true;

// Function prototypes
void taskReadSensors(void *parameter);
void taskCommunication(void *parameter);
void scaleDisplay(uint8_t mode, float weight = 0.0f, uint64_t time = 0);
void tare_short_pressed();
void tare_long_pressed();
void time_short_pressed();
void time_long_pressed();
void calibrateScale(float knownWeight);
void poweroff();
void wakeup();

// Refactor helpers
void loadCalibrationFactor();
void saveCalibrationFactor(float f);
void registerBLECallbacks();
void showTransientMessage(const char *msg, uint32_t ms = 1000);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HX711 scale;
float reading;
float lastReading;
float factor = 642.6133;
float weight = 10;
uint64_t timer_elapsed;
uint64_t startTime;
Timer timer;
Preferences preferences;

float lastStableWeight = 0; // Variable to store the last stable weight
uint64_t lastStableTime = 0; // Timestamp of the last stable weight
bool isStable = false; // Flag to indicate if weight is currently stable
bool isTareComplete = false; // Flag to indicate if autotare has been completed
uint64_t lastWeightIncreaseTime = 0; // Timestamp of the last weight increase
volatile bool isCalibrating = false; // Flag to indicate calibration mode

bool isConnected = false;

// tasks and mutex
SemaphoreHandle_t dataMutex;
TaskHandle_t TaskHandle_ReadSensors = NULL;
TaskHandle_t TaskHandle_Communication = NULL;
volatile uint64_t powerOnTime = 0;
#define AUTOSTANDBY 30000

// BLE defaults
const char* deviceName = "DecentScale";
const char* serviceUUID = "0000FFF0-0000-1000-8000-00805F9B34FB";
const char* characteristicUUID = "0000FFF4-0000-1000-8000-00805F9B34FB";

void setup() {
    Serial.begin(115200);
    Serial.println("Booting...");
    powerOnTime = esp_timer_get_time();

    ButtonHandler::begin();
    ButtonHandler::setTareCallback(tare_short_pressed);
    ButtonHandler::setTareLongCallback(tare_long_pressed);
    ButtonHandler::setTimeCallback(time_short_pressed);
    ButtonHandler::setTimeLongCallback(time_long_pressed);
    //ButtonHandler::setUsePolling(true);
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;) ;
    }
    delay(2000);
    display.display();
    scaleDisplay(0);

    Serial.println("Initializing the scale this is the new code I swear");
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    delay(2000);

    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("Failed to create dataMutex");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize preferences (NVS) and load calibration factor
    loadCalibrationFactor();

    xTaskCreate(taskReadSensors, "ReadSensors", 4096, NULL, 1, &TaskHandle_ReadSensors);
    xTaskCreate(taskCommunication, "Communication", 4096, NULL, 2, &TaskHandle_Communication);

    // Register BLE callbacks and handlers
    registerBLECallbacks();
}

void loop() {
    if ((esp_timer_get_time() - powerOnTime) >= (uint64_t)AUTOSTANDBY * 1000ULL) {
          poweroff();
    }
    delay(1000);
}

void taskCommunication(void *parameter) {
    // Initialize BLE from integration module
    bleInit(deviceName, serviceUUID, characteristicUUID);

    for (;;) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        timer_elapsed = timer.elapsedSeconds();
        scaleDisplay(1, weight, timer_elapsed);
        // notify weight over BLE
        
        bleNotifyWeight((uint16_t)roundf(weight), isStable, timer_elapsed * 1000000ULL);

        xSemaphoreGive(dataMutex);
        ButtonHandler::process();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

float weights[3];
float temp;

float getWeight(){
    #if defined(TEN_HZ)
    weight = scale.get_units(NUM_SAMPLE_AVG);
    return weight;
    #elif defined(EIGHTY_HZ)

    weights[0] = scale.get_units(NUM_SAMPLE_AVG);
    delay(5); // small delay to allow HX711 to settle
    weights[1] = scale.get_units(NUM_SAMPLE_AVG);
    delay(5);
    weights[2] = scale.get_units(NUM_SAMPLE_AVG);
    //sort weights array with just if-s
    if(weights[0]>weights[1]){
        temp = weights[0];
        weights[0] = weights[1];
        weights[1] = temp;
    }
    if(weights[0]>weights[2]){
        temp = weights[0];
        weights[0] = weights[2];
        weights[2] = temp;
    }
    if(weights[1]>weights[2]){
        temp = weights[1];
        weights[1] = weights[2];
        weights[2] = temp;
    }

    // low-pass filter
    //weights[1] = (weights[1]*0.6) + (weights[0]*0.2) + (weights[2]*0.2);

    return weights[1];
    #endif
}

void taskReadSensors(void *parameter) {
    scale.set_scale(factor);
    scale.set_gain(HX711_GAIN_FACTOR);
    delay(100);
    scale.tare();
    for (;;) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        if (!isCalibrating) {
            if (scale.wait_ready_timeout(200)) {
                weight = getWeight();
            }
        }
        xSemaphoreGive(dataMutex);
        if (smartfunctionOn){
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}


void scaleDisplay(uint8_t mode, float weight, uint64_t time) {
    int CHAR_WIDTH, totalHeight, timerWidth, separatorWidth, weightWidth, totalWidth, startX, startY;
    uint32_t seconds = (uint32_t)time;
    uint32_t minutes = seconds / 60;
    uint32_t secs = seconds % 60;
    char timeBuf[8];
    char weightBuf[16];
    int grams = (int)weight;
    int dec = (int)roundf((weight - (float)grams) * 10.0f);

    if(fabs(weight)>15000 || isnan(factor) || isnan(weight)){
        mode = 4;
    } else if(fabs(weight)>1000){
        mode = 5;
    }
    // Always show calibration message during calibration
    if (isCalibrating) {
        mode = 2;
    }

    switch(mode) {
        case 0:
            display.ssd1306_command(SSD1306_DISPLAYON);
            display.clearDisplay();
            display.drawBitmap(0, 0, bootload, 128, 32, WHITE);
            display.display();
            break;
        case 1:
            CHAR_WIDTH = 6;
            timerWidth = 4 * CHAR_WIDTH + 1;
            separatorWidth = 1;
            weightWidth = 5 * CHAR_WIDTH + 1;
            totalWidth = timerWidth + separatorWidth + weightWidth;
            totalHeight = 8;
            
            display.clearDisplay();

            if(smartfunctionOn && isConnected) {
                display.drawBitmap(0, 0, epd_bitmap_bluetooth, 128, 32, WHITE);
            }

            display.setTextSize(1);
            display.setTextColor(WHITE);
            // top row reserved (no button text)

            startX = (128 - totalWidth) / 2;
            startY = (display.height() - 8) / 2;

            snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u", (unsigned)minutes, (unsigned)secs);
            display.setCursor(startX, startY);
            display.print(timeBuf);

            display.setCursor(startX + timerWidth + 6, startY);
            display.print("|");

            grams = (int)weight;
            dec = (int)roundf((weight - (float)grams) * 10.0f);
            if (dec == 10) { grams += 1; dec = 0; }
            snprintf(weightBuf, sizeof(weightBuf), "%03d.%1d", grams, abs(dec));
            display.setCursor(startX + timerWidth + separatorWidth + 12, startY);
            display.print(weightBuf);
            display.print("g");

            display.display();
            break;
        case 2:
            display.clearDisplay();
            display.setCursor(0, 16);
            display.println("Calibrate with 50g weight");
            display.display();
            break;
        case 4:
            display.clearDisplay();
            display.setCursor(0, 16);
            display.println("Needs Calibration!");
            display.display();
            break;
        case 5:
            display.clearDisplay();
            display.setCursor(0, 16);
            display.println("Overload!");
            display.display();
            break;
        case 255:
            display.clearDisplay();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            break;
        default:
            break;
    }
}

void tare_short_pressed() {
    Serial.println("Tare button short press detected. Taring scale.");
    scale.tare();
}

void tare_long_pressed() {
    Serial.println("Tare button long press detected. Entering calibration mode.");
    calibrateScale(50.0f); // Calibrate with a 50g weight
    scaleDisplay(2);
}

void time_short_pressed() {
    Serial.println("Time button short press detected. Toggling timer.");
    timer.toggle();
}

void time_long_pressed() {
    Serial.println("Time button long press detected. Resetting timer.");
    timer.reset();
}

void calibrateScale(float knownWeight) {
    const int SAMPLES = 20;
    const int SAMPLE_DELAY_MS = 100;

    scale.set_scale(1);
    // Step 1: Ask user to remove all weight
    display.clearDisplay();
    display.setCursor(0, 16);
    display.println("Remove all weight");
    display.display();
    delay(5000);

    scale.tare();
    // Step 2: Ask user to wait after taring
    display.clearDisplay();
    display.setCursor(0, 16);
    display.println("Wait after taring...");
    display.display();
    delay(1000);

    // Step 3: Ask user to place known weight
    display.clearDisplay();
    display.setCursor(0, 16);
    display.println("Place 50g weight");
    display.display();
    delay(5000);

    factor = (scale.get_units(10))/knownWeight;
    Serial.printf("New calibration factor: %.4f\n", factor);
    scale.set_scale(factor); // Set calibration factor
    // Save calibration factor to NVS: open namespace, write, then close
    preferences.begin("smartscales", false);
    preferences.putFloat("CF", factor);
    // read back immediately to verify write
    float saved = preferences.getFloat("CF", NAN);
    Serial.printf("Saved CF (readback): %.4f\n", saved);
    preferences.end();
}

void poweroff() {

    // Serial.println("Powering off...");
    // Serial.println("Power off not implemented yet");
    // Serial.println("Returning to main loop");
    // Serial.println("Remove this return statement when power off is implemented and external PSU is used");
    return;

    // Attach wakeup interrupt on the load cell data pin
    attachInterrupt(digitalPinToInterrupt(LOADCELL_DOUT_PIN), wakeup, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_TARE), wakeup, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_TIME), wakeup, FALLING);
    timer.reset();
    timer.pause();
    scaleDisplay(255); // turn off display
    // Power down HX711 to save energy
    scale.power_down();
    delay(10);
    // Enter deep sleep until DOUT pulls low (or external wake)
    esp_deep_sleep_start();
}

void wakeup() {
    // Detach the wakeup interrupt handler
    detachInterrupt(digitalPinToInterrupt(LOADCELL_DOUT_PIN));
    detachInterrupt(digitalPinToInterrupt(BUTTON_TARE));
    detachInterrupt(digitalPinToInterrupt(BUTTON_TIME));
    // Power up HX711 and give it time to stabilise
    scale.power_up();
    delay(50);
    powerOnTime = esp_timer_get_time();
    scaleDisplay(0); // show boot/initial screen
}

// Load calibration factor from NVS and validate it
void loadCalibrationFactor() {
    preferences.begin("smartscales", false);
    factor = preferences.getFloat("CF", factor);
    if (std::isnan(factor) || factor <= 0.0f) {
        Serial.println("Invalid calibration factor in NVS; restoring default.");
        factor = 642.6133f; // default calibration factor
        preferences.putFloat("CF", factor);
    }
    Serial.printf("Calibration factor: %.4f\n", factor);
    preferences.end();
}

// Save calibration factor to NVS
void saveCalibrationFactor(float f) {
    preferences.begin("smartscales", false);
    preferences.putFloat("CF", f);
    preferences.end();
}

// Helper to show a brief on-screen message
void showTransientMessage(const char *msg, uint32_t ms) {
    display.clearDisplay();
    display.setCursor(0, 16);
    display.println(msg);
    display.display();
    delay(ms);
}

// Register BLE callbacks and handlers (extracted from setup for clarity)
void registerBLECallbacks() {
    setTareCallback(tare_short_pressed);
    setTimerCallback([](uint8_t state){ 
        if(state == TIMER_START) {
            timer.start();
        } else if(state == TIMER_PAUSE) {
            timer.pause();
        } else if(state == TIMER_RESET) {
            timer.reset();
        }
    });
    setLCDCallback([](bool on){ 
        //ignore the display commands
        return;
         if (on) scaleDisplay(0); else scaleDisplay(255);
    });
    setPoweroffCallback(poweroff);
    setCalibrateCallback([](){ calibrateScale(50.0f); });
    setFactorCallback([](float f){ factor = f; saveCalibrationFactor(f); });

    // Print/display on BLE connect/disconnect
    setConnectedCallback([](){
        Serial.println("[MAIN] BLE connected");
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        isConnected = true;
        xSemaphoreGive(dataMutex);
    });
    setDisconnectedCallback([](){
        Serial.println("[MAIN] BLE disconnected");
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        isConnected = false;
        xSemaphoreGive(dataMutex);
    });
}