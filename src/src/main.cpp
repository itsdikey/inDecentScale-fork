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
#include "ICON_BMP.h"
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
#define NUM_SAMPLE_AVG 5
#define HX711_GAIN_FACTOR 64
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET OLED_RESET_PIN

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

// tasks and mutex
SemaphoreHandle_t dataMutex;
TaskHandle_t TaskHandle_ReadSensors = NULL;
TaskHandle_t TaskHandle_Communication = NULL;
volatile uint64_t powerOnTime = 0;
#define AUTOSTANDBY 900000

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

    Serial.println("Initializing the scale");
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    delay(2000);

    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        Serial.println("Failed to create dataMutex");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize preferences (NVS) and load calibration factor
    preferences.begin("smartscales", false);
    factor = preferences.getFloat("calibrationFactor", factor);

    // Validate loaded factor: reject NaN or non-positive values
    if (std::isnan(factor) || factor <= 0.0f) {
        Serial.println("Invalid calibration factor in NVS; restoring default.");
        factor = 642.6133f; // default calibration factor
        // write default back immediately
        preferences.putFloat("calibrationFactor", factor);
    }

    Serial.printf("Calibration factor: %.4f\n", factor);

    // close NVS namespace until needed; reopen when saving
    preferences.end();

    xTaskCreate(taskReadSensors, "ReadSensors", 4096, NULL, 1, &TaskHandle_ReadSensors);
    xTaskCreate(taskCommunication, "Communication", 4096, NULL, 2, &TaskHandle_Communication);
}

void loop() {
    if ((esp_timer_get_time() - powerOnTime) >= (uint64_t)AUTOSTANDBY * 1000ULL) {
        //poweroff();
    }
    delay(1000);
}

void taskCommunication(void *parameter) {
    for (;;) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        timer_elapsed = timer.elapsedSeconds();
        scaleDisplay(1, weight, timer_elapsed);
        xSemaphoreGive(dataMutex);
        ButtonHandler::process();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void taskReadSensors(void *parameter) {
    scale.set_scale(factor);
    scale.set_gain(HX711_GAIN_FACTOR);
    delay(100);
    scale.tare();
    for (;;) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        if (scale.wait_ready_timeout(200)) {
            weight = scale.get_units(NUM_SAMPLE_AVG);
        }
        xSemaphoreGive(dataMutex);
         if (smartfunctionOn){
        // if (!isStable && (esp_timer_get_time() - lastStableTime) >= STABILITY_DURATION * 1000) {
        //     // Check if weight has remained stable for a certain duration
        //     if (fabs(weight - lastStableWeight) <= WEIGHT_STABILITY_THRESHOLD) {
        //         // Weight is stable
        //         lastStableTime = esp_timer_get_time(); // Record the time when weight became stable
        //         isStable = true; // Mark weight as stable
        //     }
        // } else if (isStable && (esp_timer_get_time() - lastStableTime) >= TIMER_PAUSE_DELAY * 1000) {
        //     // Check if weight is still stable after the pause delay
        //     if (fabs(weight - lastStableWeight) <= WEIGHT_STABILITY_THRESHOLD) {
        //         // Weight remains stable after the delay
        //         // timer(2); // Pause the timer
        //         isStable = false; // Mark weight as unstable
        //     } else {
        //         // Weight has changed, reset stability check
        //         isStable = false;
        //     }
        // }
        // // Check for weight increase after autotare
        // if (isTareComplete && weight - lastStableWeight >= WEIGHT_INCREASE_THRESHOLD) {
        //     // // Weight has increased by the threshold after autotare
        //     // // Start the timer after a delay
        //     // if ((esp_timer_get_time() - lastWeightIncreaseTime) >= TIMER_START_DELAY * 1000) {
        //     //     timer(1); // Start the timer
        //     // }}
        // }
        Serial.printf("Read weight: %.2f g, freeHeap=%u\n", weight, ESP.getFreeHeap());
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
    scale.tare();
    
    delay(2000);

    factor = (scale.get_units(10))/knownWeight;
    Serial.printf("New calibration factor: %.4f\n", factor);
    scale.set_scale(factor); // Set calibration factor
    // Save calibration factor to NVS: open namespace, write, then close
    preferences.begin("smartscales", false);
    preferences.putFloat("calibrationFactor", factor);
    // read back immediately to verify write
    float saved = preferences.getFloat("calibrationFactor", NAN);
    Serial.printf("Saved calibrationFactor (readback): %.4f\n", saved);
    preferences.end();
}