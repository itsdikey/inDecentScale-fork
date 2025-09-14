#pragma once

// Board pin mappings. Change the active block to match your target board.

// ========== ESP32 DevKit (default) ==========
// #define BOARD_NAME "ESP32-Dev"
// #define LOADCELL_DOUT_PIN 16
// #define LOADCELL_SCK_PIN 4
// #define I2C_SDA_PIN SDA
// #define I2C_SCL_PIN SCL
// #define OLED_RESET_PIN -1
//#define BUTTON_TIME 0
//#define BUTTON_TARE 1


// ========== ESP32-C3 Mini (example)
// To use C3 Mini, comment out the ESP32 Dev block above and uncomment the block below.
#define BOARD_NAME "ESP32-C3-Mini"
#define LOADCELL_DOUT_PIN 3
#define LOADCELL_SCK_PIN 4
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define OLED_RESET_PIN -1
#define BUTTON_TIME 5
#define BUTTON_TARE 6

// Helper macro to stringify board name
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
