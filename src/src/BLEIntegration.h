#pragma once
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <string>
#include <functional>

void bleInit(const char* deviceName, const char* serviceUUID, const char* characteristicUUID);
void bleNotifyWeight(uint16_t weight, bool isStable, uint64_t elapsedMicros);

// Command callbacks (set these from main)
void setTareCallback(std::function<void()> cb);
void setTimerCallback(std::function<void(uint8_t state)> cb); // 0=reset,1=start,2=pause
void setLCDCallback(std::function<void(bool on)> cb);
void setPoweroffCallback(std::function<void()> cb);
void setCalibrateCallback(std::function<void()> cb);
void setFactorCallback(std::function<void(float)> cb);

// Connection callbacks
void setConnectedCallback(std::function<void()> cb);
void setDisconnectedCallback(std::function<void()> cb);

// helper hex conversion
std::string intToHexString(int value);
int hexStringToInt(const std::string& hexStr);
