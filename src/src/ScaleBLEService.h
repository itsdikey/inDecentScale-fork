#ifndef SCALE_BLE_SERVICE_H
#define SCALE_BLE_SERVICE_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class ScaleBLEService {
public:
    ScaleBLEService(const char* deviceName, const char* serviceUUID, const char* characteristicUUID);
    void begin();
    void updateWeight(float weight);

private:
    const char* deviceName;
    const char* serviceUUID;
    const char* characteristicUUID;
    BLECharacteristic* pCharacteristic;
};

#endif