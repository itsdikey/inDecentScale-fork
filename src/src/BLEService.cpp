#include "ScaleBLEService.h"
#include "string.h"

ScaleBLEService::ScaleBLEService(const char* deviceName, const char* serviceUUID, const char* characteristicUUID)
    : deviceName(deviceName), serviceUUID(serviceUUID), characteristicUUID(characteristicUUID) {}

void ScaleBLEService::begin() {
    BLEDevice::init(deviceName);
    // BLEServer* pServer = BLEDevice::createServer();
    // BLEService* pService = pServer->createService(serviceUUID);
    // pCharacteristic = pService->createCharacteristic(
    //     characteristicUUID,
    //     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    // );
    // pCharacteristic->addDescriptor(new BLE2902());
    // pService->start();
    // BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    // pAdvertising->start();
}

void ScaleBLEService::updateWeight(float weight) {
    if (pCharacteristic) {
        pCharacteristic->setValue(weight);
        pCharacteristic->notify();
    }
}