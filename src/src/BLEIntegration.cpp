// #define LOG_BLE_COMMANDS

#include "BLEIntegration.h"
#include <Arduino.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include "TimerCommands.h"
#include <string>
#include <vector>

static BLECharacteristic* weightCharacteristic = nullptr;
static BLEServer* gServer = nullptr;
static BLECharacteristic* writeCharacteristic = nullptr;


// Command callbacks
static std::function<void()> tareCb = nullptr;
static std::function<void(uint8_t)> timerCb = nullptr;
static std::function<void(bool)> lcdCb = nullptr;
static std::function<void()> poweroffCb = nullptr;
static std::function<void()> calibrateCb = nullptr;
static std::function<void(float)> factorCb = nullptr;
// connection callbacks
static std::function<void()> connectedCb = nullptr;
static std::function<void()> disconnectedCb = nullptr;


// Helper: build Decent-format weight message
static std::vector<unsigned char> buildWeightMessage(int gramsTimes10, int minutes, int seconds, int milliseconds, bool isStable) {
    uint8_t status = isStable ? 0xCE : 0xCA;
    std::vector<unsigned char> message;
    message.push_back(0x03); // DecentHeader
    message.push_back(status); // WEIGHT or WEIGHT_CHANGE
    message.push_back((gramsTimes10 >> 8) & 0xFF);
    message.push_back(gramsTimes10 & 0xFF);
    message.push_back(minutes & 0xFF);
    message.push_back(seconds & 0xFF);
    message.push_back(milliseconds & 0xFF);
    message.push_back(0x00);
    message.push_back(0x00);

    unsigned char xor_result = 0;
    for (uint8_t idx = 0; idx < 4; idx++) {
        xor_result ^= message[idx];
    }
    message.push_back(xor_result);
    return message;
}

// Helper: parse and print a Decent command bytes vector according to the spec provided.
static void parseAndPrintCommand(const std::vector<uint8_t> &b) {
    // Bytes: [0]=Model(0x03), [1]=Type, [2]=Cmd/Data1, [3]=Cmd/Data2, [4]=Cmd/Data3, [5]=Cmd/Data4, [6]=???, [7]=XOR
    // The provided spec indicates XOR is over first 6 bytes (indices 0..5) and final byte is validation.
    uint8_t model = b[0];
    uint8_t type = b[1];
    uint8_t d1 = b[2];
    uint8_t d2 = b[3];
    uint8_t d3 = b[4];
    uint8_t d4 = b[5];
    uint8_t xorByte = b[6];

    // Compute XOR over first 6 bytes
    uint8_t xorCalc = 0;
    for (int i = 0; i < 6; ++i) xorCalc ^= b[i];

    Serial.println("--- Decent Command Parsed ---");
    Serial.printf("Model: 0x%02X\n", model);
    Serial.printf("Type:  0x%02X ", type);
    switch (type) {
        case 0x0A: Serial.println("(LED on/off / power)"); break;
        case 0x0B: Serial.println("(Timer)"); break;
        case 0x0F: Serial.println("(Tare)"); break;
        default: Serial.println("(Unknown)"); break;
    }
    Serial.printf("Data1: 0x%02X\n", d1);
    Serial.printf("Data2: 0x%02X\n", d2);
    Serial.printf("Data3: 0x%02X\n", d3);
    Serial.printf("Data4: 0x%02X\n", d4);
    Serial.printf("XOR (recv): 0x%02X  XOR(calc over first6): 0x%02X => %s\n", xorByte, xorCalc, (xorByte==xorCalc)?"OK":"MISMATCH");

    // Interpret some common commands per table
    if (model == 0x03) {
        if (type == 0x0A) {
            return;
            // LED / power
            if (d1 == 0x01 && d2 == 0x01) {
                Serial.println("Command: LED ON");
            } else if (d1 == 0x00 && d2 == 0x00) {
                Serial.println("Command: LED OFF");
            } else if (d1 == 0x02) {
                Serial.println("Command: Power Off");
            } else {
                Serial.println("Command: LED/Power (unknown subcode)");
            }
        } else if (type == 0x0B) {
            // Timer
            if (d1 == 0x03) Serial.println("Command: Timer Start");
            else if (d1 == 0x00) Serial.println("Command: Timer Stop");
            else if (d1 == 0x02) Serial.println("Command: Timer Zero");
            else Serial.println("Command: Timer (unknown)");
        } else if (type == 0x0F) {
            Serial.println("Command: Tare");
        }
    }
    Serial.println("-----------------------------");
}

// Process a command byte vector: validate, optionally print, and invoke callbacks.
static void processCommand(const std::vector<uint8_t> &bytes) {
    if (bytes.size() < 2) return;
#ifdef LOG_BLE_COMMANDS
    parseAndPrintCommand(bytes);
#endif
    if (bytes[0] != 0x03) return; // not Decent header
    uint8_t cmd = bytes[1];
    if (cmd == 0x0F) { if (tareCb) tareCb(); }
    else if (cmd == 0x0B) {
        uint8_t action = 0x00;
        if (bytes.size() >= 3) action = bytes[2];
        if (timerCb) timerCb((uint8_t)mapDecentTimerAction(action));
    }
    else if (cmd == 0x0A) {
        uint8_t sub = 0;
        if (bytes.size() >= 3) sub = bytes[2];
        if (sub == 0x02) { if (poweroffCb) poweroffCb(); }
        else if (sub == 0x01) { if (lcdCb) lcdCb(true); }
        else { if (lcdCb) lcdCb(false); }
    }
}



class IntegrationCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        std::string val = pChar->getValue();
        if (val.size() < 1) return;

        const char *data = val.c_str();
        Serial.print("BLE Write: ");

        std::vector<uint8_t> bytes((const uint8_t*)val.data(), (const uint8_t*)val.data() + val.size());
        #ifdef LOG_BLE_COMMANDS
        parseAndPrintCommand(bytes);
        #endif
        processCommand(bytes);
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        Serial.println("BLE device connected");
        if (connectedCb) connectedCb();
    }
    void onDisconnect(BLEServer* pServer) override {
        Serial.println("BLE device disconnected");
        if (disconnectedCb) disconnectedCb();
    }
};

void bleInit(const char* deviceName, const char* serviceUUID, const char* characteristicUUID) {
    BLEDevice::init(deviceName);
    BLEServer* pServer = BLEDevice::createServer();
    gServer = pServer;
    pServer->setCallbacks(new ServerCallbacks());
    BLEService* pService = pServer->createService(serviceUUID);
    weightCharacteristic = pService->createCharacteristic(
        characteristicUUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
    );
    weightCharacteristic->addDescriptor(new BLE2902());
    weightCharacteristic->setCallbacks(new IntegrationCallbacks());
    // Create a separate writable characteristic (UUID 36F5) to accept external writes
    writeCharacteristic = pService->createCharacteristic(
        "36F5",
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    writeCharacteristic->setCallbacks(new IntegrationCallbacks());
    pService->start();
    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
}

// Callback setters
void setTareCallback(std::function<void()> cb) { tareCb = cb; }
void setTimerCallback(std::function<void(uint8_t)> cb) { timerCb = cb; }
void setLCDCallback(std::function<void(bool)> cb) { lcdCb = cb; }
void setPoweroffCallback(std::function<void()> cb) { poweroffCb = cb; }
void setCalibrateCallback(std::function<void()> cb) { calibrateCb = cb; }
void setFactorCallback(std::function<void(float)> cb) { factorCb = cb; }
void setConnectedCallback(std::function<void()> cb) { connectedCb = cb; }
void setDisconnectedCallback(std::function<void()> cb) { disconnectedCb = cb; }


void bleNotifyWeight(uint16_t weight, bool isStable, uint64_t elapsedMicros) {
    if (!weightCharacteristic) return;
    int gramsTimes10 = (int)weight * 10;
    uint8_t high = (gramsTimes10 >> 8) & 0xFF;
    uint8_t low = gramsTimes10 & 0xFF;
    uint8_t status = isStable ? 0xCE : 0xCA;

    status = 0xCE; // always stable for now

    uint32_t ms = (uint32_t)(elapsedMicros / 1000ULL);
    int minutes = ms / 1000 / 60;
    int seconds = ms / 1000 - (minutes * 60);
    int milliseconds = ms - (seconds * 1000) - (minutes * 60 * 1000);

    auto message = buildWeightMessage(gramsTimes10, minutes, seconds, milliseconds, true);
    weightCharacteristic->setValue(message.data(), (size_t)message.size());
    // Only notify if there's an active connection
    if (gServer && gServer->getConnectedCount() > 0) {
        weightCharacteristic->notify();
    }
}
