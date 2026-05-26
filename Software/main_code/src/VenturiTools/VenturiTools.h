//
// Created by David Vacaroiu on 19.05.26.
//

#ifndef UNTITLED3_VENTURITOOLS_H
#define UNTITLED3_VENTURITOOLS_H
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <FS.h>
#include <SD_MMC.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h> // Make sure to install this via the Library Manager
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // For safe cross-thread communication

#define DEFAULT_SERVICE_UUID        "4faac06d-5cb0-4b08-a2d3-1d26c3597074"
#define DEFAULT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define BOOT_BUTTON_PIN 35

#define SDMMC_PIN_CMD  44
#define SDMMC_PIN_CLK  43
#define SDMMC_PIN_D0   39
#define SDMMC_PIN_D1   40
#define SDMMC_PIN_D2   41
#define SDMMC_PIN_D3   42

struct MatchedNetwork {
    String ssid;
    String password;
    int rssi;
};

class VenturiTools {
    BLECharacteristic *pCharacteristic = nullptr;
    static bool deviceConnected;
    int debugLevel = 0;

    // A global static ISR wrapper that the Arduino API can accept
    static void IRAM_ATTR globalTouchISR();

    // Arrays to hold the volatile flags for the 16 possible channels
    static volatile bool s_touchFlags[16];
    static volatile bool s_fingerOnPad[16];

    class ServerCallbacks: public BLEServerCallbacks {
    public:
        void onConnect(BLEServer* pServer) override {
            VenturiTools::deviceConnected = true;

            // Dynamic Negotiation: scale up the pipeline ONLY after the connection is stable
            delay(10);
            BLEDevice::setMTU(517);
        }

        void onDisconnect(BLEServer* pServer) override {
            VenturiTools::deviceConnected = false;

            BLEDevice::startAdvertising(); // Kick off advertising again automatically
        }
    };

    void initBLE(String name, String serviceUUID, String characteristicUUID);
    void initSD();


public:
    VenturiTools(String bleName, int debug_level = 0, String serviceUUID = DEFAULT_SERVICE_UUID, String characteristicUUID = DEFAULT_CHARACTERISTIC_UUID);
    ~VenturiTools();

    bool validBLEConnection();

    template <typename T>
    void writeBLE(T message) {
        if (!validBLEConnection()) return;

        // 1. Force a cast of the object instance address to a raw byte pointer
        const uint8_t* bytePointer = reinterpret_cast<const uint8_t*>(&message);

        // 2. Extract the exact memory footprint size of the structure
        size_t dataSize = sizeof(T);

        // 3. Use the 2-argument BLE method to transmit the raw block over the air
        pCharacteristic->setValue(bytePointer, dataSize);
        pCharacteristic->notify();
    }

    void initTouchChannelInterrupt(uint8_t channel, uint16_t threshold);

    File openFile(String filename, const char* mode);
    void reloadSD();

    void initTouchChannel(uint8_t channel);
    bool isChannelTouched(uint8_t channel);

    String scanBestAvailableNetwork(JsonDocument& allowedNetworks);
    void autoConnectWiFi(const char* jsonPath = "/network.json");

};


#endif
