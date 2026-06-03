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

#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1

#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <driver/temperature_sensor.h>
#include <soc/spi_struct.h>

#include "freertos/FreeRTOS.h"
#include "VenturiEngine/VenturiEngine.h"

#define DEFAULT_SERVICE_UUID        "4faac06d-5cb0-4b08-a2d3-1d26c3597074"
#define DEFAULT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define IP_CHARACTERISTIC_UUID      "822c9530-9548-4375-8422-909247192312"

#define BOOT_BUTTON_PIN 35

#define SDMMC_PIN_CMD  44
#define SDMMC_PIN_CLK  43
#define SDMMC_PIN_D0   39
#define SDMMC_PIN_D1   40
#define SDMMC_PIN_D2   41
#define SDMMC_PIN_D3   42


#define PIN_ADC_CS   2
#define PIN_ADC_CLK  3
#define PIN_ADC_MOSI 4
#define PIN_ADC_MISO 5

#define PIN_IMU_CS   23
#define PIN_IMU_CLK  22
#define PIN_IMU_MOSI 21
#define PIN_IMU_MISO 20
#define PIN_IMU_INT  51


struct MatchedNetwork {
    String ssid;
    String password;
    int rssi;
};

class VenturiTools {
    BLECharacteristic *pCharacteristic = nullptr;
    BLECharacteristic *pIPCharacteristic = nullptr;
    static bool deviceConnected;
    int debugLevel = 0;
    AsyncWebServer* otaServer;
    bool otaEnabled = false;
    temperature_sensor_handle_t tempHandle = NULL;
    temperature_sensor_config_t tempSensorConfig = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    bool temperatureSensorAvailable = false;
    static TaskHandle_t taskToNotify;


    spi_device_handle_t spi_imu;
    spi_device_handle_t spi_adc;



    static WiFiUDP udpSender;    // Used to send data to the learned peer
    static IPAddress peerIP;
    static uint16_t peerPort;
    static bool peerKnown;
    static uint16_t localPort; // Port the ESP listens on for commands


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

    class PCIPCallbacks: public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic *pChar) override {
            String value = pChar->getValue();
            if (value.length() > 0) {
                peerIP.fromString(value.c_str());
                peerPort = 5005;
                peerKnown = true;
                Serial.println(">>> PC IP received via BLE: " + String(value.c_str()));
                beginUDP();
            }
        }
    };


    void initSD();

public:
    static volatile TelemetryFrame* buffer;
    spi_transaction_t _imu_trans;
    spi_transaction_t _adc_trans;

    // Fast-access pointer references for hardware registers
    spi_dev_t* _spi_imu_hw;
    spi_dev_t* _spi_adc_hw;

    // Allocated internal safe DMA destinations
    uint8_t* _hw_imu_buffer;
    uint16_t* _hw_adc_buffer;


    VenturiTools(String bleName, int debug_level = 0, bool ble_enabled = false, String serviceUUID = DEFAULT_SERVICE_UUID, String characteristicUUID = DEFAULT_CHARACTERISTIC_UUID);
    ~VenturiTools();

    void initBLE(String name, String serviceUUID, String characteristicUUID);
    bool validBLEConnection();
    void initSPI();
    void initFastHardwarePipeline();
    void startSPIReads();
    void getSPIResults(uint16_t* adc_buf, uint8_t* imu_buf);

    void setTaskNotificationHandle(TaskHandle_t taskHandle);

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

    void updateBLEIP(String ip);

    void initTouchChannelInterrupt(uint8_t channel, uint16_t threshold);

    File openFile(String filename, const char* mode);
    void reloadSD();

    void initTouchChannel(uint8_t channel);
    bool isChannelTouched(uint8_t channel);

    String scanBestAvailableNetwork(JsonDocument& allowedNetworks);
    void autoConnectWiFi(const char* jsonPath = "/network.json");

    void enableOtaListening();

    static void beginUDP();

    bool streamUDP(TelemetryFrame frame);

    float getTemperature();

};


#endif