#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ArduinoOTA.h> // Switched to native OTA

#define DEFAULT_SERVICE_UUID        "4faac06d-5cb0-4b08-a2d3-1d26c3597074"
#define DEFAULT_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define IP_CHARACTERISTIC_UUID      "822c9530-9548-4375-8422-909247192312"

#define PIN_UDPR 23
#define PIN_TM 22
HardwareSerial P4_Link(1);

struct __attribute__((packed)) TelemetryFrame {
    uint32_t timestamp;
    int32_t steering;
    float temperature_c;
    int16_t gyro[3];
    int16_t accel[3];
    uint16_t battery_mv;
    uint16_t update_speed;
    uint8_t adc_data[16];
};

struct MatchedNetwork {
    String ssid;
    String password;
    int rssi;
};

bool deviceConnected = false;
WiFiUDP udpSender;
IPAddress peerIP;
uint16_t peerPort;
bool peerKnown = false;
uint16_t localPort = 5005;

BLECharacteristic *pCharacteristic = nullptr;
BLECharacteristic *pIPCharacteristic = nullptr;

// --- Helper Functions ---

void beginUDP() {
    udpSender.begin(localPort);
    Serial.printf(">>> UDP Listener active on port %d\n", localPort);
}

void setupOTA() {
    ArduinoOTA.setHostname("venturi");
    ArduinoOTA.begin();
    Serial.println(">>> Native ArduinoOTA ready.");
}

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override { deviceConnected = true; BLEDevice::setMTU(517); }
    void onDisconnect(BLEServer *pServer) override { deviceConnected = false; BLEDevice::startAdvertising(); }
};

class PCIPCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String value = pChar->getValue();
        if (value.length() > 0) {
            peerIP.fromString(value.c_str());
            peerPort = 5005;
            peerKnown = true;
            Serial.println(">>> PC IP received: " + String(value.c_str()));
            beginUDP();
        }
    }
};

void autoConnectWiFi(String json) {
    JsonDocument doc;
    if (deserializeJson(doc, json)) return;

    WiFi.mode(WIFI_STA);
    int found = WiFi.scanNetworks();

    // Simple connection logic (Omitted sorting for brevity, add back if needed)
    for (int i = 0; i < found; i++) {
        String ssid = WiFi.SSID(i);
        if (doc.containsKey(ssid)) {
            WiFi.begin(ssid.c_str(), doc[ssid].as<String>().c_str());
            if (WiFi.waitForConnectResult() == WL_CONNECTED) {
                Serial.println(">>> Connected to " + ssid);
                MDNS.begin("venturi");
                return;
            }
        }
    }
    WiFi.scanDelete();
}

void initBLE(String name) {
    BLEDevice::init(name);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    BLEService *pService = pServer->createService(DEFAULT_SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(DEFAULT_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pIPCharacteristic = pService->createCharacteristic(IP_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
    pIPCharacteristic->setCallbacks(new PCIPCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(DEFAULT_SERVICE_UUID);
    BLEDevice::startAdvertising();
}

// --- Main Flow ---

void setup() {
    pinMode(PIN_UDPR, OUTPUT);
    pinMode(PIN_TM, OUTPUT);
    digitalWrite(PIN_UDPR, LOW); // Signal readiness

    Serial.begin(115200);
    P4_Link.begin(921600, SERIAL_8N1, 18, 19);

    // Handshake logic
    String incomingJson = "";
    unsigned long start = millis();
    while (millis() - start < 3000) {
        if (P4_Link.available() && P4_Link.peek() == '{') {
            incomingJson = P4_Link.readStringUntil('}');
            incomingJson += "}";
            break;
        }
    }

    if (incomingJson.length() > 5) autoConnectWiFi(incomingJson);

    initBLE("Venturi");
    setupOTA();

    digitalWrite(PIN_UDPR, LOW);
    digitalWrite(PIN_TM, LOW);
    Serial.println(">>> System Ready.");
}

void loop() {
    ArduinoOTA.handle();

    static uint8_t frameBuffer[sizeof(TelemetryFrame)];
    static int bytesRead = 0;
    static int state = 0; // 0: Wait AA, 1: Wait 00, 2: Wait AA, 3: Read Data

    if (P4_Link.available()) {
        digitalWrite(PIN_TM, HIGH); // Signal Busy

        while (P4_Link.available()) {
            uint8_t byte = P4_Link.read();

            switch (state) {
                case 0: if (byte == 0xAA) state = 1; break;
                case 1: if (byte == 0x00) state = 2; else state = 0; break;
                case 2:
                    if (byte == 0xAA) {
                        state = 3;
                        bytesRead = 0;
                    } else {
                        state = (byte == 0xAA) ? 1 : 0;
                    }
                    break;
                case 3:
                    frameBuffer[bytesRead++] = byte;
                    if (bytesRead >= sizeof(TelemetryFrame)) {
                        // Frame complete, process it
                        TelemetryFrame rxFrame;
                        memcpy(&rxFrame, frameBuffer, sizeof(TelemetryFrame));

                        if (peerKnown) {
                            udpSender.beginPacket(peerIP, peerPort);
                            udpSender.write((uint8_t *)&rxFrame, sizeof(TelemetryFrame));
                            udpSender.endPacket();
                        }
                        state = 0; // Reset for next header
                    }
                    break;
            }
        }
        digitalWrite(PIN_TM, LOW); // Signal Ready
    }
}