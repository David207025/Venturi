#include "VenturiTools.h"

bool VenturiTools::deviceConnected = false;
volatile bool VenturiTools::s_touchFlags[16] = {false};
volatile bool VenturiTools::s_fingerOnPad[16] = {false};

// 2. The master hardware interrupt handler
void IRAM_ATTR VenturiTools::globalTouchISR() {
    // Check all 14 active touch channels mapped via GPIO 2 to 15
    for (int pin = 2; pin <= 14; pin++) {
        uint8_t arrayIdx = pin - 2;

        // Use the official Arduino-ESP32 API to check if this specific pin caused the ISR trip
        if (touchInterruptGetLastStatus(pin)) {
            if (!s_fingerOnPad[arrayIdx]) {
                s_touchFlags[arrayIdx] = true;
                s_fingerOnPad[arrayIdx] = true; // Edge-trigger lock to prevent ISR bouncing
            }
        }
    }
}

VenturiTools::VenturiTools(String bleName, int debug_level, String serviceUUID, String characteristicUUID) {
    this->debugLevel = debug_level;
    Serial.println(this->debugLevel);
    initBLE(bleName, serviceUUID, characteristicUUID);
    initSD();

    if (debug_level > 1) {
        Serial.println(">>> Warm-up completed. Settling peripheral registers...");
    }
}

VenturiTools::~VenturiTools() {
}

void VenturiTools::initBLE(String name, String serviceUUID, String characteristicUUID) {
    BLEDevice::init(name);

    BLEServer *pServer = BLEDevice::createServer();
    // Swap your old ServerCallbacks with our dynamic one that tracks debug level
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(serviceUUID);

    pCharacteristic = pService->createCharacteristic(
        characteristicUUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->setScanResponse(true);

    // Tight connection windows for maximum responsiveness
    pAdvertising->setMinPreferred(0x06);  // 7.5ms
    pAdvertising->setMaxPreferred(0x0C);  // 15ms

    BLEDevice::startAdvertising();

    if (debugLevel > 1) {
        Serial.println(">>> BLE Advertising active and safe with name: " + name);
    }
}

void VenturiTools::initSD() {
    if (!SD_MMC.setPins(SDMMC_PIN_CLK, SDMMC_PIN_CMD, SDMMC_PIN_D0, SDMMC_PIN_D1, SDMMC_PIN_D2, SDMMC_PIN_D3)) {
        if (debugLevel > 1) {
            Serial.println(">>> SD-Reader Pin configuration failed!");
        }
        return;
    }

    if (!SD_MMC.begin("/sdcard", false, false, 20000)) {
        if (debugLevel > 1) {
            Serial.println(">>> SD-Card Mount Failed! Is the card inserted and formatted to FAT32?");
        }
        return;
    }

    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        if (debugLevel > 1) {
            Serial.println(">>> No SD card attached.");
        }
        return;
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);

    if (debugLevel > 1) {
        Serial.print(">>> SD_MMC Card Type: ");
        if (cardType == CARD_MMC)
            Serial.println("MMC");
        else if (cardType == CARD_SD)
            Serial.println("SDSC");
        else if (cardType == CARD_SDHC)
            Serial.println("SDHC/SDXC");
        else
            Serial.println("UNKNOWN");
        Serial.printf(">>> Card size: %llu MB\n", cardSize);
    }
}

void VenturiTools::initTouchChannel(uint8_t pin) {
    if (pin < 2 || pin > 14) {
        if (debugLevel > 1) {
            Serial.println(">>> Touch Channel initialization failed! No touch channel on selected pin: " + String(pin));
            return;
        }
    }

    if (debugLevel > 1) {
        Serial.println(">>> Initializing Touch Channel on pin: " + String(pin));
    }

    for (int i = 0; i < 4; i++) {
        uint16_t result = touchRead(pin);
        if (debugLevel > 1) {
            Serial.println(">>> Touch Channel initialized: " + String(result));
        }
        if (result < 65535) {
            break;
        }
    }
}

// 3. Your modified initialization method with index offset normalization
void VenturiTools::initTouchChannelInterrupt(uint8_t pin, uint16_t threshold) {
    if (pin < 2 || pin > 14) return; // Boundary safeguard

    uint8_t arrayIdx = pin - 2;

    s_touchFlags[arrayIdx] = false;
    s_fingerOnPad[arrayIdx] = false;

    // Pass the physical GPIO pin directly to the Arduino wrapper API
    touchAttachInterrupt(pin, globalTouchISR, threshold);
}

// 4. A clean helper method to read and reset the triggers in your main loop
bool VenturiTools::isChannelTouched(uint8_t pin) {
    if (pin < 2 || pin > 15) return false;

    uint8_t arrayIdx = pin - 2;

    if (s_touchFlags[arrayIdx]) {
        s_touchFlags[arrayIdx] = false; // Clear the edge-trigger latch
        s_fingerOnPad[arrayIdx] = false; // Reset the lock so the next hardware touch can fire the ISR
        return true;
    }
    return false;
}

bool VenturiTools::validBLEConnection() {
    return deviceConnected && pCharacteristic != nullptr;
}

File VenturiTools::openFile(String filename, const char *mode) {
    return SD_MMC.open(filename, mode);
}

void VenturiTools::autoConnectWiFi(const char *jsonPath) {
    if (debugLevel > 1) {
        Serial.println(">>> Initializing Wi-Fi Auto-Connect via SD Card...");
    }

    File configFile = SD_MMC.open(jsonPath, "r");
    if (!configFile) {
        Serial.println(">>> ERROR: Could not open network.json on SD Card!");
        return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();

    if (error) {
        Serial.print(">>> ERROR: Failed to parse network.json: ");
        Serial.println(error.c_str());
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (debugLevel > 1) {
        Serial.println(">>> Scanning local airwaves for broadcasted SSIDs...");
    }

    int foundNetworksCount = WiFi.scanNetworks();
    if (foundNetworksCount <= 0) {
        Serial.println(">>> ERROR: No Wi-Fi networks detected in environment hardware scan.");
        WiFi.scanDelete();
        return;
    }

    // Filter networks from the scan that exist inside your network.json file
    MatchedNetwork matches[32];
    int matchCount = 0;

    for (int i = 0; i < foundNetworksCount && matchCount < 32; i++) {
        String scanSSID = WiFi.SSID(i);
        if (debugLevel > 1) {
            Serial.printf("    Detected SSID: %s (%d dBm)\n", scanSSID.c_str(), WiFi.RSSI(i));
        }
        if (doc.containsKey(scanSSID)) {
            matches[matchCount].ssid = scanSSID;
            matches[matchCount].password = doc[scanSSID].as<String>();
            matches[matchCount].rssi = WiFi.RSSI(i);
            matchCount++;
        }
    }
    WiFi.scanDelete(); // Free memory immediately

    if (matchCount == 0) {
        Serial.println(">>> ERROR: No matching networks from network.json were detected nearby.");
        return;
    }

    // Selection Sort: Order matches array descending by signal strength (RSSI)
    for (int i = 0; i < matchCount - 1; i++) {
        int maxIdx = i;
        for (int j = i + 1; j < matchCount; j++) {
            if (matches[j].rssi > matches[maxIdx].rssi) {
                maxIdx = j;
            }
        }
        if (maxIdx != i) {
            MatchedNetwork temp = matches[i];
            matches[i] = matches[maxIdx];
            matches[maxIdx] = temp;
        }
    }

    // Try connecting to networks one by one, starting from the strongest
    bool connectionSuccessful = false;
    for (int i = 0; i < matchCount; i++) {
        if (debugLevel > 1) {
            Serial.printf(">>> Attempting connection to network profile [%d/%d]: %s (%d dBm)\n",
                          i + 1, matchCount, matches[i].ssid.c_str(), matches[i].rssi);
        }

        WiFi.begin(matches[i].ssid.c_str(), matches[i].password.c_str());

        uint8_t connectionAttempts = 0;
        while (WiFi.status() != WL_CONNECTED && connectionAttempts < 25) {
            // ~12.5 seconds timeout per profile
            delay(500);
            if (debugLevel > 1)
                Serial.print(".");
            connectionAttempts++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            connectionSuccessful = true;
            Serial.println("\n>>> Wi-Fi Connection Established Successfully!");
            Serial.printf("    SSID: %s\n", WiFi.SSID().c_str());
            Serial.printf("    IP Address: %s\n", WiFi.localIP().toString().c_str());

            if (!MDNS.begin("venturi")) {
                // Sets the hostname to "venturi.local"
                Serial.println("Error setting up MDNS responder!");
            } else {
                Serial.println(">>> mDNS responder started: http://venturi.local");
                // Advertise that this device is running an HTTP server over TCP on port 80
                MDNS.addService("http", "tcp", 80);
            }

            if (debugLevel > 1) {
                Serial.printf("    Signal Strength (RSSI): %d dBm\n", WiFi.RSSI());
            }
            break; // Exit connection loop since we are online
        } else {
            if (debugLevel > 1) {
                Serial.printf("\n>>> Profile connection failed for %s. Dropping to next available profile.\n",
                              matches[i].ssid.c_str());
            }
            WiFi.disconnect();
            delay(100);
        }
    }

    if (!connectionSuccessful) {
        Serial.println(">>> ERROR: Failed to connect to any matched profiles. Check passwords or proximity.");
        return;
    }

    // Fire up the local mDNS broadcast handler
    if (MDNS.begin("venturi")) {
        if (debugLevel > 1) {
            Serial.println(">>> mDNS Responder configuration ready: http://venturi.local/");
        }
    } else {
        Serial.println(">>> ERROR: Setting up mDNS responder failed.");
    }
}

String VenturiTools::scanBestAvailableNetwork(JsonDocument &allowedNetworks) {
    if (debugLevel > 1) {
        Serial.println(">>> Scanning local airwaves for broadcasted SSIDs...");
    }

    // Scan networks synchronously (returns number of networks found)
    int foundNetworksCount = WiFi.scanNetworks();
    if (foundNetworksCount == 0) {
        if (debugLevel > 1) {
            Serial.println(">>> No networks found in range.");
        }
        return "";
    }

    String bestSSID = "";
    int bestRSSI = -100; // Lower numbers mean weaker signals

    for (int i = 0; i < foundNetworksCount; i++) {
        String currentSSID = WiFi.SSID(i);
        int currentRSSI = WiFi.RSSI(i);

        // Check if this specific broadcasted SSID exists inside our JSON configuration dictionary
        if (allowedNetworks.containsKey(currentSSID)) {
            if (debugLevel > 1) {
                Serial.printf("    Match Found: %s (%d dBm)\n", currentSSID.c_str(), currentRSSI);
            }

            // Elect the network with the highest relative signal strength
            if (currentRSSI > bestRSSI) {
                bestRSSI = currentRSSI;
                bestSSID = currentSSID;
            }
        }
    }

    // Clean up memory allocated during the scan process
    WiFi.scanDelete();
    return bestSSID;
}

void VenturiTools::reloadSD() {
    if (debugLevel > 1) {
        Serial.println(">>> Reloading SD Card File System...");
    }

    SD_MMC.end(); // Unmount the current SD card
    delay(100); // Short delay to ensure unmount completes
    initSD(); // Re-initialize the SD card interface

    if (debugLevel > 1) {
        Serial.println(">>> SD Card reloaded successfully.");
    }
}