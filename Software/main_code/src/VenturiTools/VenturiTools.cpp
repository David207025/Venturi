#include "VenturiTools.h"

#include <utility>


bool VenturiTools::deviceConnected = false;
volatile bool VenturiTools::s_touchFlags[16] = {false};
volatile bool VenturiTools::s_fingerOnPad[16] = {false};
TaskHandle_t VenturiTools::taskToNotify = NULL;
volatile TelemetryFrame *VenturiTools::buffer = nullptr;

IPAddress VenturiTools::peerIP = "";
uint16_t VenturiTools::peerPort = 0;
bool VenturiTools::peerKnown = false;
uint16_t VenturiTools::localPort = 5005;
WiFiUDP VenturiTools::udpSender;


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

VenturiTools::VenturiTools(String bleName, int debug_level, bool ble_enabled, String serviceUUID,
                           String characteristicUUID) {
    this->debugLevel = debug_level;
    Serial.println(this->debugLevel);

    initSPI();

    if (temperature_sensor_install(&tempSensorConfig, &tempHandle) == ESP_OK) {
        temperature_sensor_enable(tempHandle);
        temperatureSensorAvailable = true;
    } else {
        temperatureSensorAvailable = false;
        Serial.println("Failed to install temperature sensor!");
    }

    if (ble_enabled) {
        initBLE(std::move(bleName), std::move(serviceUUID), std::move(characteristicUUID));
    }

    initSD();

    if (debug_level > 1) {
        Serial.println(">>> Warm-up completed. Settling peripheral registers...");
    }
}

VenturiTools::~VenturiTools() {
}

void VenturiTools::setTaskNotificationHandle(TaskHandle_t taskHandle) {
    this->taskToNotify = taskHandle;
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

    pIPCharacteristic = pService->createCharacteristic(
        IP_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );

    pIPCharacteristic->setValue("0.0.0.0");

    pIPCharacteristic->setCallbacks(new PCIPCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->setScanResponse(true);

    // Tight connection windows for maximum responsiveness
    pAdvertising->setMinPreferred(0x06); // 7.5ms
    pAdvertising->setMaxPreferred(0x0C); // 15ms

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

void VenturiTools::initSPI() {
    spi_bus_config_t adcbuscfg = {};
    adcbuscfg.mosi_io_num = PIN_ADC_MOSI;
    adcbuscfg.miso_io_num = PIN_ADC_MISO;
    adcbuscfg.sclk_io_num = PIN_ADC_CLK;
    adcbuscfg.quadwp_io_num = GPIO_NUM_NC;
    adcbuscfg.quadhd_io_num = GPIO_NUM_NC;
    adcbuscfg.max_transfer_sz = 4096; // Allow a buffer size matching your transactions
    adcbuscfg.flags = SPICOMMON_BUSFLAG_MASTER;

    // Change from SPI_DMA_DISABLED to AUTO to allow hardware background transfers
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &adcbuscfg, SPI_DMA_CH_AUTO));

    gpio_pulldown_en((gpio_num_t)PIN_ADC_MISO);

    spi_device_interface_config_t adc_devcfg = {};
    adc_devcfg.mode = 0;
    adc_devcfg.clock_speed_hz = 20000000;
    adc_devcfg.spics_io_num = PIN_ADC_CS;
    adc_devcfg.queue_size = 5; // Increase queue depth for pipelining

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &adc_devcfg, &spi_adc));

    // --- Do the exact same change for SPI3 (IMU) below ---
    spi_bus_config_t imubuscfg = {};
    imubuscfg.mosi_io_num = PIN_IMU_MOSI;
    imubuscfg.miso_io_num = PIN_IMU_MISO;
    imubuscfg.sclk_io_num = PIN_IMU_CLK;
    imubuscfg.quadwp_io_num = GPIO_NUM_NC;
    imubuscfg.quadhd_io_num = GPIO_NUM_NC;
    imubuscfg.max_transfer_sz = 4096;
    imubuscfg.flags = SPICOMMON_BUSFLAG_MASTER;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &imubuscfg, SPI_DMA_CH_AUTO));

    gpio_pulldown_en((gpio_num_t)PIN_IMU_MISO);

    spi_device_interface_config_t imu_devcfg = {};
    imu_devcfg.mode = 0;
    imu_devcfg.clock_speed_hz = 20000000;
    imu_devcfg.spics_io_num = PIN_IMU_CS;
    imu_devcfg.queue_size = 5;

    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &imu_devcfg, &spi_imu));
}


void VenturiTools::initFastHardwarePipeline() {
    _spi_adc_hw = &GPSPI2;
    _spi_imu_hw = &GPSPI3;

    // 1. Core full-duplex configuration
    _spi_imu_hw->user.usr_miso = 1;
    _spi_imu_hw->user.usr_mosi = 1;
    _spi_adc_hw->user.usr_miso = 1;
    _spi_adc_hw->user.usr_mosi = 1;

    // 2. Clear out driver lingering timing configurations
    _spi_imu_hw->ctrl.val = 0;
    _spi_adc_hw->ctrl.val = 0;

    // ====================================================================
    // SINGLE-FRAME INITIALIZATION FOR ADS7961 (AUTO-2 MODE)
    // ====================================================================

    // Temporarily set ADC transaction bit length to 16 bits (0-indexed)
    _spi_adc_hw->ms_dlen.ms_data_bitlen = 16 - 1;

    // Send Auto-2 Command (0x93C0) to set max channel index to 15
    _spi_adc_hw->data_buf[0].val = ((uint32_t)0x93C0 << 16);
    _spi_adc_hw->cmd.update = 1;
    _spi_adc_hw->cmd.usr = 1;
    while (_spi_adc_hw->cmd.usr); // Wait for the configuration byte to clear

    // ====================================================================
    // SWITCH TO HIGH-SPEED RUNTIME PROFILES
    // ====================================================================

    // Reconfigure the hardware blocks to handle continuous loop reads
    _spi_imu_hw->ms_dlen.ms_data_bitlen = 104 - 1; // 13 Bytes for unified IMU burst
    _spi_adc_hw->ms_dlen.ms_data_bitlen = 256 - 1; // 32 Bytes for 16 sequential ADC reads

    // Commit changes to hardware parameters
    _spi_imu_hw->cmd.update = 1;
    _spi_adc_hw->cmd.update = 1;
}

void IRAM_ATTR VenturiTools::startSPIReads() {
    // 1. Manually toggle the update configuration bit.
    // This forces the hardware to apply your bit-length parameters.
    _spi_imu_hw->cmd.update = 1;
    _spi_adc_hw->cmd.update = 1;

    // 2. Flash the user execution bit. The physical clock pins start pulsing immediately.
    _spi_imu_hw->cmd.usr = 1;
    _spi_adc_hw->cmd.usr = 1;
}

void IRAM_ATTR VenturiTools::getSPIResults(uint16_t* adc_buf, uint8_t* imu_buf) {
    while (_spi_imu_hw->cmd.usr);
    while (_spi_adc_hw->cmd.usr);

    // 1. Parse IMU into raw bytes directly
    uint32_t imu0 = _spi_imu_hw->data_buf[0].val;
    uint32_t imu1 = _spi_imu_hw->data_buf[1].val;
    uint32_t imu2 = _spi_imu_hw->data_buf[2].val;
    uint32_t imu3 = _spi_imu_hw->data_buf[3].val;

    // --- Unpack Gyro (Bytes 1-6 of real data) ---
    // imu0 bits [31:24] is Byte 1 (Garbage echo). Skip it!
    imu_buf[0] = (imu0 >> 16) & 0xFF; // Byte 2  -> Gyro X High
    imu_buf[1] = (imu0 >> 8)  & 0xFF; // Byte 3  -> Gyro X Low
    imu_buf[2] = imu0 & 0xFF;         // Byte 4  -> Gyro Y High

    imu_buf[3] = (imu1 >> 24) & 0xFF; // Byte 5  -> Gyro Y Low
    imu_buf[4] = (imu1 >> 16) & 0xFF; // Byte 6  -> Gyro Z High
    imu_buf[5] = (imu1 >> 8)  & 0xFF; // Byte 7  -> Gyro Z Low

    // --- Unpack Accel (Bytes 7-12 of real data) ---
    imu_buf[6] = imu1 & 0xFF;         // Byte 8  -> Accel X High

    imu_buf[7] = (imu2 >> 24) & 0xFF; // Byte 9  -> Accel X Low
    imu_buf[8] = (imu2 >> 16) & 0xFF; // Byte 10 -> Accel Y High
    imu_buf[9] = (imu2 >> 8)  & 0xFF; // Byte 11 -> Accel Y Low
    imu_buf[10] = imu2 & 0xFF;        // Byte 12 -> Accel Z High

    imu_buf[11] = (imu3 >> 24) & 0xFF; // Byte 13 -> Accel Z Low

    // 2. Parse ADC and fix Endianness/Alignment on the fly
    // Instead of a blind cast, unpack each 32-bit register into two 16-bit channels
    for (int i = 0; i < 8; i++) {
        uint32_t raw_reg = _spi_adc_hw->data_buf[i].val;

        // Extract high 16 bits and low 16 bits, swapping bytes to fix Little-Endian CPU conversion
        uint16_t word_high = (uint16_t)(raw_reg >> 16);
        uint16_t word_low  = (uint16_t)(raw_reg & 0xFFFF);

        adc_buf[i * 2]     = __builtin_bswap16(word_high);
        adc_buf[(i * 2) + 1] = __builtin_bswap16(word_low);
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

            IPAddress gateway = WiFi.gatewayIP();
            IPAddress subnet = WiFi.subnetMask();
            IPAddress dns = WiFi.dnsIP();
            IPAddress local_IP = WiFi.localIP();

            for (int i = 0; i < 4; i++) {
                if (subnet[i] == 0) {
                    local_IP[i] = 200;
                }
            }

            WiFi.disconnect();

            delay(500);

            WiFi.config(local_IP, gateway, subnet, dns);
            WiFi.begin(matches[i].ssid.c_str(), matches[i].password.c_str());

            Serial.println("\n>>> Connecting to network with static IP-Adress: ");
            Serial.printf(" IP: %s \n", local_IP.toString().c_str());
            Serial.printf(" Netmask: %s \n", subnet.toString().c_str());

            uint8_t staticAttempts = 0;
            while (WiFi.status() != WL_CONNECTED && staticAttempts < 20) {
                delay(500);
                if (debugLevel > 1)
                    Serial.print(".");
                staticAttempts++;
            }

            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("\n>>> Static IP re-connection failed, reverting to DHCP...");
                // Fallback: Clear config and connect without static IP
                WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
                WiFi.begin(matches[i].ssid.c_str(), matches[i].password.c_str());
            }


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

    delay(500);

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


void VenturiTools::enableOtaListening() {
    // 1. Create the server
    otaServer = new AsyncWebServer(80);

    // 2. IMPORTANT: Force the server to use the system's internal event loop
    // This prevents your motor loops from blocking the TCP stack.
    otaServer->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Venturi Engine Online");
    });

    ElegantOTA.begin(otaServer);

    // 3. Start it
    otaServer->begin();
    Serial.println(">>> AsyncWebServer is now LIVE.");
}

void VenturiTools::beginUDP() {
    udpSender.begin(localPort);
    Serial.printf(">>> UDP Listener active on port %d\n", localPort);
    delay(200);
}

bool VenturiTools::streamUDP(TelemetryFrame frame) {
    if (!peerKnown) return false;
    if (udpSender.beginPacket(peerIP, peerPort) == 0) return false;
    udpSender.write((uint8_t *) &frame, sizeof(frame));

    // endPacket() returns 1 on success, 0 on failure
    int result = udpSender.endPacket();

    if (result == 0) {
        // Log the failure or track it
        return false;
    }
    return true;
}

void VenturiTools::updateBLEIP(String ip) {
    if (pIPCharacteristic != nullptr) {
        pIPCharacteristic->setValue(ip.c_str());
        if (debugLevel > 1)
            Serial.println(">>> BLE IP Characteristic updated to: " + ip);
    }
}

float VenturiTools::getTemperature() {
    float temperature = -1.0f; // Force distinct float assignment
    if (temperature_sensor_get_celsius(tempHandle, &temperature) == ESP_OK) {
        return temperature;
    }
    return -1.0f; // Return an obvious error state if the driver trips
}
