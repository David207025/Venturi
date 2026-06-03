#include "VenturiEngine.h"

#include <esp_task_wdt.h>

volatile bool VenturiEngine::frameReady = false;

VenturiEngine* VenturiEngine::_instance = nullptr;
TaskHandle_t VenturiEngine::_core0TaskHandle = NULL;
TaskHandle_t VenturiEngine::_core1TaskHandle = NULL;

VenturiEngine::VenturiEngine() {
    _instance = this; // Bind the current instance
}

VenturiEngine::~VenturiEngine() {
}

void VenturiEngine::start() {

    // You should fix this mapping:
    xTaskCreatePinnedToCore(VenturiEngine::vCore0TaskWrapper, "Engine_Core0", 16348, this, 10, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(VenturiEngine::vCore1TaskWrapper, "Engine_Core1", 16348, this, 24,  NULL, 1); // Core 1
}

void VenturiEngine::vCore0TaskWrapper(void *pvParameters) {
    VenturiEngine *instance = static_cast<VenturiEngine *>(pvParameters);
    instance->core0_loop();
    vTaskDelete(NULL); // Safeguard if loop ever breaks
}

void VenturiEngine::vCore1TaskWrapper(void *pvParameters) {
    VenturiEngine *instance = static_cast<VenturiEngine *>(pvParameters);
    instance->core1_loop();
    vTaskDelete(NULL); // Safeguard if loop ever breaks
}