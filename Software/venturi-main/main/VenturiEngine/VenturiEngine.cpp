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

    xTaskCreatePinnedToCore(VenturiEngine::vCore1TaskWrapper, "Engine_Core1", 8192, this, 9, NULL, 1);
    xTaskCreatePinnedToCore(VenturiEngine::vCore0TaskWrapper, "Engine_Core0", 8192, this, 10, NULL, 0);
}

void VenturiEngine::vCore0TaskWrapper(void *pvParameters) {
    VenturiEngine *instance = static_cast<VenturiEngine *>(pvParameters);
    printf("Task Wrapper running for core0 at: %p\n", instance);
    instance->core0_loop();
    vTaskDelete(NULL);
}

void VenturiEngine::vCore1TaskWrapper(void *pvParameters) {
    VenturiEngine *instance = static_cast<VenturiEngine *>(pvParameters);
    printf("Task Wrapper running for core1 at: %p\n", instance);
    instance->core1_loop();
    vTaskDelete(NULL);
}