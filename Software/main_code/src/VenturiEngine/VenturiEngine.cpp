#include "VenturiEngine.h"

VenturiEngine::VenturiEngine() {
}

VenturiEngine::~VenturiEngine() {
}

void VenturiEngine::start() {
    // 1. Spawn Core 1 Task (High-priority Physics/Motors)
    // Assigned to Core 1, Priority 10 (Very High)
    xTaskCreatePinnedToCore(
        VenturiEngine::vCore0TaskWrapper,
        "Engine_Core0",
        8192, // 32KB Stack allocation
        this, // Pass instance pointer
        10, // Priority
        &_core1TaskHandle,
        0 // Pinned to Core 1
    );

    // 2. Spawn Core 2 Task (High-speed ADC Data Ingestion)
    // Assigned to Core 0, Priority 9 (High)
    xTaskCreatePinnedToCore(
        VenturiEngine::vCore1TaskWrapper,
        "Engine_Core1",
        8192, // 32KB Stack allocation
        this, // Pass instance pointer
        9, // Priority
        &_core1TaskHandle,
        1 // Pinned to Core 0
    );

    xTaskCreatePinnedToCore(
        VenturiEngine::vTelemetryTaskWrapper,
        "Engine_Telemetry",
        4096,               // 16KB Stack allocation
        this,               // Pass instance pointer
        1,                  // Priority 1 (Low - Preempted by everything else)
        &_telemetryTaskHandle,
        1                   // Pinned to Core 0
    );
}

// Static wrapper transitions execution safely from C scheduler back to C++ instance
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

void VenturiEngine::vTelemetryTaskWrapper(void *pvParameters) {
    VenturiEngine* instance = static_cast<VenturiEngine*>(pvParameters);
    instance->telemetry_loop();
    vTaskDelete(NULL);
}