#ifndef VENTURI_ENGINE_H
#define VENTURI_ENGINE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// THIS IS WHAT RUST EXPECTS:
struct __attribute__((packed)) TelemetryFrame {
    uint32_t timestamp;      // 4 bytes
    uint8_t  adc_data[16];   // 16 bytes
    int16_t  gyro[3];        // 6 bytes
    int16_t  accel[3];       // 6 bytes
    uint16_t battery_mv;     // 2 bytes
};

class VenturiEngine {
public:
    VenturiEngine();

    virtual ~VenturiEngine();

    // Starts both FreeRTOS schedulers on their respective cores
    void start();

protected:
    TaskHandle_t _core0TaskHandle = NULL;
    TaskHandle_t _core1TaskHandle = NULL;
    TaskHandle_t _telemetryTaskHandle = NULL;

    virtual void core0_loop() = 0;

    virtual void core1_loop() = 0;

    virtual void telemetry_loop() = 0;

private:


    // Static FreeRTOS task wrappers
    static void vCore0TaskWrapper(void *pvParameters);

    static void vCore1TaskWrapper(void *pvParameters);

    static void vTelemetryTaskWrapper(void *pvParameters);
};

#endif
