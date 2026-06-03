#ifndef VENTURI_ENGINE_H
#define VENTURI_ENGINE_H

#include <Arduino.h>
#include <driver/spi_master.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// THIS IS WHAT RUST EXPECTS:
struct __attribute__((packed)) TelemetryFrame {
    uint32_t timestamp;      // 4 bytes
    uint8_t  adc_data[16];   // 16 bytes
    int16_t  gyro[3];        // 6 bytes
    int16_t  accel[3];       // 6 bytes
    uint16_t battery_mv;     // 2 bytes
    int32_t  steering;       // Add this (4 bytes) - Ensure Rust struct matches!
    float temperature_c;
    uint16_t update_speed;
};

class VenturiEngine {
public:
    VenturiEngine();

    virtual ~VenturiEngine();

    // Starts both FreeRTOS schedulers on their respective cores
    void start();

protected:
    static VenturiEngine* _instance;

    static TaskHandle_t _core0TaskHandle;
    static TaskHandle_t _core1TaskHandle;



    static void IRAM_ATTR timerCallback(); // The static ISR

    static volatile bool frameReady;

    virtual void core0_loop() = 0;

    virtual void core1_loop() = 0;


private:


    // Static FreeRTOS task wrappers
    static void vCore0TaskWrapper(void *pvParameters);

    static void vCore1TaskWrapper(void *pvParameters);

};

#endif
