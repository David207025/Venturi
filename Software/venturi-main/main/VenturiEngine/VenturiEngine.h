#ifndef VENTURI_ENGINE_H
#define VENTURI_ENGINE_H

#include <driver/spi_master.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// THIS IS WHAT RUST EXPECTS:
struct __attribute__((packed)) TelemetryFrame {
    uint32_t timestamp;      // Offset 0  (4-byte aligned)
    int32_t  steering;       // Offset 4  (4-byte aligned)
    float    temperature_c;  // Offset 8  (4-byte aligned)
    int16_t  gyro[3];        // Offset 12 (2-byte aligned) -> 6 bytes
    int16_t  accel[3];       // Offset 18 (2-byte aligned) -> 6 bytes
    uint16_t battery_mv;     // Offset 24 (2-byte aligned)
    uint16_t update_speed;   // Offset 26 (2-byte aligned)
    uint8_t  adc_data[16];   // Offset 28 (1-byte aligned) -> 16 bytes

}; // Total: 44 Bytes. Zero hidden padding holes!

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

    static volatile bool frameReady;

    virtual void core0_loop() = 0;

    virtual void core1_loop() = 0;


private:


    // Static FreeRTOS task wrappers
    static void vCore0TaskWrapper(void *pvParameters);

    static void vCore1TaskWrapper(void *pvParameters);

};

#endif
