#include "VenturiEngine/VenturiEngine.h"
#include "VenturiTools/VenturiTools.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Network.h>

static TaskHandle_t globalCore0TaskHandle = NULL;

class Venturi : public VenturiEngine {
public:
    VenturiTools *tools;
    static void IRAM_ATTR onTimerTick();

    const int8_t weights[16] = {
        127, 64, 32, 16, 8, 4, 2, 0,
        0, -2, -4, -8, -16, -32, -64, -127
    };

    volatile int32_t steering = 0;

    Venturi() : VenturiEngine() {
        tools = new VenturiTools("Venturi_P4", 2, true);
        tools->autoConnectWiFi();

        VenturiTools::buffer = new TelemetryFrame();
        memset((void *) VenturiTools::buffer, 0, sizeof(TelemetryFrame));
    }

protected:
    // ====================================================================
    // CORE 0 LOOP: Dedicated strictly to Network, WebServer, and UDP Streaming
    // ====================================================================
    void core0_loop() override {
        _core0TaskHandle = xTaskGetCurrentTaskHandle();
        esp_task_wdt_add(_core0TaskHandle);

        while (true) {
            esp_task_wdt_reset();

            if (WiFi.status() == WL_CONNECTED) {
                TelemetryFrame frameSnapshot;

                // Disable interrupts briefly to capture a clean memory snapshot
                portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
                portENTER_CRITICAL(&myMutex);
                memcpy(&frameSnapshot, (void *) VenturiTools::buffer, sizeof(TelemetryFrame));
                portEXIT_CRITICAL(&myMutex);

                // Fetch temperature directly into the frame copy
                frameSnapshot.temperature_c = tools->getTemperature();

                tools->streamUDP(frameSnapshot);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

// ====================================================================
    // CORE 1 LOOP: Your Ultra-Fast Control & Math Engine
    // ====================================================================
    void core1_loop() override {
        _core1TaskHandle = xTaskGetCurrentTaskHandle();
        globalCore0TaskHandle = _core1TaskHandle; // Point our ISR notification directly to Core 1!

        // Disable the watchdog for Core 1 so your high-speed loop can dominate the CPU
        esp_task_wdt_delete(_core1TaskHandle);

        if (VenturiTools::buffer == nullptr) {
            VenturiTools::buffer = new TelemetryFrame();
            memset((void *) VenturiTools::buffer, 0, sizeof(TelemetryFrame));
        }

        static uint16_t proc_adc[16] = {0};
        static uint8_t  proc_imu[12] = {0};

        // Initialize and lock the SPI hardware channels on Core 1's memory space
        tools->initFastHardwarePipeline();

        // --- Hardware Timer Initialization (Targeting 20us on Core 1) ---
        hw_timer_t *timer = timerBegin(1000000);
        timerAttachInterrupt(timer, &onTimerTick);
        timerAlarm(timer, 20, true, 0); // Fires exactly every 20 microseconds

        uint32_t delta_math_us = 0;
        uint32_t delta_execution_us = 0;
        bool pipeline_primed = false;

        // Variables used for the 1-second rate-limited Serial print
        uint64_t last_print_time = esp_timer_get_time();

        while (true) {
            // Unblock as soon as the timer interrupt fires
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            uint64_t loop_start = esp_timer_get_time();

            gpio_set_level(GPIO_NUM_29, 1);

            // 1. Kick off the asynchronous DMA read sequence
            tools->startSPIReads();

            // 2. Process math calculations for the data gathered in the last frame
            uint64_t math_start = esp_timer_get_time();

            if (pipeline_primed) {
                // Explicitly copy into separate positions to prevent overflowing memory blocks
                memcpy((void *) &VenturiTools::buffer->gyro[0], proc_imu, 6);
                memcpy((void *) &VenturiTools::buffer->accel[0], proc_imu + 6, 6);

                int32_t acc = 0;
                for (int i = 0; i < 16; i++) {
                    uint8_t compressed_val = (uint8_t) (proc_adc[i] >> 4);
                    VenturiTools::buffer->adc_data[i] = compressed_val;
                    acc += (int32_t) compressed_val * weights[i];
                }

                // Fixed: Explicitly write local calculation 'acc' to the telemetry frame buffer
                VenturiTools::buffer->steering = acc;

                esp_rom_delay_us(2);

                VenturiTools::buffer->timestamp = (uint32_t) (loop_start / 1000);
            }

            delta_math_us = (uint32_t) (esp_timer_get_time() - math_start);

            // 3. Complete data collection via direct register polling
            tools->getSPIResults(proc_adc, proc_imu);
            pipeline_primed = true;

            delta_execution_us = (uint32_t) (esp_timer_get_time() - loop_start);
            gpio_set_level(GPIO_NUM_29, 0);

            // Ship out the exact microsecond value to the buffer array
            VenturiTools::buffer->update_speed = (uint16_t) delta_execution_us;

        }
    }
};

// ====================================================================
// TIMER INTERRUPTION FUNCTION (Now targeting Core 1 Context)
// ====================================================================
void IRAM_ATTR Venturi::onTimerTick() {
    if (globalCore0TaskHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // This wakes up the task handle pinned to globalCore0TaskHandle (which is now Core 1)
        vTaskNotifyGiveFromISR(globalCore0TaskHandle, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    gpio_reset_pin(GPIO_NUM_29);
    gpio_set_direction(GPIO_NUM_29, GPIO_MODE_OUTPUT);

    Serial.println(">>> Initializing Venturi Engine Multitasking System...");

    Venturi *car = new Venturi();
    car->start();
    car->tools->enableOtaListening();
}

void loop() {}