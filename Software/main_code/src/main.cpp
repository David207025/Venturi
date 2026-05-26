#include "VenturiEngine/VenturiEngine.h"
#include "VenturiTools/VenturiTools.h" // Your original tools class
#include <Arduino.h>

class Venturi : public VenturiEngine {
private:
    VenturiTools* tools;

    // Double-buffering array layout to isolate execution threads across cores
    volatile TelemetryFrame bufferPool[2];
    volatile uint8_t writeIdx;
    volatile uint8_t readIdx;

    void printFrameDebug(const TelemetryFrame& frame) {
        uint8_t* bytePtr = (uint8_t*)&frame;
        size_t frameSize = sizeof(TelemetryFrame);

        Serial.printf("[TX Debug] Size: %d Bytes | Data: [", frameSize);
        for (size_t i = 0; i < frameSize; i++) {
            Serial.printf("%02X", bytePtr[i]);
            if (i < frameSize - 1) {
                Serial.print(" ");
            }
        }
        Serial.println("]");
    }

public:
    Venturi() : VenturiEngine() {
        tools = new VenturiTools("Venturi_P4", 2);

        // Clear out the memory pool and initialize dual tracking pointers
        memset((void*)bufferPool, 0, sizeof(bufferPool));
        writeIdx = 0;
        readIdx = 1;


    }

protected:
    // ─── CORE 0: HIGH-SPEED SENSOR INGESTION (PRODUCER) ───
    void core0_loop() override {
        while (true) {
            // Point to our isolated front active write-buffer
            volatile TelemetryFrame* frontBuffer = &bufferPool[writeIdx];

            // 1. Read ADS7961SDBT over 20MHz SPI
            // readSensorsOverSPI((uint8_t*)frontBuffer->adc_data);

            // 2. Update data structure snapshot (Dummy line data matching your simulation profile)
            frontBuffer->timestamp = millis();
            // (Populate IMU gyro/accel and battery data here too)

            for(int i = 0; i < 8; i++) {
                frontBuffer->adc_data[i] = 30 * i;
                frontBuffer->adc_data[15 - i] = 30 * i;
            }

            // Simulates raw FPU processing delay on Core 1
            volatile float simulatedStress = 1.234f;
            for (int i = 0; i < 100; i++) {
                simulatedStress = (simulatedStress * 1.001f) + 0.005f;
            }


            // 3. Atomically flip the buffer pool indices so Core 1 reads our completed dataset
            uint8_t temp = writeIdx;
            writeIdx = readIdx;
            readIdx = temp;

            // 4. BUMP CORE 1: Wake up the motor control thread instantly
            if (_core1TaskHandle != NULL) {
                xTaskNotifyGive(_core1TaskHandle);
            }

            // Yield control briefly to give Core 0's low-priority internal
            // BLE radio stack a chance to sweep network registers
            //taskYIELD();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // ─── CORE 1: PD ENGINE & MOTOR CONTROL (CONSUMER) ───
    void core1_loop() override {
        while (true) {
            // SLEEP STATE: Relinquishes 100% of Core 1's processing time.
            // Awakens in fractions of a microsecond the exact instant Core 0 triggers a notification.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Safely bind to the stable read back-buffer
            volatile TelemetryFrame* activeFrame = &bufferPool[readIdx];

            // ─── Run high-priority PD Engine & Motor Control here ───
            // float lineError = calculateLinePosition(activeFrame->adc_data);
            // runMotorControlMath(lineError, activeFrame->gyro, activeFrame->accel);

            // Once finished, this loop naturally cycles back up and blocks on ulTaskNotifyTake,
            // immediately freeing up Core 1 for background telemetry processing.
        }
    }

    // ─── CORE 1 BACKGROUND PROCESSING: TELEMETRY STREAM ───
    void telemetry_loop() override {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // Throttled to ~40Hz (Every 25ms). Because this task runs at Priority 1 on Core 1,
        // it will execute quietly whenever core1_loop is asleep waiting for a sensor bump.
        const TickType_t xFrequency = pdMS_TO_TICKS(1);

        while (true) {
            //vTaskDelayUntil(&xLastWakeTime, xFrequency);

            // Take a local copy snapshot of the current backbuffer frame to prevent any tearing
            TelemetryFrame frameSnapshot = {};
            memcpy(&frameSnapshot, (void*)&bufferPool[readIdx], sizeof(TelemetryFrame));

            // Print the raw bytes to the Serial monitor if debugging
            // printFrameDebug(frameSnapshot);

            // Pass the packed struct into your BLE transmission tool
            tools->writeBLE<TelemetryFrame>(frameSnapshot);

            vTaskDelay(1);
        }
    }
};

// Standard Arduino entry hooks to prevent duplicate app_main links
void setup() {
    Serial.begin(460800);
    delay(500);
    Serial.println(">>> Initializing Venturi Engine Multitasking System...");

    // Allocate your customized car controller engine onto the heap
    Venturi* car = new Venturi();

    // Fire up your FreeRTOS tasks pinned across Core 0 and Core 1
    car->start();

    // Reclaim memory: Delete the temporary Arduino loop task setup thread
    Serial.println(">>> Engine active. Terminating initialization thread.");
    vTaskDelete(NULL);
}

void loop() {
    // This loop is now dead code and will NEVER be reached because
    // vTaskDelete(NULL) terminated this thread. 0% CPU wasted here.
}