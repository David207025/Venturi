#include "VenturiTools/VenturiTools.h"
#include "VenturiEngine/VenturiEngine.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

class Venturi : public VenturiEngine {
public:
    VenturiTools *tools = nullptr;
    const int8_t weights[16] = {
        127, 64, 32, 16, 8, 4, 2, 0,
        0, -2, -4, -8, -16, -32, -64, -127
    };

    static volatile bool hw_timer_trigger;
    static volatile bool core_tick;
    static volatile bool system_ready;

    gptimer_handle_t samplingTimer = nullptr;

    static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                                         void *user_ctx) {
        hw_timer_trigger = true;
        return true;
    }

    Venturi() : VenturiEngine() {
        system_ready = false;
        hw_timer_trigger = false;
        core_tick = false;

        tools = new VenturiTools(2);

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << PIN_TM),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,
            .intr_priority = 1,
            .flags = {.intr_shared = false}
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &samplingTimer));

        gptimer_event_callbacks_t cbs = {.on_alarm = timer_callback};
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(samplingTimer, &cbs, NULL));

        gptimer_alarm_config_t alarm_config = {};
        alarm_config.alarm_count = 50;
        alarm_config.reload_count = 0;
        alarm_config.flags.auto_reload_on_alarm = true;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(samplingTimer, &alarm_config));

        ESP_ERROR_CHECK(gptimer_enable(samplingTimer));
        ESP_ERROR_CHECK(gptimer_start(samplingTimer));

        system_ready = true;
    }

    ~Venturi() {
        if (samplingTimer) {
            gptimer_stop(samplingTimer);
            gptimer_disable(samplingTimer);
            gptimer_del_timer(samplingTimer);
        }
        delete tools;
    }

protected:

    // --- Inside core0_loop ---
// Core 0 now owns activeWriteFrame. No mutex needed for the write.
void core0_loop() override {
    while (!system_ready) { esp_rom_delay_us(1); }
    tools->initFastHardwarePipeline();

    uint64_t start_spi = 0;

    while (true) {
        while (!hw_timer_trigger) { asm volatile("nop"); }
        hw_timer_trigger = false;

        start_spi = esp_timer_get_time();

        // Write directly into the activeWriteFrame buffers
        // Explicitly cast away the volatile qualifier for the duration of this call
        tools->performFastSPIRead(
            const_cast<uint8_t*>(VenturiTools::activeWriteFrame->adc_data),
            const_cast<uint8_t*>((uint8_t*)&VenturiTools::activeWriteFrame->gyro[0])
        );

        VenturiTools::activeWriteFrame->timestamp = (uint32_t)(esp_timer_get_time() / 1000);
        VenturiTools::activeWriteFrame->update_speed = (uint16_t)(esp_timer_get_time() - start_spi);

        // Ping-Pong: Atomically swap the pointers
        // Core 1 will see the new 'activeReadFrame' on its next iteration
        TelemetryFrame* temp = (TelemetryFrame*)VenturiTools::activeWriteFrame;
        VenturiTools::activeWriteFrame = VenturiTools::activeReadFrame;
        VenturiTools::activeReadFrame = temp;

        core_tick = true;
        taskYIELD();
    }
}

// --- Inside core1_loop ---
void core1_loop() override {
    static bool uart_initialized = false;
    if (!uart_initialized) {
        tools->initUART(921600, RX_PIN, TX_PIN, tools->jsonBuffer);
        uart_initialized = true;
    }

    while (!system_ready) { esp_rom_delay_us(1); }

    TelemetryFrame frameSnapshot;
    bool pipeline_primed = false;
    uint64_t last_send_time = esp_timer_get_time();
    const uint64_t SEND_INTERVAL_US = 10000;

    while (true) {
        while (!core_tick) { asm volatile("nop"); }
        core_tick = false;

        // Read directly from activeReadFrame (No mutex required for reading)
        if (pipeline_primed) {
            int32_t acc = 0;
            for (int i = 0; i < 16; i++) {
                acc += (int32_t)(VenturiTools::activeReadFrame->adc_data[i] * weights[i]);
            }
            // Note: If you need to store steering, consider moving it to a non-shared member
            // or perform the calculation on the local snapshot if the frame is swapped.
        }
        pipeline_primed = true;

        uint64_t now = esp_timer_get_time();
        if (now - last_send_time >= SEND_INTERVAL_US) {
            last_send_time = now;

            // Snapshot the read-frame for transmission
            memcpy(&frameSnapshot, (void*)VenturiTools::activeReadFrame, sizeof(TelemetryFrame));
            frameSnapshot.temperature_c = tools->getTemperature();
            tools->sendFrame(&frameSnapshot);
        }
        taskYIELD();
    }
}
};

volatile bool Venturi::hw_timer_trigger = false;
volatile bool Venturi::core_tick = false;
volatile bool Venturi::system_ready = false;


extern "C" void app_main(void) {
    Venturi *app = new Venturi();
    app->start();
    vTaskDelete(NULL);
}
