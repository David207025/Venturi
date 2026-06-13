#ifndef VENTURITOOLS_H
#define VENTURITOOLS_H

#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/temperature_sensor.h"
#include "driver/touch_sens.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "soc/spi_struct.h"
#include "soc/gpio_struct.h"
#include "VenturiEngine/VenturiEngine.h"

#define UART_PORT     UART_NUM_1
#define TX_PIN        18
#define RX_PIN        19
#define UART_BUF_SIZE 2048

#define PIN_TM        16
#define PIN_UDPR      17

#define PIN_ADC_CS    2
#define PIN_ADC_CLK   3
#define PIN_ADC_MOSI  4
#define PIN_ADC_MISO  5

#define PIN_IMU_CS    23
#define PIN_IMU_CLK   22
#define PIN_IMU_MOSI  21
#define PIN_IMU_MISO  20
#define PIN_IMU_INT   51

class VenturiTools {
private:
    int debugLevel;
    temperature_sensor_handle_t tempHandle;
    bool temperatureSensorAvailable;

    spi_device_handle_t spi_imu;
    spi_device_handle_t spi_adc;

    sdmmc_card_t *card;
    bool sdCardMounted;

    static TaskHandle_t taskToNotify;
    static volatile bool s_touchFlags[16];
    static volatile bool s_fingerOnPad[16];


    touch_sensor_handle_t global_touch_controller;
    // Native Touch structures
    touch_channel_handle_t touchHandle[15];

    static bool touch_filter_cb(touch_sensor_handle_t handle, const touch_active_event_data_t *event_data,
                                          void *user_ctx);

    void initSD();

public:
    char* jsonBuffer;
    static portMUX_TYPE bufferMutex;

    static volatile TelemetryFrame frameA, frameB;
    static volatile TelemetryFrame* activeWriteFrame;
    static volatile TelemetryFrame* activeReadFrame;

    spi_dev_t *_spi_imu_hw;
    spi_dev_t *_spi_adc_hw;

    VenturiTools(int debug_level = 0);

    ~VenturiTools();

    static void initUART(int baudRate, int txPin, int rxPin, const char *configJson);

    static void sendFrame(const TelemetryFrame *frame);

    void initSPI();

    void initFastHardwarePipeline();

    void performFastSPIRead(uint8_t *adc_buf, uint8_t *imu_buf);

    void setTaskNotificationHandle(TaskHandle_t taskHandle);

    void initTouchChannel(uint8_t pin, uint16_t threshold);

    void initTouchChannelInterrupt(uint8_t pin);

    bool isChannelTouched(uint8_t pin);

    FILE *openFile(const char *filename, const char *mode);

    void reloadSD();

    float getTemperature();
};

#endif // VENTURITOOLS_H
