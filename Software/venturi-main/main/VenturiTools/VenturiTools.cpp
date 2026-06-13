#include "VenturiTools.h"

#include "rom/ets_sys.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

static const char *TAG = "VenturiTools";

volatile bool VenturiTools::s_touchFlags[16] = {false};
volatile bool VenturiTools::s_fingerOnPad[16] = {false};
TaskHandle_t VenturiTools::taskToNotify = NULL;
volatile TelemetryFrame VenturiTools::frameA;
volatile TelemetryFrame VenturiTools::frameB;
volatile TelemetryFrame* VenturiTools::activeWriteFrame = &VenturiTools::frameA;
volatile TelemetryFrame* VenturiTools::activeReadFrame = &VenturiTools::frameB;
portMUX_TYPE VenturiTools::bufferMutex = portMUX_INITIALIZER_UNLOCKED;
// Native ESP-IDF v5+ Touch ISR Filter Callback
bool VenturiTools::touch_filter_cb(touch_sensor_handle_t handle, const touch_active_event_data_t *event_data,
                                   void *user_ctx) {
    int pin_num = (int) user_ctx;
    if (pin_num >= 2 && pin_num <= 14) {
        uint8_t arrayIdx = pin_num - 2;
        if (!s_fingerOnPad[arrayIdx]) {
            s_touchFlags[arrayIdx] = true;
            s_fingerOnPad[arrayIdx] = true;
        }
    }
    return false;
}

VenturiTools::VenturiTools(int debug_level) : debugLevel(debug_level), tempHandle(NULL),
                                              temperatureSensorAvailable(false),
                                              spi_imu(NULL), spi_adc(NULL), card(NULL), sdCardMounted(false) {
    ESP_LOGI(TAG, "Initializing Engine Components. Debug Level: %d", debugLevel);
    initSD();
    initSPI();


    // Read config JSON bare-metal style from filesystem
    char *json_buffer = nullptr;
    FILE *f = fopen("/sdcard/network.json", "r");
    if (f != NULL) {
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        fseek(f, 0, SEEK_SET);

        json_buffer = (char *) malloc(fsize + 1);
        if (json_buffer) {
            fread(json_buffer, 1, fsize, f);
            json_buffer[fsize] = '\0';
        }
        fclose(f);

        jsonBuffer = (char*)malloc(fsize + 1);
        if (jsonBuffer && json_buffer) {
            memcpy(jsonBuffer, json_buffer, fsize);
            jsonBuffer[fsize] = '\0';
        }

        if (json_buffer) free(json_buffer);
    }



    temperature_sensor_config_t tempSensorConfig = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
    if (temperature_sensor_install(&tempSensorConfig, &tempHandle) == ESP_OK) {
        temperature_sensor_enable(tempHandle);
        temperatureSensorAvailable = true;
    } else {
        ESP_LOGE(TAG, "Failed to install hardware temperature sensor.");
    }

    if (debugLevel > 1) {
        ESP_LOGI(TAG, ">>> Warm-up completed. Settling peripheral registers...");
    }
}

VenturiTools::~VenturiTools() {
    if (sdCardMounted) {
        esp_vfs_fat_sdcard_unmount("/sdcard", card);
    }
    if (temperatureSensorAvailable && tempHandle) {
        temperature_sensor_disable(tempHandle);
        temperature_sensor_uninstall(tempHandle);
    }
}

void VenturiTools::setTaskNotificationHandle(TaskHandle_t taskHandle) {
    taskToNotify = taskHandle;
}

void VenturiTools::initUART(int baudRate, int txPin, int rxPin, const char *configJson) {
    uart_config_t uart_config = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 4096, 4096, 0, NULL, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Bare-metal pin initialization replaces Arduino pinMode
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_UDPR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (gpio_get_level((gpio_num_t) PIN_UDPR) == 1) {
        esp_rom_delay_us(10000); // Strict, low-level microsecond block (10ms)
    }

    if (configJson && strlen(configJson) > 0) {
        uart_write_bytes(UART_PORT, configJson, strlen(configJson));
        uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, ">>> UART Initialized. Config JSON sent");
}

void VenturiTools::sendFrame(const TelemetryFrame *frame) {
    uint8_t header[3] = {0xAA, 0x00, 0xAA};

    if (gpio_get_level((gpio_num_t)PIN_UDPR) == 0) {
        // Use non-blocking write. If the UART driver is using DMA,
        // this will return immediately after queuing the data.
        uart_write_bytes(UART_PORT, (const char *)header, 3);
        uart_write_bytes(UART_PORT, (const char *)frame, sizeof(TelemetryFrame));

        // REMOVE uart_wait_tx_done!
        // Let the UART peripheral/DMA handle the shifting in the background.
    }
}

#include "driver/gpio.h"
#include "esp_ldo_regulator.h" // Required for ESP-IDF v5.3+ LDO management

#define SDMMC_PIN_CMD 44
#define SDMMC_PIN_CLK 43
#define SDMMC_PIN_D0  39
#define SDMMC_PIN_D1  40
#define SDMMC_PIN_D2  41
#define SDMMC_PIN_D3  42

void VenturiTools::initSD() {
    ESP_LOGI("VenturiTools", "Initializing Native SDMMC Card Host...");

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = 20000;

    // ==========================================
    // STEP 1: INITIALIZE ON-CHIP LDO FOR SD P4 IO
    // ==========================================
    // On the ESP32-P4, LDO_VO4 is typically dedicated to the SDMMC IO domain (Channel ID 4)
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    esp_err_t pwr_ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (pwr_ret != ESP_OK) {
        ESP_LOGE("VenturiTools", "Failed to power up the ESP32-P4 internal SD LDO domain!");
        return;
    }
    // Bind the active power handle to the host driver configuration
    host.pwr_ctrl_handle = pwr_ctrl_handle;
    // ==========================================

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = (gpio_num_t)SDMMC_PIN_CLK;
    slot_config.cmd = (gpio_num_t)SDMMC_PIN_CMD;
    slot_config.d0  = (gpio_num_t)SDMMC_PIN_D0;
    slot_config.d1  = (gpio_num_t)SDMMC_PIN_D1;
    slot_config.d2  = (gpio_num_t)SDMMC_PIN_D2;
    slot_config.d3  = (gpio_num_t)SDMMC_PIN_D3;
    slot_config.width = 4;

    // Apply the reference internal pull-up fallback flag
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // Mount filesystem
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE("VenturiTools", "SD-Card Mount Timeout (0x107). Check your pull-up resistors.");
        } else {
            ESP_LOGE("VenturiTools", "Failed to initialize SD card over SDMMC Slot (%s).", esp_err_to_name(ret));
        }
        return;
    }

    if (debugLevel > 1) {
        ESP_LOGI("VenturiTools", ">>> SD_MMC Card successfully mounted!");
        ESP_LOGI("VenturiTools", ">>> Name: %s", card->cid.name);
        ESP_LOGI("VenturiTools", ">>> Capacity: %llu MB", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
    }
}


void VenturiTools::initSPI() {
    // 1. ADC Bus Initialization (SPI2)
    spi_bus_config_t adcbuscfg = {
        .mosi_io_num = PIN_ADC_MOSI,
        .miso_io_num = PIN_ADC_MISO,
        .sclk_io_num = PIN_ADC_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &adcbuscfg, SPI_DMA_DISABLED));

    // Set pull-down on ADC MISO
    gpio_set_pull_mode((gpio_num_t)PIN_ADC_MISO, GPIO_PULLDOWN_ONLY);

    spi_device_interface_config_t adc_devcfg = {
        .mode = 0,
        .clock_speed_hz = 20000000,
        .spics_io_num = -1,
        .queue_size = 5
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &adc_devcfg, &spi_adc));

    // 2. IMU Bus Initialization (SPI3)
    spi_bus_config_t imubuscfg = {
        .mosi_io_num = PIN_IMU_MOSI,
        .miso_io_num = PIN_IMU_MISO,
        .sclk_io_num = PIN_IMU_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &imubuscfg, SPI_DMA_DISABLED));

    // Set pull-down on IMU MISO
    gpio_set_pull_mode((gpio_num_t)PIN_IMU_MISO, GPIO_PULLDOWN_ONLY);

    spi_device_interface_config_t imu_devcfg = {
        .mode = 0,
        .clock_speed_hz = 20000000,
        .spics_io_num = PIN_IMU_CS,
        .queue_size = 5
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &imu_devcfg, &spi_imu));

    // 3. Configure Manual ADC CS
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_ADC_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)PIN_ADC_CS, 1);
}

void VenturiTools::initFastHardwarePipeline() {

    _spi_adc_hw = &GPSPI2;
    _spi_imu_hw = &GPSPI3;

    _spi_imu_hw->user.usr_miso = 1;
    _spi_imu_hw->user.usr_mosi = 1;
    _spi_adc_hw->user.usr_miso = 1;
    _spi_adc_hw->user.usr_mosi = 1;

    spi_transaction_t dummy_init_trans = {};
    dummy_init_trans.length = 8;
    dummy_init_trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    spi_device_transmit(spi_adc, &dummy_init_trans);
    spi_device_transmit(spi_imu, &dummy_init_trans);

    _spi_adc_hw->ms_dlen.ms_data_bitlen = 16 - 1;

    uint32_t ads_auto2_sequence[4] = {
        (uint32_t) 0x1000 << 16,
        (uint32_t) 0x1000 << 16,
        (uint32_t) 0x93C0 << 16,
        (uint32_t) 0x3000 << 16
    };

    for (int i = 0; i < 4; i++) {
        GPIO.out_w1tc.val = (1ULL << PIN_ADC_CS);
        _spi_adc_hw->data_buf[0].val = ads_auto2_sequence[i];
        _spi_adc_hw->cmd.update = 1;
        _spi_adc_hw->cmd.usr = 1;
        while (_spi_adc_hw->cmd.usr);
        GPIO.out_w1ts.val = (1ULL << PIN_ADC_CS);
        esp_rom_delay_us(2);
    }

    _spi_imu_hw->ms_dlen.ms_data_bitlen = 104 - 1;
    _spi_adc_hw->ms_dlen.ms_data_bitlen = 16 - 1;

#pragma GCC unroll 4
    for (int i = 0; i < 16; i++) {
        _spi_adc_hw->data_buf[i].val = 0x00000000;
    }

    _spi_imu_hw->cmd.update = 1;
    _spi_adc_hw->cmd.update = 1;
}

void VenturiTools::performFastSPIRead(uint8_t *adc_dest, uint8_t *imu_dest) {
    // 1. Trigger IMU
    taskENTER_CRITICAL(&bufferMutex);
    _spi_imu_hw->cmd.usr = 1;

    // 2. ADC Pipelined Read
#pragma GCC unroll 4
    for (int i = 0; i < 16; i++) {
        GPIO.out_w1tc.val = (1ULL << PIN_ADC_CS); // CS Low

        // Trigger the ADC conversion/read
        _spi_adc_hw->cmd.usr = 1;
        while (_spi_adc_hw->cmd.usr); // Wait for the 16 bits

        // CS High - Reset ADC state machine for next channel
        GPIO.out_w1ts.val = (1ULL << PIN_ADC_CS);

        // Shift by 4 to align bits 11-4 to the LSB
        adc_dest[i] = (uint8_t)((_spi_adc_hw->data_buf[0].val >> 4) & 0xFF);
    }


    uint32_t *imu_ptr = (uint32_t*)imu_dest;
    uint32_t *raw_buf = (uint32_t*)&_spi_imu_hw->data_buf[0].val;
    imu_ptr[0] = raw_buf[0];
    imu_ptr[1] = raw_buf[1];
    imu_ptr[2] = raw_buf[2];
    taskEXIT_CRITICAL(&bufferMutex);
}

void VenturiTools::initTouchChannel(uint8_t pin, uint16_t threshold) {
    if (pin < 2 || pin > 14) return;
    uint8_t arrayIdx = pin - 2;

    // 1. Initialize the master core controller if it does not exist yet
    if (global_touch_controller == NULL) {
        // Create a single default hardware sample profile
        // Adjust V3 vs V2 macro names if your specific framework micro-release flags it
#define SAMPLE_NUM 1
        touch_sensor_sample_config_t sample_cfg[SAMPLE_NUM] = {
            TOUCH_SENSOR_V3_DEFAULT_SAMPLE_CONFIG(1000, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2)
        };

        // Initialize using the base structure layout
        touch_sensor_config_t touch_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(SAMPLE_NUM, sample_cfg);

        ESP_ERROR_CHECK(touch_sensor_new_controller(&touch_cfg, &global_touch_controller));
    }

    // 2. Map your parameter threshold variable directly into the configuration array
    touch_channel_config_t chan_cfg = {
        .active_thresh = { threshold }
    };

    // 3. Complete the physical channel allocation
    ESP_ERROR_CHECK(touch_sensor_new_channel(global_touch_controller, pin, &chan_cfg, &touchHandle[arrayIdx]));
}

void VenturiTools::initTouchChannelInterrupt(uint8_t pin) {
    if (pin < 2 || pin > 14) return;
    uint8_t arrayIdx = pin - 2;

    s_touchFlags[arrayIdx] = false;
    s_fingerOnPad[arrayIdx] = false;

    // 1. Setup the callback wrappers
    touch_event_callbacks_t callbacks = {
        .on_active = touch_filter_cb,
        .on_inactive = NULL
    };

    // 2. FIXED ARGUMENT: Pass global_touch_controller (touch_sensor_handle_t) here, NOT touchHandle[arrayIdx]
    // The driver registers callbacks globally onto the master module configuration engine.
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(global_touch_controller, &callbacks, (void*)(intptr_t)pin));

    // 3. Enable and fire up the scanning engine safely
    static bool controller_enabled = false;
    if (!controller_enabled) {
        ESP_ERROR_CHECK(touch_sensor_enable(global_touch_controller));

        // FIXED SYMBOL: Full string macro identifier mapping
        ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(global_touch_controller));

        controller_enabled = true;
    }
}

bool VenturiTools::isChannelTouched(uint8_t pin) {
    if (pin < 2 || pin > 14) return false;
    uint8_t arrayIdx = pin - 2;

    if (s_touchFlags[arrayIdx]) {
        s_touchFlags[arrayIdx] = false;
        s_fingerOnPad[arrayIdx] = false;
        return true;
    }
    return false;
}

FILE *VenturiTools::openFile(const char *filename, const char *mode) {
    char path_buf[128];
    snprintf(path_buf, sizeof(path_buf), "/sdcard%s", filename);
    return fopen(path_buf, mode);
}

void VenturiTools::reloadSD() {
    if (sdCardMounted) {
        esp_vfs_fat_sdcard_unmount("/sdcard", card);
        sdCardMounted = false;
    }
    esp_rom_delay_us(100000); // 100ms
    initSD();
}

float VenturiTools::getTemperature() {
    float temperature = -1.0f;
    if (temperature_sensor_get_celsius(tempHandle, &temperature) == ESP_OK) {
        return temperature;
    }
    return -1.0f;
}
